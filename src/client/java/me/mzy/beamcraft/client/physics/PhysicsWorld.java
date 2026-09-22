package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import me.mzy.beamcraft.network.VehicleSyncPayload;

import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.IntStream;

/**
 * Core physical world controller for beam-based vehicle simulation
 * Manages nodes, beams, collision caching and physics integration
 */
public class PhysicsWorld {
    private static final int CANDIDATE_MESHLETS_PER_TASK = 4;
    private static final ExecutorService EVENT_TRACE_WRITER = Executors.newSingleThreadExecutor(task -> {
        Thread thread = new Thread(task, "BeamCraft physics trace writer");
        thread.setDaemon(true);
        return thread;
    });
    public static final float GRAVITY = -9.81f;
    public static final float SOUND_SPEED = 340.0f;
    public static final float BLOCK_REBOUND = 0.0f;
    public static final float BLOCK_FRICTION = 1.0f;
    public static final float KINDA_SMALL_NUMBER = 1e-8f;
    public static final float KINDA_BIG_NUMBER = 1e8f;
    public static final int MAX_AABB_SIZE = 10;
    public static final float invPhysicsDT = 2000.0f;
    /** Publish one render snapshot per this many 2000 Hz physics substeps. */
    public static final int RENDER_SNAPSHOT_SUBSTEP_INTERVAL = 10;
    /** Refresh cross-thread electric inputs at 200 Hz of simulated time. */
    public static final int ELECTRIC_SNAPSHOT_SUBSTEP_INTERVAL = 10;

    public final VoxelSnapshot voxelSnapshot = new VoxelSnapshot();
    BlockPos.Mutable mutablePos = new BlockPos.Mutable();

    public final DynamicAxisSweep globalSap = new DynamicAxisSweep();
    public final SoftBodyCollisionManager collisionManager = new SoftBodyCollisionManager();
    private final PhysicsEventTrace eventTrace = new PhysicsEventTrace();
    private Path eventTraceDirectory;
    private volatile boolean eventTraceWriteInFlight;
    private CollisionCandidateBuffer[] candidateTaskBuffers = new CollisionCandidateBuffer[0];
    private SoftBodyVehicle[] candidateTaskVehicles = new SoftBodyVehicle[0];
    private int[] candidateTaskMeshletStarts = new int[0];
    private int[] candidateTaskMeshletEnds = new int[0];

    /** Shared collision pipeline: candidate generation, soft-contact solving and environment collision. */
    public final CollisionPipeline collisionPipeline = new CollisionPipeline(voxelSnapshot, globalSap, collisionManager);

    private int nextVehicleId = 0;

    public final java.util.List<SoftBodyVehicle> vehicles = new java.util.concurrent.CopyOnWriteArrayList<>();

    public PhysicsWorld() {
        // Empty constructor, data will be injected by JBeam parser
    }

    public void configureEventTrace(boolean enabled, Path outputDirectory) {
        eventTrace.configure(enabled);
        eventTraceDirectory = outputDirectory;
        if (enabled) {
            BeamCraft.LOGGER.info("BeamCraft manual substep trace enabled; use its key binding to start and stop");
        }
    }

    public boolean eventTraceAvailable() {
        return eventTrace.available();
    }

    public boolean eventTraceRecording() {
        return eventTrace.enabled();
    }

    /** Called at the client tick barrier, while no physics job is in flight. */
    public boolean toggleEventTrace() {
        if (!eventTrace.available()) return false;
        if (!eventTrace.enabled()) {
            if (eventTraceWriteInFlight) {
                BeamCraft.LOGGER.warn("BeamCraft physics trace is still being written; start ignored");
                return false;
            }
            eventTrace.start();
            BeamCraft.LOGGER.info("BeamCraft manual physics trace recording started");
            return true;
        }

        PhysicsEventTrace.CompletedCapture capture = eventTrace.stop();
        if (capture != null) {
            BeamCraft.LOGGER.info("BeamCraft manual physics trace recording stopped ({} retained samples)",
                    capture.retainedSamples());
            eventTraceWriteInFlight = true;
            persistEventTrace(capture);
        }
        return false;
    }

    public void addVehicle(SoftBodyVehicle vehicle) {
        if (vehicle == null || vehicles.contains(vehicle)) return;

        vehicle.vehicleId = nextVehicleId++;
        vehicles.add(vehicle);
    }

    /**
     * Remove a vehicle and release its owned SoA buffers.
     */
    public void removeVehicle(SoftBodyVehicle vehicle) {
        if (vehicle == null || !vehicles.contains(vehicle)) return;

        vehicles.remove(vehicle);

        vehicle.clear();

        System.out.println("Vehicle removed safely. ID: " + vehicle.vehicleId);

    }

    public void clear() {
        vehicles.clear();
        collisionManager.clearContacts();
        System.out.println("Physics world data cleared");
    }

    /**
     * Captures every piece of Minecraft-owned state needed by one physics step.
     * This method must run on the client thread, after the previous prepared
     * step has completed.
     */
    public PreparedStep prepareStep(World mcWorld, double dt) {
        long startedNanos = System.nanoTime();
        List<SoftBodyVehicle> activeVehicles = List.copyOf(vehicles);
        int subSteps = (int)Math.ceil(dt * invPhysicsDT);

        voxelSnapshot.clear();
        for (SoftBodyVehicle vehicle : activeVehicles) {
            vehicle.cacheEntityLocation();
            vehicle.updateVoxelSnapshot(mcWorld, voxelSnapshot, mutablePos, dt);
        }

        int renderSnapshotCount = 1 + Math.ceilDiv(subSteps, RENDER_SNAPSHOT_SUBSTEP_INTERVAL);
        long stepDurationNanos = Math.round(dt * 1_000_000_000.0);
        List<PhysicsRenderTimeline.Writer> renderWriters = new ArrayList<>(activeVehicles.size());
        List<ElectricSnapshot> electricSnapshots = new ArrayList<>(activeVehicles.size());
        for (SoftBodyVehicle vehicle : activeVehicles) {
            ElectricSnapshot electricSnapshot = vehicle.electrics.snapshot();
            vehicle.driverInputs.latchTargets(electricSnapshot);
            electricSnapshots.add(electricSnapshot);
            NodeContainer nodes = vehicle.nodes;
            renderWriters.add(vehicle.renderTimeline.beginStep(
                    startedNanos,
                    stepDurationNanos,
                    renderSnapshotCount,
                    vehicle.entityX,
                    vehicle.entityY,
                    vehicle.entityZ,
                    nodes.posX,
                    nodes.posY,
                    nodes.posZ,
                    nodes.count
            ));
        }

        double mcWorldScanMs = (System.nanoTime() - startedNanos) / 1_000_000.0;
        return new PreparedStep(activeVehicles, renderWriters, electricSnapshots,
                dt, subSteps, startedNanos, mcWorldScanMs);
    }

    /**
     * Runs the pure physics portion of a prepared step. Minecraft world,
     * entity, renderer and networking APIs must not be touched from here.
     */
    public StepResult simulatePreparedStep(PreparedStep preparedStep) {
        List<SoftBodyVehicle> activeVehicles = preparedStep.activeVehicles();
        double dt = preparedStep.dt();
        int subSteps = preparedStep.subSteps();
        float subDt = (float) (dt / subSteps);
        float plasticRelaxation = 1.0f;
        int broadphaseRate = 10;
        double internalForceMs = 0.0, globalSAPMs = 0.0, dyeCollisionMs = 0.0, softCollisionMs = 0.0, mcCollisionMs = 0.0;
        double candidateGenerationMs = 0.0, colorMs = 0.0;
        double candidateParallelMs = 0.0, candidateMergeMs = 0.0;
        int lastCandidateTaskCount = 0;
        double refitNodeBoundsMs = 0.0, refitNodeChunkBoundsMs = 0.0, refitChunkSapMs = 0.0;
        double refitLocalSapsMs = 0.0, refitTriangleBoundsMs = 0.0, refitMeshletBoundsMs = 0.0;
        double refitLocalKeyFillMs = 0.0, refitLocalSortMs = 0.0, refitLocalPrefixMs = 0.0;
        long narrowChecks = 0L, narrowAabbPassed = 0L, narrowResolved = 0L, narrowCertificateSkipped = 0L;
        int lastSapHits = 0, lastCandidatesStored = 0, lastCandidatesDropped = 0;
        int lastChunkPairTests = 0, lastChunkPairOverlaps = 0, lastChunkPairProductive = 0;
        long lastFinePairTests = 0L;
        int lastNodeChunkCount = 0, lastInflatedNodeChunkCount = 0;
        int lastMeshletCount = 0, lastInflatedMeshletCount = 0;
        double lastMaxNodeChunkInflation = 1.0, lastMaxMeshletInflation = 1.0;
        List<ElectricSnapshot> electricSnapshots = new ArrayList<>(preparedStep.electricSnapshots());

        int nextRenderSnapshotIndex = 1;
        for (int s = 0; s < subSteps; s++) {
            boolean traceEnabled = eventTrace.enabled();
            long substepStartedNanos = traceEnabled ? System.nanoTime() : 0L;
            int brokenBeamsBefore = 0, breakGroupsBefore = 0, brokenTrianglesBefore = 0;
            if (traceEnabled) {
                for (SoftBodyVehicle vehicle : activeVehicles) {
                    vehicle.physicsEventTraceEnabled = true;
                    brokenBeamsBefore += vehicle.physicsEventTraceBrokenBeamCount();
                    breakGroupsBefore += vehicle.physicsEventTraceBreakGroupCount();
                    brokenTrianglesBefore += vehicle.triangles.brokenCount();
                }
            } else {
                for (SoftBodyVehicle vehicle : activeVehicles) vehicle.physicsEventTraceEnabled = false;
            }

            if (s > 0 && s % ELECTRIC_SNAPSHOT_SUBSTEP_INTERVAL == 0) {
                for (int i = 0; i < activeVehicles.size(); i++) {
                    electricSnapshots.set(i, activeVehicles.get(i).electrics.snapshot());
                }
            }

            long ti1 = System.nanoTime();

            IntStream.range(0, activeVehicles.size()).parallel().forEach(index ->
                    activeVehicles.get(index).solveInternalForces(
                            subDt, plasticRelaxation, electricSnapshots.get(index)));

            long ti2 = System.nanoTime();
            long substepInternalNs = ti2 - ti1;
            internalForceMs += substepInternalNs / 1_000_000.0;
            long substepSapNs = 0L, substepCandidateNs = 0L, substepColorNs = 0L;

            if (s % broadphaseRate == 0) {
                long tii1 = System.nanoTime();
                int activeOffset = 0;
                lastNodeChunkCount = 0;
                lastInflatedNodeChunkCount = 0;
                lastMeshletCount = 0;
                lastInflatedMeshletCount = 0;
                lastMaxNodeChunkInflation = 1.0;
                lastMaxMeshletInflation = 1.0;
                for (SoftBodyVehicle vehicle : activeVehicles) {
                    if (vehicle.nodes.count > SoftBodyCollisionManager.MAX_GLOBAL_NODES - activeOffset) {
                        throw new IllegalStateException("Total vehicle node count exceeds collision capacity "
                                + SoftBodyCollisionManager.MAX_GLOBAL_NODES);
                    }
                    vehicle.globalNodeOffset = activeOffset;
                    activeOffset += vehicle.nodes.count;
                    vehicle.collisionChunks.refit(subDt * broadphaseRate);
                    refitNodeBoundsMs += vehicle.collisionChunks.refitNodeBoundsNanos / 1_000_000.0;
                    refitNodeChunkBoundsMs += vehicle.collisionChunks.refitNodeChunkBoundsNanos / 1_000_000.0;
                    refitChunkSapMs += vehicle.collisionChunks.refitChunkSapNanos / 1_000_000.0;
                    refitLocalSapsMs += vehicle.collisionChunks.refitLocalSapsNanos / 1_000_000.0;
                    refitLocalKeyFillMs += vehicle.collisionChunks.refitLocalKeyFillNanos / 1_000_000.0;
                    refitLocalSortMs += vehicle.collisionChunks.refitLocalSortNanos / 1_000_000.0;
                    refitLocalPrefixMs += vehicle.collisionChunks.refitLocalPrefixNanos / 1_000_000.0;
                    refitTriangleBoundsMs += vehicle.collisionChunks.refitTriangleBoundsNanos / 1_000_000.0;
                    refitMeshletBoundsMs += vehicle.collisionChunks.refitMeshletBoundsNanos / 1_000_000.0;
                    lastNodeChunkCount += vehicle.collisionChunks.regroupableNodeChunkCount();
                    lastInflatedNodeChunkCount += vehicle.collisionChunks.stretchedNodeChunkCount();
                    lastMeshletCount += vehicle.collisionChunks.regroupableMeshletCount();
                    lastInflatedMeshletCount += vehicle.collisionChunks.stretchedMeshletCount();
                    lastMaxNodeChunkInflation = Math.max(lastMaxNodeChunkInflation,
                            vehicle.collisionChunks.maxNodeChunkSpanRatio());
                    lastMaxMeshletInflation = Math.max(lastMaxMeshletInflation,
                            vehicle.collisionChunks.maxMeshletSpanRatio());
                }

                long tii2 = System.nanoTime();
                substepSapNs = tii2 - tii1;
                globalSAPMs += (tii2 - tii1) / 1_000_000.0;

                collisionManager.clearContacts();

                long candidateGenerationStarted = System.nanoTime();
                int candidateTaskCount = configureCandidateTasks(activeVehicles);
                IntStream.range(0, candidateTaskCount).parallel().forEach(task ->
                        collisionPipeline.generateChunkCollisionCandidates(
                                candidateTaskVehicles[task], activeVehicles,
                                candidateTaskMeshletStarts[task], candidateTaskMeshletEnds[task],
                                candidateTaskBuffers[task]));
                long candidateParallelFinished = System.nanoTime();
                for (int task = 0; task < candidateTaskCount; task++) {
                    CollisionCandidateBuffer buffer = candidateTaskBuffers[task];
                    int stored = collisionManager.appendContacts(buffer);
                    collisionPipeline.applyCandidateStats(
                            buffer, stored, buffer.dropped + buffer.count - stored);
                }
                long candidateGenerationFinished = System.nanoTime();
                candidateParallelMs += (candidateParallelFinished - candidateGenerationStarted) / 1_000_000.0;
                candidateMergeMs += (candidateGenerationFinished - candidateParallelFinished) / 1_000_000.0;
                lastCandidateTaskCount = candidateTaskCount;
                substepCandidateNs = candidateGenerationFinished - candidateGenerationStarted;
                candidateGenerationMs += substepCandidateNs / 1_000_000.0;

                lastSapHits = 0;
                lastCandidatesStored = 0;
                lastCandidatesDropped = 0;
                lastChunkPairTests = 0;
                lastChunkPairOverlaps = 0;
                lastChunkPairProductive = 0;
                lastFinePairTests = 0L;
                for (SoftBodyVehicle vehicle : activeVehicles) {
                    lastSapHits += vehicle.collisionCandidateSapHits;
                    lastCandidatesStored += vehicle.collisionCandidateStored;
                    lastCandidatesDropped += vehicle.collisionCandidateDropped;
                    lastChunkPairTests += vehicle.collisionChunkPairTests;
                    lastChunkPairOverlaps += vehicle.collisionChunkPairOverlaps;
                    lastChunkPairProductive += vehicle.collisionChunkPairProductive;
                    lastFinePairTests += vehicle.collisionFinePairTests;
                }

                long colorStarted = System.nanoTime();
                collisionManager.buildAndColorBatches();
                long colorFinished = System.nanoTime();
                substepColorNs = colorFinished - colorStarted;
                colorMs += substepColorNs / 1_000_000.0;

                long tii3 = System.nanoTime();
                dyeCollisionMs += (tii3 - tii2) / 1_000_000.0;
            }

            long ti3 = System.nanoTime();

            narrowChecks += collisionManager.contactCount.get();
            collisionManager.sweptResolvedCount.set(0);
            long narrowStats = collisionPipeline.solveSoftBodyContacts(subDt);
            int substepAabbPassed = (int) (narrowStats & CollisionPipeline.NARROW_STAT_MASK);
            int substepResolved = (int) ((narrowStats >>> CollisionPipeline.NARROW_STAT_BITS)
                    & CollisionPipeline.NARROW_STAT_MASK);
            int substepCertificateSkipped = (int) (narrowStats >>> (CollisionPipeline.NARROW_STAT_BITS * 2));
            narrowAabbPassed += substepAabbPassed;
            narrowResolved += substepResolved;
            narrowCertificateSkipped += substepCertificateSkipped;

            long ti4 = System.nanoTime();
            long substepSoftNs = ti4 - ti3;
            softCollisionMs += substepSoftNs / 1_000_000.0;

            activeVehicles.parallelStream().forEach(vehicle -> {
                collisionPipeline.solveEnvironmentCollisions(vehicle, subDt);
            });

            long ti5 = System.nanoTime();
            long substepEnvironmentNs = ti5 - ti4;
            mcCollisionMs += substepEnvironmentNs / 1_000_000.0;

            if (traceEnabled) {
                int brokenBeamsAfter = 0, breakGroupsAfter = 0, brokenTrianglesAfter = 0;
                long breakCommitNs = 0L;
                for (SoftBodyVehicle vehicle : activeVehicles) {
                    brokenBeamsAfter += vehicle.physicsEventTraceBrokenBeamCount();
                    breakGroupsAfter += vehicle.physicsEventTraceBreakGroupCount();
                    brokenTrianglesAfter += vehicle.triangles.brokenCount();
                    breakCommitNs += vehicle.physicsEventTraceBreakCommitNanos;
                }
                eventTrace.record(s,
                        System.nanoTime() - substepStartedNanos,
                        substepInternalNs, breakCommitNs, substepSapNs, substepCandidateNs,
                        substepColorNs, substepSoftNs, substepEnvironmentNs,
                        collisionManager.contactCount.get(), substepCertificateSkipped,
                        substepAabbPassed, substepResolved, collisionManager.sweptResolvedCount.get(),
                        Math.max(0, brokenBeamsAfter - brokenBeamsBefore),
                        Math.max(0, breakGroupsAfter - breakGroupsBefore),
                        Math.max(0, brokenTrianglesAfter - brokenTrianglesBefore));
            }

            int completedSubSteps = s + 1;
            if (isRenderSnapshotBoundary(completedSubSteps, subSteps)) {
                long simulatedOffsetNanos = Math.round(
                        (double) completedSubSteps / (double) subSteps * dt * 1_000_000_000.0);
                publishRenderSnapshots(
                        activeVehicles,
                        preparedStep.renderWriters(),
                        nextRenderSnapshotIndex++,
                        simulatedOffsetNanos
                );
            }
        }

        long t3 = System.nanoTime();
        activeVehicles.parallelStream().forEach(vehicle -> {
            vehicle.updateLocalOriginCache();
            vehicle.updateBeamPrecompression(dt);
            vehicle.sampleMotion(dt);
        });
        long t4 = System.nanoTime();
        double postUpdateMs = (t4 - t3) / 1_000_000.0;

        double[] timings = new double[60];
        timings[1] = preparedStep.mcWorldScanMs();
        timings[2] = internalForceMs;
        timings[3] = globalSAPMs;
        timings[4] = dyeCollisionMs;
        timings[5] = softCollisionMs;
        timings[6] = mcCollisionMs;
        timings[7] = postUpdateMs;
        timings[9] = candidateGenerationMs;
        timings[12] = colorMs;
        timings[13] = lastSapHits;
        timings[14] = lastCandidatesStored;
        timings[15] = lastCandidatesDropped;
        timings[16] = narrowChecks;
        timings[17] = narrowAabbPassed;
        timings[18] = narrowResolved;
        timings[37] = narrowCertificateSkipped;
        timings[38] = lastChunkPairTests;
        timings[39] = lastChunkPairOverlaps;
        timings[40] = lastFinePairTests;
        timings[41] = lastChunkPairProductive;
        timings[42] = refitNodeBoundsMs;
        timings[43] = refitNodeChunkBoundsMs;
        timings[44] = refitChunkSapMs;
        timings[45] = refitLocalSapsMs;
        timings[46] = refitTriangleBoundsMs;
        timings[47] = refitMeshletBoundsMs;
        timings[48] = refitLocalKeyFillMs;
        timings[49] = refitLocalSortMs;
        timings[50] = refitLocalPrefixMs;
        timings[51] = lastCandidateTaskCount;
        timings[52] = candidateParallelMs;
        timings[53] = candidateMergeMs;
        timings[54] = lastNodeChunkCount;
        timings[55] = lastInflatedNodeChunkCount;
        timings[56] = lastMaxNodeChunkInflation;
        timings[57] = lastMeshletCount;
        timings[58] = lastInflatedMeshletCount;
        timings[59] = lastMaxMeshletInflation;
        timings[19] = collisionManager.activeBatchCount;
        int largestBatch = 0;
        for (int batch = 0; batch < collisionManager.activeBatchCount; batch++) {
            int size = collisionManager.batchSize[batch];
            timings[21 + batch] = size;
            if (size > largestBatch) largestBatch = size;
        }
        timings[20] = largestBatch;
        return new StepResult(preparedStep, timings, System.nanoTime());
    }

    /**
     * Publishes a completed physics step back to Minecraft. This method must
     * run on the client thread before vehicle lifecycle changes for the tick.
     */
    public double[] commitPreparedStep(StepResult result) {
        long commitStartedNanos = System.nanoTime();
        for (SoftBodyVehicle vehicle : result.preparedStep().activeVehicles()) {
            vehicle.nodes.writeRenderBuffer();
            vehicle.updateEntityLocation();
            if (vehicle.parentEntity != null && ClientPlayNetworking.canSend(VehicleSyncPayload.ID)) {
                ClientPlayNetworking.send(new VehicleSyncPayload(
                        vehicle.parentEntity.getId(),
                        vehicle.parentEntity.getX(),
                        vehicle.parentEntity.getY(),
                        vehicle.parentEntity.getZ(),
                        vehicle.parentEntity.getYaw()
                ));
            }
        }
        double[] timings = result.timings();
        timings[8] = (System.nanoTime() - commitStartedNanos) / 1_000_000.0;
        timings[0] = (result.finishedNanos() - result.preparedStep().startedNanos()) / 1_000_000.0
                + timings[8];
        return timings;
    }

    private void persistEventTrace(PhysicsEventTrace.CompletedCapture capture) {
        Path outputDirectory = eventTraceDirectory;
        if (outputDirectory == null) {
            eventTraceWriteInFlight = false;
            BeamCraft.LOGGER.warn("BeamCraft physics trace completed without an output directory");
            return;
        }
        long capturedAtMillis = System.currentTimeMillis();
        EVENT_TRACE_WRITER.execute(() -> {
            try {
                Files.createDirectories(outputDirectory);
                Path output = outputDirectory.resolve("physics-trace-" + capturedAtMillis + ".csv");
                Files.writeString(output, capture.formatCsv(), StandardCharsets.UTF_8,
                        StandardOpenOption.CREATE_NEW, StandardOpenOption.WRITE);
                BeamCraft.LOGGER.info("BeamCraft physics trace written to {}", output.toAbsolutePath());
            } catch (IOException exception) {
                BeamCraft.LOGGER.error("Failed to write BeamCraft physics trace", exception);
            } finally {
                eventTraceWriteInFlight = false;
            }
        });
    }

    private int configureCandidateTasks(List<SoftBodyVehicle> activeVehicles) {
        int taskCount = 0;
        for (SoftBodyVehicle vehicle : activeVehicles) {
            CollisionPipeline.clearCandidateStats(vehicle);
            int meshletCount = vehicle.collisionChunks.triangleMeshletCount();
            taskCount += (meshletCount + CANDIDATE_MESHLETS_PER_TASK - 1)
                    / CANDIDATE_MESHLETS_PER_TASK;
        }
        ensureCandidateTaskCapacity(taskCount);

        int task = 0;
        for (SoftBodyVehicle vehicle : activeVehicles) {
            int meshletCount = vehicle.collisionChunks.triangleMeshletCount();
            for (int start = 0; start < meshletCount; start += CANDIDATE_MESHLETS_PER_TASK) {
                candidateTaskVehicles[task] = vehicle;
                candidateTaskMeshletStarts[task] = start;
                candidateTaskMeshletEnds[task] = Math.min(meshletCount,
                        start + CANDIDATE_MESHLETS_PER_TASK);
                task++;
            }
        }
        return taskCount;
    }

    private void ensureCandidateTaskCapacity(int required) {
        if (required <= candidateTaskBuffers.length) return;
        int oldCapacity = candidateTaskBuffers.length;
        int capacity = Math.max(required, Math.max(8, oldCapacity << 1));
        candidateTaskBuffers = Arrays.copyOf(candidateTaskBuffers, capacity);
        candidateTaskVehicles = Arrays.copyOf(candidateTaskVehicles, capacity);
        candidateTaskMeshletStarts = Arrays.copyOf(candidateTaskMeshletStarts, capacity);
        candidateTaskMeshletEnds = Arrays.copyOf(candidateTaskMeshletEnds, capacity);
        for (int task = oldCapacity; task < capacity; task++) {
            candidateTaskBuffers[task] = new CollisionCandidateBuffer();
        }
    }

    /**
     * Synchronous compatibility entry point used by tools and tests.
     */
    public void step(World mcWorld, double dt, double[] lastPhycisMsDetail) {
        StepResult result = simulatePreparedStep(prepareStep(mcWorld, dt));
        double[] timings = commitPreparedStep(result);
        System.arraycopy(timings, 0, lastPhycisMsDetail, 0,
                Math.min(timings.length, lastPhycisMsDetail.length));
    }

    public record PreparedStep(
            List<SoftBodyVehicle> activeVehicles,
            List<PhysicsRenderTimeline.Writer> renderWriters,
            List<ElectricSnapshot> electricSnapshots,
            double dt,
            int subSteps,
            long startedNanos,
            double mcWorldScanMs
    ) {
        public PreparedStep(List<SoftBodyVehicle> activeVehicles,
                            List<PhysicsRenderTimeline.Writer> renderWriters,
                            double dt, int subSteps, long startedNanos, double mcWorldScanMs) {
            this(activeVehicles, renderWriters,
                    Collections.nCopies(activeVehicles.size(), ElectricSnapshot.EMPTY),
                    dt, subSteps, startedNanos, mcWorldScanMs);
        }
    }

    public record StepResult(PreparedStep preparedStep, double[] timings, long finishedNanos) {
    }

    private static boolean isRenderSnapshotBoundary(int completedSubSteps, int totalSubSteps) {
        return completedSubSteps == totalSubSteps
                || completedSubSteps % RENDER_SNAPSHOT_SUBSTEP_INTERVAL == 0;
    }

    private static void publishRenderSnapshots(
            List<SoftBodyVehicle> activeVehicles,
            List<PhysicsRenderTimeline.Writer> renderWriters,
            int snapshotIndex,
            long simulatedOffsetNanos
    ) {
        for (int vehicleIndex = 0; vehicleIndex < activeVehicles.size(); vehicleIndex++) {
            NodeContainer nodes = activeVehicles.get(vehicleIndex).nodes;
            renderWriters.get(vehicleIndex).publish(
                    snapshotIndex,
                    simulatedOffsetNanos,
                    nodes.posX,
                    nodes.posY,
                    nodes.posZ
            );
        }
    }

}
