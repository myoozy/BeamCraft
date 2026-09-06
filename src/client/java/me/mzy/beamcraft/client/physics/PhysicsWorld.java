package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import me.mzy.beamcraft.network.VehicleSyncPayload;

import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Core physical world controller for beam-based vehicle simulation
 * Manages nodes, beams, collision caching and physics integration
 */
public class PhysicsWorld {
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

    /** Shared collision pipeline: candidate generation, soft-contact solving and environment collision. */
    public final CollisionPipeline collisionPipeline = new CollisionPipeline(voxelSnapshot, globalSap, collisionManager);

    private int nextVehicleId = 0;

    public final java.util.List<SoftBodyVehicle> vehicles = new java.util.concurrent.CopyOnWriteArrayList<>();

    public PhysicsWorld() {
        // Empty constructor, data will be injected by JBeam parser
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
            electricSnapshots.add(vehicle.electrics.snapshot());
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
        List<ElectricSnapshot> electricSnapshots = new ArrayList<>(preparedStep.electricSnapshots());

        int nextRenderSnapshotIndex = 1;
        for (int s = 0; s < subSteps; s++) {
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
            internalForceMs += (ti2 - ti1) / 1_000_000.0;

            if (s % broadphaseRate == 0) {
                long tii1 = System.nanoTime();
                globalSap.clear();

                int activeOffset = 0;
                for (SoftBodyVehicle vehicle : activeVehicles) {
                    vehicle.globalNodeOffset = activeOffset;
                    activeOffset += vehicle.nodes.count;

                    globalSap.insertNodes(vehicle);
                }

                // if (activeOffset >= SoftBodyCollisionManager.MAX_GLOBAL_NODES) { ... }

                globalSap.updateAndSort();

                long tii2 = System.nanoTime();
                globalSAPMs += (tii2 - tii1) / 1_000_000.0;

                collisionManager.clearContacts();

                activeVehicles.parallelStream().forEach(vehicle -> {
                    collisionPipeline.generateCollisionCandidates(vehicle, subDt * broadphaseRate);
                });

                collisionManager.buildAndColorBatches();

                long tii3 = System.nanoTime();
                dyeCollisionMs += (tii3 - tii2) / 1_000_000.0;
            }

            long ti3 = System.nanoTime();

            collisionPipeline.solveSoftBodyContacts(subDt);

            long ti4 = System.nanoTime();
            softCollisionMs += (ti4 - ti3) / 1_000_000.0;

            activeVehicles.parallelStream().forEach(vehicle -> {
                collisionPipeline.solveEnvironmentCollisions(vehicle, subDt);
            });

            long ti5 = System.nanoTime();
            mcCollisionMs += (ti5 - ti4) / 1_000_000.0;

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
        });
        long t4 = System.nanoTime();
        double postUpdateMs = (t4 - t3) / 1_000_000.0;

        double[] timings = new double[9];
        timings[1] = preparedStep.mcWorldScanMs();
        timings[2] = internalForceMs;
        timings[3] = globalSAPMs;
        timings[4] = dyeCollisionMs;
        timings[5] = softCollisionMs;
        timings[6] = mcCollisionMs;
        timings[7] = postUpdateMs;
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
