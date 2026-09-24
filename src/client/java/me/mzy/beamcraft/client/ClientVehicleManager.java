package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.config.BeamCraftConfigManager;
import me.mzy.beamcraft.client.debug.LoadTiming;
import me.mzy.beamcraft.client.material.MaterialLibrary;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.JBeamLoader;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.utility.Utility;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.entity.Entity;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.chunk.ChunkStatus;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public final class ClientVehicleManager {

    private static final double MIN_INTERACTION_BOUNDS_SIDE = 0.75;
    private static final double MIN_RENDER_BOUNDS_SPAN = 4.0;
    private static final double RENDER_DEFORMATION_MARGIN = 2.0;
    private static final double CHUNK_SAFETY_MARGIN = 2.0;
    private static final int MAX_REQUIRED_CHUNK_SPAN = 8;
    private static float[] boundsScratch = new float[NodeContainer.INIT_NODE_CAP];

    static record BoundsProfile(double interactionSide, double renderMaxSpan) {}

    private static final Map<Integer, ManagedVehicle> TRACKED_VEHICLES = new HashMap<>();
    private static final Map<UUID, ManagedVehicle> RETAINED_VEHICLES = new HashMap<>();
    private static final VehicleLoadFailureCache LOAD_FAILURES = new VehicleLoadFailureCache();
    private static ClientWorld managedWorld;

    private static final class ManagedVehicle {
        final UUID uuid;
        final String rootPart;
        final String pcFile;
        final SoftBodyVehicle softBody;
        PhysicsVehicleEntity entity;
        boolean active;

        ManagedVehicle(PhysicsVehicleEntity entity, SoftBodyVehicle softBody) {
            this.uuid = entity.getUuid();
            this.rootPart = entity.getRootPartName();
            this.pcFile = entity.getPcFileName();
            this.entity = entity;
            this.softBody = softBody;
        }

        boolean matches(PhysicsVehicleEntity candidate) {
            return uuid.equals(candidate.getUuid())
                    && rootPart.equals(candidate.getRootPartName())
                    && pcFile.equals(candidate.getPcFileName());
        }
    }

    // Reused for every vehicle because each upload is completed before the next
    // vehicle overwrites these interpolation arrays.
    private static float[] sharedInterpX = new float[NodeContainer.INIT_NODE_CAP];
    private static float[] sharedInterpY = new float[NodeContainer.INIT_NODE_CAP];
    private static float[] sharedInterpZ = new float[NodeContainer.INIT_NODE_CAP];

    private ClientVehicleManager() {
    }

    public static void update(MinecraftClient client) {
        if (client.world == null) {
            clearVehicles();
            managedWorld = null;
            return;
        }
        if (managedWorld != client.world) {
            clearVehicles();
            managedWorld = client.world;
        }

        Set<Integer> seenEntityIds = new HashSet<>();
        for (Entity entity : client.world.getEntities()) {
            if (!(entity instanceof PhysicsVehicleEntity vehicleEntity)) {
                continue;
            }

            int entityId = vehicleEntity.getId();
            seenEntityIds.add(entityId);
            // A spawn packet can briefly expose the entity before its tracked
            // setup strings arrive. Do not mistake that transient empty setup
            // for a replacement of a retained vehicle with the same UUID.
            if (vehicleEntity.getRootPartName().isEmpty()) {
                continue;
            }
            LOAD_FAILURES.removeStale(entityId, vehicleEntity.getUuid());

            ManagedVehicle managed = RETAINED_VEHICLES.get(vehicleEntity.getUuid());
            if (managed != null && !managed.matches(vehicleEntity)) {
                destroyVehicle(managed);
                managed = null;
            }
            if (managed == null && LOAD_FAILURES.shouldAttempt(
                    entityId,
                    vehicleEntity.getUuid(),
                    vehicleEntity.getRootPartName(),
                    vehicleEntity.getPcFileName())) {
                managed = createVehicle(client, vehicleEntity);
                if (managed != null) {
                    RETAINED_VEHICLES.put(managed.uuid, managed);
                }
            }
            if (managed == null) {
                continue;
            }

            bindTrackedEntity(managed, vehicleEntity);
            TRACKED_VEHICLES.put(entityId, managed);
            updateEntityBounds(managed.softBody);
            setActive(managed, hasRequiredChunks(client.world, managed.softBody));
        }

        // Leaving vanilla's tracking view must not destroy the expensive client
        // soft body. Keep it by UUID and detach its temporary client entity.
        for (Map.Entry<Integer, ManagedVehicle> entry : new ArrayList<>(TRACKED_VEHICLES.entrySet())) {
            ManagedVehicle managed = entry.getValue();
            Entity current = client.world.getEntityById(entry.getKey());
            if (seenEntityIds.contains(entry.getKey()) && current == managed.entity) {
                continue;
            }
            TRACKED_VEHICLES.remove(entry.getKey(), managed);
            setActive(managed, false);
            if (managed.entity != null && managed.entity.getId() == entry.getKey()) {
                managed.entity = null;
                managed.softBody.bindParentEntity(null);
            }
        }
    }

    private static ManagedVehicle createVehicle(MinecraftClient client, PhysicsVehicleEntity vehicleEntity) {
        String rootPart = vehicleEntity.getRootPartName();
        if (rootPart.isEmpty()) {
            return null;
        }

        SoftBodyVehicle softBody = new SoftBodyVehicle(vehicleEntity);
        Map<String, com.google.gson.JsonObject> localRegistry = new HashMap<>();
        Map<String, String> localConfig = new HashMap<>();
        List<File> assetRoots = BeamCraftConfigManager.assetRoots();

        long totalStart = LoadTiming.start();
        long phaseStart = LoadTiming.start();
        if (!JBeamLoader.loadVehicle(
                assetRoots,
                rootPart,
                vehicleEntity.getPcFileName(),
                localRegistry,
                localConfig
        )) {
            // The named .pc could not be resolved. Stopping here is the point: an
            // empty userConfig assembles every slot from its default part, so
            // carrying on would produce a vehicle other than the one that was asked
            // for, with nothing reported.
            LOAD_FAILURES.recordFailure(
                    vehicleEntity.getId(),
                    vehicleEntity.getUuid(),
                    rootPart,
                    vehicleEntity.getPcFileName());
            System.err.println("Vehicle load failed for entity " + vehicleEntity.getId());
            return null;
        }
        LoadTiming.log("[load 1/4] JBeam scan + parse", phaseStart);

        // Assembly comes before the mesh import because the flexbody table it builds
        // is what names the meshes this vehicle needs. Importing by name is what keeps
        // the load off the ~615 MB of common assets the vehicle never touches.
        phaseStart = LoadTiming.start();
        boolean assembled = new JBeamAssembler().assembleVehicle(
                rootPart,
                localConfig,
                localRegistry,
                softBody
        );
        if (!assembled) {
            // Nothing has been acquired yet here, so there is nothing to release —
            // releasing would unbalance a live sibling instance's ref count.
            LOAD_FAILURES.recordFailure(
                    vehicleEntity.getId(),
                    vehicleEntity.getUuid(),
                    rootPart,
                    vehicleEntity.getPcFileName());
            System.err.println("Vehicle assembly failed for entity " + vehicleEntity.getId());
            return null;
        }
        LoadTiming.log("[load 2/4] assembly", phaseStart);

        boolean materialsAcquired = false;
        boolean meshesAcquired = false;
        try {
            phaseStart = LoadTiming.start();
            // Both resource loaders retain the namespace before doing their
            // potentially failing scan/import work, so mark ownership first.
            materialsAcquired = true;
            MaterialLibrary.requireMaterials(assetRoots, rootPart);
            LoadTiming.log("[load 3/4] material index", phaseStart);

            phaseStart = LoadTiming.start();
            meshesAcquired = true;
            DaeMeshLoader.requireMeshes(assetRoots, rootPart, flexbodyMeshNames(softBody));
            LoadTiming.log("[load 4/4] mesh import", phaseStart);

            LoadTiming.log("[load total] vehicle load (" + rootPart + ")", totalStart);

            BoundsProfile boundsProfile = computeBoundsProfile(softBody.nodes);
            softBody.interactionBoundsSide = boundsProfile.interactionSide();
            softBody.renderBoundsMaxSpan = boundsProfile.renderMaxSpan();

            float playerYaw = client.player != null ? client.player.getYaw() : 0.0f;
            softBody.nodes.rotateNodes(playerYaw, 0, 0);
            LOAD_FAILURES.recordSuccess(vehicleEntity.getId());
            return new ManagedVehicle(vehicleEntity, softBody);
        } catch (RuntimeException failure) {
            softBody.clear();
            if (meshesAcquired) {
                DaeMeshLoader.releaseVehicleModels(rootPart);
            }
            if (materialsAcquired) {
                MaterialLibrary.releaseMaterials(rootPart);
            }
            LOAD_FAILURES.recordFailure(
                    vehicleEntity.getId(), vehicleEntity.getUuid(), rootPart, vehicleEntity.getPcFileName());
            BeamCraft.LOGGER.error("Vehicle load failed for entity {} ({}/{})",
                    vehicleEntity.getId(), rootPart, vehicleEntity.getPcFileName(), failure);
            return null;
        }
    }

    private static void bindTrackedEntity(ManagedVehicle managed, PhysicsVehicleEntity entity) {
        if (managed.entity == entity) {
            return;
        }
        if (managed.entity != null) {
            TRACKED_VEHICLES.remove(managed.entity.getId(), managed);
        }
        managed.entity = entity;
        managed.softBody.bindParentEntity(entity);
    }

    private static void setActive(ManagedVehicle managed, boolean active) {
        active &= managed.entity != null;
        if (managed.active == active) {
            return;
        }
        managed.active = active;
        if (active) {
            BeamCraftClient.PHYSICS_WORLD.addVehicle(managed.softBody);
        } else {
            BeamCraftClient.PHYSICS_WORLD.suspendVehicle(managed.softBody);
        }
    }

    private static boolean hasRequiredChunks(ClientWorld world, SoftBodyVehicle vehicle) {
        PhysicsVehicleEntity entity = vehicle.parentEntity;
        if (entity == null) {
            return false;
        }
        return hasRequiredChunks(
                vehicle.nodes,
                entity.getX(),
                entity.getZ(),
                BeamCraftClient.DELTA_TIME,
                (chunkX, chunkZ) -> world.getChunkManager().getChunk(
                        chunkX, chunkZ, ChunkStatus.FULL, false) != null
        );
    }

    @FunctionalInterface
    interface ChunkAvailability {
        boolean isLoaded(int chunkX, int chunkZ);
    }

    /**
     * Requires the current and next-tick neighbourhood of every node. Missing
     * client chunks resolve as an EmptyChunk full of air, so physics and
     * rendering must remain asleep until all of these columns are present.
     */
    static boolean hasRequiredChunks(
            NodeContainer nodes,
            double originX,
            double originZ,
            double dt,
            ChunkAvailability availability
    ) {
        if (nodes.count == 0) {
            return false;
        }
        double minX = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;
        for (int node = 0; node < nodes.count; node++) {
            double x = originX + nodes.posX[node];
            double z = originZ + nodes.posZ[node];
            double nextX = x + nodes.velX[node] * dt;
            double nextZ = z + nodes.velZ[node] * dt;
            if (!Double.isFinite(x) || !Double.isFinite(z)
                    || !Double.isFinite(nextX) || !Double.isFinite(nextZ)) {
                return false;
            }
            minX = Math.min(minX, Math.min(x, nextX));
            minZ = Math.min(minZ, Math.min(z, nextZ));
            maxX = Math.max(maxX, Math.max(x, nextX));
            maxZ = Math.max(maxZ, Math.max(z, nextZ));
        }

        int minChunkX = MathHelper.floor(minX - CHUNK_SAFETY_MARGIN) >> 4;
        int maxChunkX = MathHelper.floor(maxX + CHUNK_SAFETY_MARGIN) >> 4;
        int minChunkZ = MathHelper.floor(minZ - CHUNK_SAFETY_MARGIN) >> 4;
        int maxChunkZ = MathHelper.floor(maxZ + CHUNK_SAFETY_MARGIN) >> 4;
        if (maxChunkX - minChunkX > MAX_REQUIRED_CHUNK_SPAN
                || maxChunkZ - minChunkZ > MAX_REQUIRED_CHUNK_SPAN) {
            return false;
        }
        for (int chunkX = minChunkX; chunkX <= maxChunkX; chunkX++) {
            for (int chunkZ = minChunkZ; chunkZ <= maxChunkZ; chunkZ++) {
                if (!availability.isLoaded(chunkX, chunkZ)) {
                    return false;
                }
            }
        }
        return true;
    }

    /** The mesh names the assembled vehicle's flexbodies resolve against. */
    private static List<String> flexbodyMeshNames(SoftBodyVehicle vehicle) {
        FlexbodyContainer flex = vehicle.flexbodies;
        List<String> names = new ArrayList<>(flex.meshCount);
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            names.add(flex.meshName[mesh]);
        }
        return names;
    }

    private static void updateEntityBounds(SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        if (nodes.count == 0 || vehicle.parentEntity == null) {
            return;
        }

        double entityX = vehicle.parentEntity.getX();
        double entityY = vehicle.parentEntity.getY();
        double entityZ = vehicle.parentEntity.getZ();
        Box interactionBounds = computeInteractionLocalBounds(vehicle);
        Box visibilityBounds = computeCappedRenderLocalBounds(nodes, vehicle.renderBoundsMaxSpan);
        vehicle.parentEntity.setBoundingBox(interactionBounds.offset(entityX, entityY, entityZ));
        vehicle.parentEntity.setVisibilityBoundingBox(visibilityBounds.offset(entityX, entityY, entityZ));
    }

    /**
     * Captures the two spawn-time dimensions that must not grow with deformation.
     * The shorter authored horizontal span is the interaction-square side.  The
     * full 3-D diagonal is a rotation-safe cap for the visibility AABB.
     */
    static BoundsProfile computeBoundsProfile(NodeContainer nodes) {
        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;
        for (int node = 0; node < nodes.count; node++) {
            double x = nodes.baseX[node];
            double y = nodes.baseY[node];
            double z = nodes.baseZ[node];
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) continue;
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
            maxZ = Math.max(maxZ, z);
        }
        if (!Double.isFinite(minX)) {
            return new BoundsProfile(MIN_INTERACTION_BOUNDS_SIDE, MIN_RENDER_BOUNDS_SPAN);
        }

        double spanX = maxX - minX;
        double spanY = maxY - minY;
        double spanZ = maxZ - minZ;
        double interactionSide = Math.max(MIN_INTERACTION_BOUNDS_SIDE, Math.min(spanX, spanZ));
        double diagonal = Math.sqrt(square(spanX) + square(spanY) + square(spanZ));
        double renderMaxSpan = Math.max(MIN_RENDER_BOUNDS_SPAN, diagonal + RENDER_DEFORMATION_MARGIN);
        return new BoundsProfile(interactionSide, renderMaxSpan);
    }

    /** Small, yaw-invariant interaction cube centred on the cabin when possible. */
    static Box computeInteractionLocalBounds(SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        ensureBoundsScratchCapacity(nodes.count);
        double medianX = NodeContainer.medianOfFinite(nodes.renderSnapCurrX, nodes.count, boundsScratch);
        double medianY = NodeContainer.medianOfFinite(nodes.renderSnapCurrY, nodes.count, boundsScratch);
        double medianZ = NodeContainer.medianOfFinite(nodes.renderSnapCurrZ, nodes.count, boundsScratch);
        double centerX = medianX;
        double centerY = medianY;
        double centerZ = medianZ;

        var driver = vehicle.cameras.driver();
        if (driver != null) {
            int driverNode = driver.nodeIndex();
            if (isFiniteNode(nodes, driverNode)) {
                centerX = nodes.renderSnapCurrX[driverNode];
                centerY = nodes.renderSnapCurrY[driverNode];
                centerZ = nodes.renderSnapCurrZ[driverNode];

                var refs = vehicle.cameras.refNodes();
                if (refs != null && isFiniteNode(nodes, refs.ref()) && isFiniteNode(nodes, refs.left())) {
                    double lateralX = nodes.renderSnapCurrX[refs.left()] - nodes.renderSnapCurrX[refs.ref()];
                    double lateralZ = nodes.renderSnapCurrZ[refs.left()] - nodes.renderSnapCurrZ[refs.ref()];
                    double lateralLength = Math.sqrt(square(lateralX) + square(lateralZ));
                    if (lateralLength > 1.0e-8) {
                        lateralX /= lateralLength;
                        lateralZ /= lateralLength;
                        double lateralOffset = (medianX - centerX) * lateralX + (medianZ - centerZ) * lateralZ;
                        centerX += lateralOffset * lateralX;
                        centerZ += lateralOffset * lateralZ;
                    }
                }
            }
        }

        double half = Math.max(MIN_INTERACTION_BOUNDS_SIDE, vehicle.interactionBoundsSide) * 0.5;
        return new Box(centerX - half, centerY - half, centerZ - half,
                centerX + half, centerY + half, centerZ + half);
    }

    /**
     * Full-node visibility envelope, capped around the robust median so one
     * detached node cannot keep the complete vehicle renderable indefinitely.
     */
    static Box computeCappedRenderLocalBounds(NodeContainer nodes, double maxSpan) {
        ensureBoundsScratchCapacity(nodes.count);
        double centerX = NodeContainer.medianOfFinite(nodes.renderSnapCurrX, nodes.count, boundsScratch);
        double centerY = NodeContainer.medianOfFinite(nodes.renderSnapCurrY, nodes.count, boundsScratch);
        double centerZ = NodeContainer.medianOfFinite(nodes.renderSnapCurrZ, nodes.count, boundsScratch);

        double minX = Double.POSITIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        double maxZ = Double.NEGATIVE_INFINITY;
        for (int node = 0; node < nodes.count; node++) {
            double x = nodes.renderSnapCurrX[node];
            double y = nodes.renderSnapCurrY[node];
            double z = nodes.renderSnapCurrZ[node];
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
                continue;
            }
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
            maxZ = Math.max(maxZ, z);
        }

        if (!Double.isFinite(minX)) {
            minX = maxX = centerX;
            minY = maxY = centerY;
            minZ = maxZ = centerZ;
        }
        double half = Math.max(MIN_RENDER_BOUNDS_SPAN, maxSpan) * 0.5;
        if (maxX - minX > half * 2.0) { minX = centerX - half; maxX = centerX + half; }
        if (maxY - minY > half * 2.0) { minY = centerY - half; maxY = centerY + half; }
        if (maxZ - minZ > half * 2.0) { minZ = centerZ - half; maxZ = centerZ + half; }
        return new Box(minX, minY, minZ, maxX, maxY, maxZ);
    }

    private static boolean isFiniteNode(NodeContainer nodes, int node) {
        return 0 <= node && node < nodes.count
                && Float.isFinite(nodes.renderSnapCurrX[node])
                && Float.isFinite(nodes.renderSnapCurrY[node])
                && Float.isFinite(nodes.renderSnapCurrZ[node]);
    }

    private static void ensureBoundsScratchCapacity(int nodeCount) {
        if (boundsScratch.length < nodeCount) {
            boundsScratch = new float[Math.max(nodeCount, boundsScratch.length * 2)];
        }
    }

    private static double square(double value) {
        return value * value;
    }

    private static void clearVehicles() {
        LOAD_FAILURES.clear();
        if (RETAINED_VEHICLES.isEmpty()) {
            TRACKED_VEHICLES.clear();
            return;
        }

        for (ManagedVehicle managed : new ArrayList<>(RETAINED_VEHICLES.values())) {
            destroyVehicle(managed);
        }
        TRACKED_VEHICLES.clear();
        RETAINED_VEHICLES.clear();
    }

    private static void destroyVehicle(ManagedVehicle managed) {
        RETAINED_VEHICLES.remove(managed.uuid, managed);
        if (managed.entity != null) {
            TRACKED_VEHICLES.remove(managed.entity.getId(), managed);
        }
        managed.active = false;
        SoftBodyVehicle vehicle = managed.softBody;
        DaeMeshLoader.releaseVehicleModels(vehicle.flexbodies.vehicleNamespace);
        MaterialLibrary.releaseMaterials(vehicle.flexbodies.vehicleNamespace);
        // PhysicsWorld.removeVehicle() clears the vehicle and therefore closes
        // its GPU skinning pipeline exactly once.
        BeamCraftClient.PHYSICS_WORLD.removeVehicle(vehicle);
        managed.entity = null;
    }

    public static SoftBodyVehicle getVehicle(int entityId) {
        ManagedVehicle managed = TRACKED_VEHICLES.get(entityId);
        return managed != null && managed.active ? managed.softBody : null;
    }

    public static void initRenderHooks() {
        // Iris renders its shadow map when vanilla enters renderSky(), before
        // Fabric's BEFORE_ENTITIES event. Prepare skinning at START so shadow
        // and main entity passes consume the same frame's node positions.
        WorldRenderEvents.START.register(context -> {
            MinecraftClient client = MinecraftClient.getInstance();
            if (client.world == null || TRACKED_VEHICLES.isEmpty()) {
                return;
            }

            float tickDelta = context.tickCounter().getTickDelta(true);
            long renderMoment = client.world.getTime() << 32
                    ^ Integer.toUnsignedLong(Float.floatToRawIntBits(tickDelta));

            for (ManagedVehicle managed : TRACKED_VEHICLES.values()) {
                if (!managed.active) {
                    continue;
                }
                SoftBodyVehicle vehicle = managed.softBody;
                FlexbodyContainer flex = vehicle.flexbodies;
                NodeContainer nodes = vehicle.nodes;

                if (!flex.isSkinningBound) {
                    FlexbodyBindingUtil.performBinding(flex, vehicle);
                }
                if (flex.totalVertexCount == 0 || nodes.count == 0) {
                    continue;
                }
                if (!flex.skinningPipeline.isReady()
                        && !flex.skinningPipeline.init(flex, nodes.count)) {
                    continue;
                }

                ensureInterpolationCapacity(nodes.count);
                Vec3d renderedEntityOrigin = getRenderedEntityOrigin(vehicle.parentEntity, tickDelta);
                boolean sampledTimeline = vehicle.renderTimeline.sampleAtTickDeltaRelativeTo(
                        tickDelta,
                        renderedEntityOrigin.x,
                        renderedEntityOrigin.y,
                        renderedEntityOrigin.z,
                        sharedInterpX,
                        sharedInterpY,
                        sharedInterpZ,
                        nodes.count
                );
                if (!sampledTimeline) {
                    for (int node = 0; node < nodes.count; node++) {
                        sharedInterpX[node] = interpolate(
                                nodes.renderSnapPrevX[node],
                                nodes.renderSnapCurrX[node],
                                tickDelta
                        );
                        sharedInterpY[node] = interpolate(
                                nodes.renderSnapPrevY[node],
                                nodes.renderSnapCurrY[node],
                                tickDelta
                        );
                        sharedInterpZ[node] = interpolate(
                                nodes.renderSnapPrevZ[node],
                                nodes.renderSnapCurrZ[node],
                                tickDelta
                        );
                    }
                    offsetNodesFromCurrentToRenderedOrigin(
                            vehicle.parentEntity.getPos(),
                            renderedEntityOrigin,
                            sharedInterpX,
                            sharedInterpY,
                            sharedInterpZ,
                            nodes.count
                    );
                }

                flex.skinningPipeline.updateGpuSkinning(
                        sharedInterpX,
                        sharedInterpY,
                        sharedInterpZ,
                        nodes.count,
                        renderMoment
                );
            }
        });
    }

    private static float interpolate(double previous, double current, float tickDelta) {
        return (float) (previous + (current - previous) * tickDelta);
    }

    /**
     * Matches WorldRenderer.renderEntity exactly. Entity#getLerpedPos uses the
     * prevX/Y/Z fields instead, which are not the origin of the entity matrix.
     */
    static Vec3d getRenderedEntityOrigin(Entity entity, float tickDelta) {
        return new Vec3d(
                MathHelper.lerp((double) tickDelta, entity.lastRenderX, entity.getX()),
                MathHelper.lerp((double) tickDelta, entity.lastRenderY, entity.getY()),
                MathHelper.lerp((double) tickDelta, entity.lastRenderZ, entity.getZ())
        );
    }

    private static void offsetNodesFromCurrentToRenderedOrigin(
            Vec3d currentOrigin,
            Vec3d renderedOrigin,
            float[] x,
            float[] y,
            float[] z,
            int count
    ) {
        float offsetX = (float) (currentOrigin.x - renderedOrigin.x);
        float offsetY = (float) (currentOrigin.y - renderedOrigin.y);
        float offsetZ = (float) (currentOrigin.z - renderedOrigin.z);
        for (int node = 0; node < count; node++) {
            x[node] += offsetX;
            y[node] += offsetY;
            z[node] += offsetZ;
        }
    }

    private static void ensureInterpolationCapacity(int nodeCount) {
        if (nodeCount <= sharedInterpX.length) {
            return;
        }
        sharedInterpX = Utility.expand(sharedInterpX, nodeCount);
        sharedInterpY = Utility.expand(sharedInterpY, nodeCount);
        sharedInterpZ = Utility.expand(sharedInterpZ, nodeCount);
    }
}
