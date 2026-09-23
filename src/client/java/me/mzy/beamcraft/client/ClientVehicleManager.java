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
import net.minecraft.entity.Entity;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class ClientVehicleManager {

    private static final double MIN_INTERACTION_BOUNDS_SIDE = 0.75;
    private static final double MIN_RENDER_BOUNDS_SPAN = 4.0;
    private static final double RENDER_DEFORMATION_MARGIN = 2.0;
    private static float[] boundsScratch = new float[NodeContainer.INIT_NODE_CAP];

    static record BoundsProfile(double interactionSide, double renderMaxSpan) {}

    private static final Map<Integer, SoftBodyVehicle> VEHICLE_MAP = new HashMap<>();
    private static final VehicleLoadFailureCache LOAD_FAILURES = new VehicleLoadFailureCache();

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
            return;
        }

        // Entity ids are scoped to one live client world and can be reused after
        // unloads or dimension changes. Retain a soft body only while the world's
        // current id lookup still points at its exact parent entity instance.
        VEHICLE_MAP.entrySet().removeIf(entry -> {
            SoftBodyVehicle vehicle = entry.getValue();
            Entity current = client.world.getEntityById(entry.getKey());
            if (vehicle.parentEntity != null
                    && !vehicle.parentEntity.isRemoved()
                    && current == vehicle.parentEntity) {
                return false;
            }
            releaseVehicle(vehicle);
            return true;
        });

        for (Entity entity : client.world.getEntities()) {
            if (!(entity instanceof PhysicsVehicleEntity vehicleEntity)) {
                continue;
            }

            int entityId = vehicleEntity.getId();
            SoftBodyVehicle existing = VEHICLE_MAP.get(entityId);
            LOAD_FAILURES.removeStale(entityId, vehicleEntity.getUuid());
            if (existing != null) {
                updateEntityBounds(existing);
            } else if (LOAD_FAILURES.shouldAttempt(
                    entityId,
                    vehicleEntity.getUuid(),
                    vehicleEntity.getRootPartName(),
                    vehicleEntity.getPcFileName())) {
                createVehicle(client, vehicleEntity);
            }
        }

    }

    private static void createVehicle(MinecraftClient client, PhysicsVehicleEntity vehicleEntity) {
        String rootPart = vehicleEntity.getRootPartName();
        if (rootPart.isEmpty()) {
            return;
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
            return;
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
            return;
        }
        LoadTiming.log("[load 2/4] assembly", phaseStart);

        boolean materialsAcquired = false;
        boolean meshesAcquired = false;
        boolean physicsWorldOwnsVehicle = false;
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
            BeamCraftClient.PHYSICS_WORLD.addVehicle(softBody);
            physicsWorldOwnsVehicle = true;
            VEHICLE_MAP.put(vehicleEntity.getId(), softBody);
            LOAD_FAILURES.recordSuccess(vehicleEntity.getId());
        } catch (RuntimeException failure) {
            if (physicsWorldOwnsVehicle) {
                BeamCraftClient.PHYSICS_WORLD.removeVehicle(softBody);
            } else {
                softBody.clear();
            }
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
        }
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
        if (VEHICLE_MAP.isEmpty()) {
            return;
        }

        for (SoftBodyVehicle vehicle : VEHICLE_MAP.values()) {
            releaseVehicle(vehicle);
        }
        VEHICLE_MAP.clear();
    }

    private static void releaseVehicle(SoftBodyVehicle vehicle) {
        DaeMeshLoader.releaseVehicleModels(vehicle.flexbodies.vehicleNamespace);
        MaterialLibrary.releaseMaterials(vehicle.flexbodies.vehicleNamespace);
        // PhysicsWorld.removeVehicle() clears the vehicle and therefore closes
        // its GPU skinning pipeline exactly once.
        BeamCraftClient.PHYSICS_WORLD.removeVehicle(vehicle);
    }

    public static SoftBodyVehicle getVehicle(int entityId) {
        return VEHICLE_MAP.get(entityId);
    }

    public static void initRenderHooks() {
        // Iris renders its shadow map when vanilla enters renderSky(), before
        // Fabric's BEFORE_ENTITIES event. Prepare skinning at START so shadow
        // and main entity passes consume the same frame's node positions.
        WorldRenderEvents.START.register(context -> {
            MinecraftClient client = MinecraftClient.getInstance();
            if (client.world == null || VEHICLE_MAP.isEmpty()) {
                return;
            }

            float tickDelta = context.tickCounter().getTickDelta(true);
            long renderMoment = client.world.getTime() << 32
                    ^ Integer.toUnsignedLong(Float.floatToRawIntBits(tickDelta));

            for (SoftBodyVehicle vehicle : VEHICLE_MAP.values()) {
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
