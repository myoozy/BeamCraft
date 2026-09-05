package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.config.BeamCraftConfigManager;
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

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class ClientVehicleManager {

    private static final double ROBUST_BOUNDS_MIN_RADIUS = 4.0;
    private static final double ROBUST_BOUNDS_RADIUS_PADDING = 2.0;
    private static final double ROBUST_BOUNDS_BOX_PADDING = 1.0;
    private static float[] boundsScratch = new float[NodeContainer.INIT_NODE_CAP];

    private static final Map<Integer, SoftBodyVehicle> VEHICLE_MAP = new HashMap<>();

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

        for (Entity entity : client.world.getEntities()) {
            if (!(entity instanceof PhysicsVehicleEntity vehicleEntity)) {
                continue;
            }

            int entityId = vehicleEntity.getId();
            SoftBodyVehicle existing = VEHICLE_MAP.get(entityId);
            if (existing == null) {
                createVehicle(client, vehicleEntity);
            } else {
                updateEntityBounds(existing);
            }
        }

        VEHICLE_MAP.entrySet().removeIf(entry -> {
            SoftBodyVehicle vehicle = entry.getValue();
            if (vehicle.parentEntity != null && !vehicle.parentEntity.isRemoved()) {
                return false;
            }

            releaseVehicle(vehicle);
            return true;
        });
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
        JBeamLoader.loadVehicle(
                assetRoots,
                rootPart,
                vehicleEntity.getPcFileName(),
                localRegistry,
                localConfig
        );

        DaeMeshLoader.requireVehicleModels(assetRoots, rootPart);
        MaterialLibrary.requireMaterials(assetRoots, rootPart);
        boolean assembled = new JBeamAssembler().assembleVehicle(
                rootPart,
                localConfig,
                localRegistry,
                softBody
        );
        if (!assembled) {
            DaeMeshLoader.releaseVehicleModels(rootPart);
            MaterialLibrary.releaseMaterials(rootPart);
            System.err.println("Vehicle assembly failed for entity " + vehicleEntity.getId());
            return;
        }

        float playerYaw = client.player != null ? client.player.getYaw() : 0.0f;
        softBody.nodes.rotateNodes(playerYaw, 0, 0);
        BeamCraftClient.PHYSICS_WORLD.addVehicle(softBody);
        VEHICLE_MAP.put(vehicleEntity.getId(), softBody);
    }

    private static void updateEntityBounds(SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        if (nodes.count == 0 || vehicle.parentEntity == null) {
            return;
        }

        double entityX = vehicle.parentEntity.getX();
        double entityY = vehicle.parentEntity.getY();
        double entityZ = vehicle.parentEntity.getZ();
        Box localBounds = computeRobustLocalBounds(nodes);
        vehicle.parentEntity.setBoundingBox(localBounds.offset(entityX, entityY, entityZ));
    }

    /**
     * Builds a render/interaction envelope around the main vehicle body without
     * allowing a detached node to expand the Minecraft entity AABB indefinitely.
     * The coordinate-wise median follows the majority of the nodes and is not
     * displaced by a small detached group. The undeformed vehicle diagonal
     * supplies a vehicle-specific acceptance radius, so this also works for
     * vehicles much larger than a passenger car.
     */
    static Box computeRobustLocalBounds(NodeContainer nodes) {
        ensureBoundsScratchCapacity(nodes.count);
        float centerX = NodeContainer.medianOfFinite(nodes.renderSnapCurrX, nodes.count, boundsScratch);
        float centerY = NodeContainer.medianOfFinite(nodes.renderSnapCurrY, nodes.count, boundsScratch);
        float centerZ = NodeContainer.medianOfFinite(nodes.renderSnapCurrZ, nodes.count, boundsScratch);

        double baseMinX = Double.POSITIVE_INFINITY;
        double baseMinY = Double.POSITIVE_INFINITY;
        double baseMinZ = Double.POSITIVE_INFINITY;
        double baseMaxX = Double.NEGATIVE_INFINITY;
        double baseMaxY = Double.NEGATIVE_INFINITY;
        double baseMaxZ = Double.NEGATIVE_INFINITY;
        for (int node = 0; node < nodes.count; node++) {
            double x = nodes.baseX[node];
            double y = nodes.baseY[node];
            double z = nodes.baseZ[node];
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
                continue;
            }
            baseMinX = Math.min(baseMinX, x);
            baseMinY = Math.min(baseMinY, y);
            baseMinZ = Math.min(baseMinZ, z);
            baseMaxX = Math.max(baseMaxX, x);
            baseMaxY = Math.max(baseMaxY, y);
            baseMaxZ = Math.max(baseMaxZ, z);
        }

        double baseDiagonal = 0.0;
        if (Double.isFinite(baseMinX)) {
            baseDiagonal = Math.sqrt(
                    square(baseMaxX - baseMinX)
                            + square(baseMaxY - baseMinY)
                            + square(baseMaxZ - baseMinZ)
            );
        }
        double radius = Math.max(ROBUST_BOUNDS_MIN_RADIUS,
                baseDiagonal + ROBUST_BOUNDS_RADIUS_PADDING);
        double radiusSquared = radius * radius;

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
            double distanceSquared = square(x - centerX) + square(y - centerY) + square(z - centerZ);
            if (distanceSquared > radiusSquared) {
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
        return new Box(
                minX - ROBUST_BOUNDS_BOX_PADDING,
                minY - ROBUST_BOUNDS_BOX_PADDING,
                minZ - ROBUST_BOUNDS_BOX_PADDING,
                maxX + ROBUST_BOUNDS_BOX_PADDING,
                maxY + ROBUST_BOUNDS_BOX_PADDING,
                maxZ + ROBUST_BOUNDS_BOX_PADDING
        );
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
        WorldRenderEvents.BEFORE_ENTITIES.register(context -> {
            MinecraftClient client = MinecraftClient.getInstance();
            if (client.world == null || VEHICLE_MAP.isEmpty()) {
                return;
            }

            float tickDelta = context.tickCounter().getTickDelta(true);
            long renderNanos = System.nanoTime();
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
                boolean sampledTimeline = vehicle.renderTimeline.sample(
                        renderNanos,
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

    private static void ensureInterpolationCapacity(int nodeCount) {
        if (nodeCount <= sharedInterpX.length) {
            return;
        }
        sharedInterpX = Utility.expand(sharedInterpX, nodeCount);
        sharedInterpY = Utility.expand(sharedInterpY, nodeCount);
        sharedInterpZ = Utility.expand(sharedInterpZ, nodeCount);
    }
}
