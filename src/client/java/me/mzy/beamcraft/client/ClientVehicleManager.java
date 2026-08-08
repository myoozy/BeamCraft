package me.mzy.beamcraft.client;

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

import java.util.HashMap;
import java.util.Map;

public final class ClientVehicleManager {

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
        JBeamLoader.loadVehicle(
                BeamCraftClient.VEHICLES_DIR,
                rootPart,
                vehicleEntity.getPcFileName(),
                localRegistry,
                localConfig
        );

        DaeMeshLoader.requireVehicleModels(BeamCraftClient.VEHICLES_DIR, rootPart);
        boolean assembled = new JBeamAssembler().assembleVehicle(
                rootPart,
                localConfig,
                localRegistry,
                softBody
        );
        if (!assembled) {
            DaeMeshLoader.releaseVehicleModels(rootPart);
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
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            minZ = Math.min(minZ, z);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
            maxZ = Math.max(maxZ, z);
        }

        double entityX = vehicle.parentEntity.getX();
        double entityY = vehicle.parentEntity.getY();
        double entityZ = vehicle.parentEntity.getZ();
        vehicle.parentEntity.setBoundingBox(new Box(
                minX + entityX,
                minY + entityY,
                minZ + entityZ,
                maxX + entityX,
                maxY + entityY,
                maxZ + entityZ
        ));
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
