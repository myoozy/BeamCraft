package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.JBeamLoader;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import net.minecraft.client.MinecraftClient;
import net.minecraft.entity.Entity;

import java.util.HashMap;
import java.util.Map;

public class ClientVehicleManager {
    // 实体 ID 映射物理载具实例
    private static final Map<Integer, SoftBodyVehicle> VEHICLE_MAP = new HashMap<>();

    public static void update(MinecraftClient client) {
        if (client.world == null) {
            if (!VEHICLE_MAP.isEmpty()) {
                VEHICLE_MAP.values().forEach(BeamCraftClient.PHYSICS_WORLD::removeVehicle);
                VEHICLE_MAP.clear();
            }
            return;
        }

        // 1. 扫描存活的实体
        for (Entity entity : client.world.getEntities()) {
            if (entity instanceof PhysicsVehicleEntity vehicleEntity) {
                int id = vehicleEntity.getId();

                // 2. 发现新实体且尚未组装物理模型
                if (!VEHICLE_MAP.containsKey(id)) {
                    String rootPart = vehicleEntity.getRootPartName();
                    String pcFile = vehicleEntity.getPcFileName();

                    if (!rootPart.isEmpty()) {
                        SoftBodyVehicle softBody = new SoftBodyVehicle(vehicleEntity);

                        // 在客户端独立完成数据加载与组装
                        Map<String, com.google.gson.JsonObject> localRegistry = new HashMap<>();
                        Map<String, String> localConfig = new HashMap<>();
                        JBeamLoader.loadVehicle(BeamCraftClient.VEHICLES_DIR, rootPart, pcFile, localRegistry, localConfig);

                        JBeamAssembler assembler = new JBeamAssembler();
                        assembler.assembleVehicle(rootPart, localConfig, localRegistry, softBody);

                        softBody.nodes.rotateNodes(client.player.getYaw(), 0, 0);

                        // 直接调用现有的 PhysicsWorld 接口安全添加
                        BeamCraftClient.PHYSICS_WORLD.addVehicle(softBody);
                        VEHICLE_MAP.put(id, softBody);
                    }
                }
            }
        }

        // 3. 回收已被移除的实体物理资源
        VEHICLE_MAP.entrySet().removeIf(entry -> {
            SoftBodyVehicle vehicle = entry.getValue();
            if (vehicle.parentEntity == null || vehicle.parentEntity.isRemoved()) {
                // 直接调用现有的 PhysicsWorld 接口安全移除
                BeamCraftClient.PHYSICS_WORLD.removeVehicle(vehicle);
                return true;
            }
            return false;
        });
    }

    public static SoftBodyVehicle getVehicle(int entityId) {
        return VEHICLE_MAP.get(entityId);
    }
}