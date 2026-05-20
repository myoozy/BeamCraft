package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.physics.*;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.utility.Utility;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.minecraft.client.MinecraftClient;
import net.minecraft.entity.Entity;
import org.lwjgl.opengl.GL43;

import java.util.HashMap;
import java.util.Map;

public class ClientVehicleManager {
    // 实体 ID 映射物理载具实例
    private static final Map<Integer, SoftBodyVehicle> VEHICLE_MAP = new HashMap<>();

    // 全局共享的插值数组（所有车轮流用，消灭内存分配）
    private static float[] sharedInterpX = new float[NodeContainer.INIT_NODE_CAP];
    private static float[] sharedInterpY = new float[NodeContainer.INIT_NODE_CAP];
    private static float[] sharedInterpZ = new float[NodeContainer.INIT_NODE_CAP];

    // 防光影重复计算的时间戳
    private static long lastComputedFrameTime = 0;

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

                        DaeMeshLoader.requireVehicleModels(BeamCraftClient.VEHICLES_DIR, rootPart);

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
                DaeMeshLoader.releaseVehicleModels(vehicle.flexbodies.vehicleNamespace);
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

    public static void initRenderHooks() {
        // 注册在 Minecraft 开始绘制所有实体之前
        WorldRenderEvents.BEFORE_ENTITIES.register(context -> {
            // 光影模组（如 Iris）一帧可能会调用这里多次（比如为了画阴影）
            // 我们利用系统时间戳拦截，保证一物理帧/渲染帧只做一次 Compute
            long currentTime = System.currentTimeMillis();
            // 如果距离上次计算小于 2 毫秒（通常一帧是 16ms），说明是光影在反复横跳，直接跳过
            if (currentTime - lastComputedFrameTime < 2) return;
            lastComputedFrameTime = currentTime;

            float partialTicks = context.tickCounter().getTickDelta(true);
            boolean dispatchedAny = false;

            // 遍历所有活着的车
            for (SoftBodyVehicle vehicle : VEHICLE_MAP.values()) {
                FlexbodyContainer flex = vehicle.flexbodies;
                NodeContainer nodes = vehicle.nodes;

                // 确保它准备好了
                if (flex == null || flex.skinningPipeline.customPosNormVbo == -1) continue;

                int nodeCount = nodes.count;
                if (nodeCount == 0) continue;

                // 动态扩容共享数组 (如果这辆车节点特别多)
                if (nodeCount > sharedInterpX.length) {
                    sharedInterpX = Utility.expand(sharedInterpX, nodeCount);
                    sharedInterpY = Utility.expand(sharedInterpY, nodeCount);
                    sharedInterpZ = Utility.expand(sharedInterpZ, nodeCount);
                }

                // 插值逻辑
                for (int n = 0; n < nodeCount; n++) {
                    sharedInterpX[n] = (float) (nodes.renderSnapPrevX[n] + (nodes.renderSnapCurrX[n] - nodes.renderSnapPrevX[n]) * partialTicks);
                    sharedInterpY[n] = (float) (nodes.renderSnapPrevY[n] + (nodes.renderSnapCurrY[n] - nodes.renderSnapPrevY[n]) * partialTicks);
                    sharedInterpZ[n] = (float) (nodes.renderSnapPrevZ[n] + (nodes.renderSnapCurrZ[n] - nodes.renderSnapPrevZ[n]) * partialTicks);
                }

                // 派发 Compute Shader（CPU 会把数组里的数据传给显存，然后立刻返回，无需等待 GPU）
                flex.skinningPipeline.dispatchCompute(sharedInterpX, sharedInterpY, sharedInterpZ, nodeCount);
                dispatchedAny = true;
            }

            // 有车都派发完了，放一个全局屏障！
            // 告诉显卡：等上面那一堆蒙皮全都算完，才准往下画画面！
            if (dispatchedAny) {
                GL43.glMemoryBarrier(GL43.GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
            }
        });
    }
}