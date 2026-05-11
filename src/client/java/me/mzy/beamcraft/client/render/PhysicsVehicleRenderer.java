package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.physics.SoftBodyVehicle;
import me.mzy.beamcraft.physics.FlexbodyContainer;
import me.mzy.beamcraft.physics.NodeContainer;

import net.minecraft.client.render.*;
import net.minecraft.util.Identifier;

// 核心渲染类
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;

// 1.21 的 MatrixStack 路径
import net.minecraft.client.util.math.MatrixStack;

// 数学库 (1.21 强制使用 JOML)
import org.joml.Matrix4f;

/**
 * 客户端专属：车辆软体蒙皮渲染器
 * 必须放置在 src/client/java 目录中，彻底免疫服务器侧崩溃风险
 */
public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    // 临时贴图占位符，后续替换为从 DAE/JBeam 解析出的真实纹理路径
    private static final Identifier DEFAULT_TEXTURE = Identifier.of("beamcraft", "textures/entity/vehicle_default.png");

    public PhysicsVehicleRenderer(EntityRendererFactory.Context ctx) {
        super(ctx);
    }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) {
        return DEFAULT_TEXTURE;
    }

    // 🚀 强行免疫 MC 的视口剔除误伤
    @Override
    public boolean shouldRender(PhysicsVehicleEntity entity, Frustum frustum, double camX, double camY, double camZ) {
        super.shouldRender(entity, frustum, camX, camY, camZ);
        // 无条件返回 true，确保无论看向哪里，蒙皮算法都能满血推流
        return true;
    }

    /**
     * 🚀 核心渲染循环
     */
    @Override
    public void render(PhysicsVehicleEntity entity, float entityYaw, float partialTicks, MatrixStack matrixStack, VertexConsumerProvider buffer, int packedLight) {

        SoftBodyVehicle vehicle = entity.softBody;
        if (vehicle == null) return;
        BeamCraft.LOGGER.info(entity.toString());

        // 如果没有绑定就花几毫秒绑定
        if (!vehicle.flexbodies.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle.nodes);
            System.out.println("🎨 懒加载触发完毕！当前实例绑定顶点总数: " + vehicle.flexbodies.totalVertexCount);
        }

        NodeContainer nodes = vehicle.nodes;
        FlexbodyContainer flex = vehicle.flexbodies;
        int vCount = flex.totalVertexCount;
        if (vCount == 0) {
            System.out.println("⚠️ 严重警告：vCount 为 0，渲染器拒绝推流！");
            return;
        }

        // 1. 获取 1.21 规范的矩阵栈帧 Entry
        MatrixStack.Entry entry = matrixStack.peek();
        Matrix4f modelMat = entry.getPositionMatrix();

        // 2. 准备实体兼任的光影渲染层 (Cutout 允许透明贴图且无剔除)
        VertexConsumer builder = buffer.getBuffer(RenderLayer.getEntityCutoutNoCull(getTexture(entity)));

        // 3. 🚀 极致性能 CPU 蒙皮拉扯与原生写入 (全程 0 内存分配)
        for (int i = 0; i < vCount; i++) {
            int c  = flex.vCenterNode[i];
            int nx = flex.vVxNode[i];
            int ny = flex.vVyNode[i];

            // 结合 Minecraft partialTicks 进行 20Hz 双缓冲视觉平滑插值
            double cx = nodes.renderSnapPrevX[c] + (nodes.renderSnapCurrX[c] - nodes.renderSnapPrevX[c]) * partialTicks;
            double cy = nodes.renderSnapPrevY[c] + (nodes.renderSnapCurrY[c] - nodes.renderSnapPrevY[c]) * partialTicks;
            double cz = nodes.renderSnapPrevZ[c] + (nodes.renderSnapCurrZ[c] - nodes.renderSnapPrevZ[c]) * partialTicks;

            double nxx = nodes.renderSnapPrevX[nx] + (nodes.renderSnapCurrX[nx] - nodes.renderSnapPrevX[nx]) * partialTicks;
            double nxy = nodes.renderSnapPrevY[nx] + (nodes.renderSnapCurrY[nx] - nodes.renderSnapPrevY[nx]) * partialTicks;
            double nxz = nodes.renderSnapPrevZ[nx] + (nodes.renderSnapCurrZ[nx] - nodes.renderSnapPrevZ[nx]) * partialTicks;
            double vxx = nxx - cx, vxy = nxy - cy, vxz = nxz - cz;

            double nyx = nodes.renderSnapPrevX[ny] + (nodes.renderSnapCurrX[ny] - nodes.renderSnapPrevX[ny]) * partialTicks;
            double nyy = nodes.renderSnapPrevY[ny] + (nodes.renderSnapCurrY[ny] - nodes.renderSnapPrevY[ny]) * partialTicks;
            double nyz = nodes.renderSnapPrevZ[ny] + (nodes.renderSnapCurrZ[ny] - nodes.renderSnapPrevZ[ny]) * partialTicks;
            double vyx = nyx - cx, vyy = nyy - cy, vyz = nyz - cz;

            double vzx, vzy, vzz;

            if (flex.vUseCrossZ[i]) {
                vzx = vxy * vyz - vxz * vyy;
                vzy = vxz * vyx - vxx * vyz;
                vzz = vxx * vyy - vxy * vyx;
            } else {
                int nz = flex.vVzNode[i];
                double nzx = nodes.renderSnapPrevX[nz] + (nodes.renderSnapCurrX[nz] - nodes.renderSnapPrevX[nz]) * partialTicks;
                double nzy = nodes.renderSnapPrevY[nz] + (nodes.renderSnapCurrY[nz] - nodes.renderSnapPrevY[nz]) * partialTicks;
                double nzz = nodes.renderSnapPrevZ[nz] + (nodes.renderSnapCurrZ[nz] - nodes.renderSnapPrevZ[nz]) * partialTicks;
                vzx = nzx - cx; vzy = nzy - cy; vzz = nzz - cz;
            }

            // 叠加静态蒙皮权重
            float wx = flex.vWeightX[i], wy = flex.vWeightY[i], wz = flex.vWeightZ[i];
            float finalX = (float)(cx + (wx * vxx) + (wy * vyx) + (wz * vzx));
            float finalY = (float)(cy + (wx * vxy) + (wy * vyy) + (wz * vzy));
            float finalZ = (float)(cz + (wx * vxz) + (wy * vyz) + (wz * vzz));

            // 🚀 1.21 原生流式推送：彻底舍弃 .next()
            builder.vertex(modelMat, finalX, finalY, finalZ)
                    .color(255, 255, 255, 255)
                    .texture(flex.uvU[i], flex.uvV[i])
                    .overlay(OverlayTexture.DEFAULT_UV)
                    .light(packedLight) // 接收外部真实的方块光照值
                    .normal(entry, flex.normalX[i], flex.normalY[i], flex.normalZ[i]);
        }

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}