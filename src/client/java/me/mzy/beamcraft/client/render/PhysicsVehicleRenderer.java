package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.*;
import net.minecraft.util.Identifier;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.RotationAxis;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of("beamcraft", "textures/entity/vehicle_default.png");

    float[] interpNodeX = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeY = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeZ = new float[NodeContainer.INIT_NODE_CAP];

    public PhysicsVehicleRenderer(EntityRendererFactory.Context ctx) {
        super(ctx);
    }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) {
        return DEFAULT_TEXTURE;
    }

    @Override
    public void render(PhysicsVehicleEntity entity, float entityYaw, float partialTicks, MatrixStack matrixStack, VertexConsumerProvider buffer, int packedLight) {
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) return;

        if (!vehicle.flexbodies.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle);
        }

        NodeContainer nodes = vehicle.nodes;
        FlexbodyContainer flex = vehicle.flexbodies;
        int vCount = flex.totalVertexCount;
        if (vCount == 0) return;

        if (flex.skinningPipeline.rawVboId == -1) {
            flex.skinningPipeline.init(flex, vehicle.nodes.count);
        }

        int nodeCount = nodes.count;
        if (nodeCount > interpNodeX.length) {
            interpNodeX = Utility.expand(interpNodeX, nodeCount);
            interpNodeY = Utility.expand(interpNodeY, nodeCount);
            interpNodeZ = Utility.expand(interpNodeZ, nodeCount);
        }

        // 线性插值物理节点
        for (int n = 0; n < nodeCount; n++) {
            interpNodeX[n] = (float) (nodes.renderSnapPrevX[n] + (nodes.renderSnapCurrX[n] - nodes.renderSnapPrevX[n]) * partialTicks);
            interpNodeY[n] = (float) (nodes.renderSnapPrevY[n] + (nodes.renderSnapCurrY[n] - nodes.renderSnapPrevY[n]) * partialTicks);
            interpNodeZ[n] = (float) (nodes.renderSnapPrevZ[n] + (nodes.renderSnapCurrZ[n] - nodes.renderSnapPrevZ[n]) * partialTicks);
        }

        // 调度 GPU 蒙皮计算
        flex.skinningPipeline.dispatchCompute(interpNodeX, interpNodeY, interpNodeZ, nodeCount);

        // ==========================================================
        // 核心修正：纯手动接管底层 OpenGL 状态，兼容 Fabric/Yarn
        // ==========================================================

        // 1. 设置 Shader 与纹理
        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.setShaderTexture(0, getTexture(entity));

        // 2. 修复 Z-Buffer 遮挡关系 (深度测试)
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515); // 等同于 GL11.GL_LEQUAL

        // 3. 修复背面剔除 (如果是双面渲染需求，可改为 disableCull)
        RenderSystem.enableCull();

        // 4. 开启 Minecraft 原生光照系统，防止模型变黑
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().enable();

        // 5. 执行绘制 (VertexBuffer 会自动将 matrix 传给 Shader)
        flex.skinningPipeline.mcVbo.bind();
        // 1. 获取包含相机视角的全局视图矩阵
        org.joml.Matrix4f mvp = new org.joml.Matrix4f(RenderSystem.getModelViewMatrix());

        // 2. 将全局视图矩阵 乘以 实体的平移矩阵
        // 矩阵乘法顺序至关重要：这意味着顶点先进行平移(对齐到实体位置)，然后再跟随相机旋转
                mvp.mul(matrixStack.peek().getPositionMatrix());

        // 3. 将合并后的完整矩阵传给 GPU 画图
        flex.skinningPipeline.mcVbo.draw(mvp, RenderSystem.getProjectionMatrix(), RenderSystem.getShader());

        // 7. 恢复环境状态，避免影响后续其他实体的渲染
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().disable();
        RenderSystem.disableCull();

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}