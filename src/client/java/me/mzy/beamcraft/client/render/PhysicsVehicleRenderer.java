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
    private ComputeSkinningPipeline pipeline = null;

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

        if (this.pipeline == null) {
            this.pipeline = new ComputeSkinningPipeline();
            this.pipeline.init(flex, 2000);
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
        this.pipeline.dispatchCompute(interpNodeX, interpNodeY, interpNodeZ, nodeCount);

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

        // 5. 彻底移除额外的 Yaw 旋转！
        // 当前的 matrixStack 已经完美包含了正确的摄像机旋转和实体平移位置。

        // 6. 执行绘制 (VertexBuffer 会自动将 matrix 传给 Shader)
        this.pipeline.mcVbo.bind();
        this.pipeline.mcVbo.draw(matrixStack.peek().getPositionMatrix(), RenderSystem.getProjectionMatrix(), RenderSystem.getShader());
        VertexBuffer.unbind();

        // 7. 恢复环境状态，避免影响后续其他实体的渲染
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().disable();
        RenderSystem.disableCull();

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}