package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.client.BeamCraftClient;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.client.gl.ShaderProgram;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.*;
import net.minecraft.util.Identifier;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.RotationAxis;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL31;

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

        FlexbodyContainer flex = vehicle.flexbodies;

        // 1. 初始化：只在车辆加载的第一帧运行
        if (!flex.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(flex, vehicle);

            // 构建空白画布 (VBO)
            if (flex.skinningMeshManager == null) flex.skinningMeshManager = new SkinningMeshManager();
            flex.skinningMeshManager.buildStaticVbo(flex);

            // 构建静态基因库 (通道 1)
            if (flex.rigTbo == null) flex.rigTbo = new RiggingTBO();
            flex.rigTbo.init(flex);
        }

        int vCount = flex.totalVertexCount;
        if (vCount == 0 || flex.skinningMeshManager == null || flex.skinningMeshManager.staticSkinningVbo == null) return;

        // 构建动态节点库 (通道 2)
        NodeContainer nodes = vehicle.nodes;
        if (flex.nodeTbo == null) {
            flex.nodeTbo = new DynamicNodeTBO();
            flex.nodeTbo.init(nodes.count + 500);
        }

        int nodeCount = nodes.count;
        if (nodeCount > interpNodeX.length) {
            interpNodeX = Utility.expand(interpNodeX, nodeCount);
            interpNodeY = Utility.expand(interpNodeY, nodeCount);
            interpNodeZ = Utility.expand(interpNodeZ, nodeCount);
        }

        // 2. 更新本帧物理节点数据
        for (int n = 0; n < nodeCount; n++) {
            interpNodeX[n] = (float) (nodes.renderSnapPrevX[n] + (nodes.renderSnapCurrX[n] - nodes.renderSnapPrevX[n]) * partialTicks);
            interpNodeY[n] = (float) (nodes.renderSnapPrevY[n] + (nodes.renderSnapCurrY[n] - nodes.renderSnapPrevY[n]) * partialTicks);
            interpNodeZ[n] = (float) (nodes.renderSnapPrevZ[n] + (nodes.renderSnapCurrZ[n] - nodes.renderSnapPrevZ[n]) * partialTicks);
        }
        flex.nodeTbo.update(interpNodeX, interpNodeY, interpNodeZ, nodeCount);

        // 3. 配置渲染管线状态
        RenderSystem.setShaderTexture(0, getTexture(entity));
        RenderSystem.setShader(() -> BeamCraftClient.skinningShader); // 使用自定义 Shader

        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.enableCull();

        // 4. 绑定我们的两个 TBO 到绝对安全的通道 (4 和 5)
        if (flex.rigTbo != null) flex.rigTbo.bind(org.lwjgl.opengl.GL13.GL_TEXTURE4);
        flex.nodeTbo.bind(org.lwjgl.opengl.GL13.GL_TEXTURE5);

        // 5. 安全地给 Shader 的 Uniform 变量赋值
        net.minecraft.client.gl.ShaderProgram shader = BeamCraftClient.skinningShader;
        if (shader != null && shader.getGlRef() != -1) {
            int shaderId = shader.getGlRef();
            int currentProgram = org.lwjgl.opengl.GL11.glGetInteger(org.lwjgl.opengl.GL20.GL_CURRENT_PROGRAM);
            org.lwjgl.opengl.GL20.glUseProgram(shaderId);

            // 告诉显卡：去通道 4 和 5 拿数据！
            int locRig = org.lwjgl.opengl.GL20.glGetUniformLocation(shaderId, "u_Rigging");
            if (locRig != -1) org.lwjgl.opengl.GL20.glUniform1i(locRig, 4);

            int locNodes = org.lwjgl.opengl.GL20.glGetUniformLocation(shaderId, "u_PhysicsNodes");
            if (locNodes != -1) org.lwjgl.opengl.GL20.glUniform1i(locNodes, 5);

            org.lwjgl.opengl.GL20.glUseProgram(currentProgram);
        }

        // 6. 核心视角修复：抵消 Entity 自动附加的 Yaw，让世界轴对齐完美生效
        matrixStack.push();
        float currentYaw = MathHelper.lerp(partialTicks, entity.prevYaw, entity.getYaw());
        matrixStack.multiply(RotationAxis.POSITIVE_Y.rotationDegrees(currentYaw - 180.0F));

        // 7. 发送绘制指令给底层显卡
        flex.skinningMeshManager.staticSkinningVbo.bind();
        flex.skinningMeshManager.staticSkinningVbo.draw(matrixStack.peek().getPositionMatrix(), RenderSystem.getProjectionMatrix(), RenderSystem.getShader());
        VertexBuffer.unbind();

        matrixStack.pop();

        // 8. 彻底清理环境，防止你的车搞坏 UI 或其他实体的渲染
        RenderSystem.activeTexture(GL13.GL_TEXTURE2);
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        RenderSystem.activeTexture(GL13.GL_TEXTURE1);
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        RenderSystem.activeTexture(GL13.GL_TEXTURE0);

        // 绘制完成后，记得把 4 和 5 解绑
        RenderSystem.activeTexture(org.lwjgl.opengl.GL13.GL_TEXTURE5);
        org.lwjgl.opengl.GL11.glBindTexture(org.lwjgl.opengl.GL31.GL_TEXTURE_BUFFER, 0);
        RenderSystem.activeTexture(org.lwjgl.opengl.GL13.GL_TEXTURE4);
        org.lwjgl.opengl.GL11.glBindTexture(org.lwjgl.opengl.GL31.GL_TEXTURE_BUFFER, 0);
        RenderSystem.activeTexture(org.lwjgl.opengl.GL13.GL_TEXTURE0);

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}