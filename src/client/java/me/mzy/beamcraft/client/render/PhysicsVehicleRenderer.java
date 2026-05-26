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
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of("beamcraft", "textures/entity/vehicle_default.png");

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

        if (flex.skinningPipeline.customPosNormVbo == -1) {
            flex.skinningPipeline.init(flex, vehicle.nodes.count);
        }

        // ==========================================================
        // 原版状态设置
        // ==========================================================
        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.setShaderTexture(0, getTexture(entity));
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.enableCull();
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().enable();

        // ==========================================================
        // 🔥 核心劫持：指针掉包
        // ==========================================================
        // 1. 让原版绑定它的 VAO (此时记录本上写的是去 mcVbo 读所有数据)
        flex.skinningPipeline.mcVbo.bind();

        // 2. 绑定我们自己的纯净 VBO
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, flex.skinningPipeline.customPosNormVbo);

        // TODO: 找个万无一失的方法，替换掉硬编码

        // 3. 强行篡改 0 号属性 (Position) 指南
        GL20.glEnableVertexAttribArray(0);
        // 告诉显卡：读 3 个 Float，步长 24 字节，从 0 字节开始读
        GL20.glVertexAttribPointer(0, 3, GL11.GL_FLOAT, false, 24, 0);

        // 4. 强行篡改 5 号属性 (Normal) 指南
        GL20.glEnableVertexAttribArray(5);
        // 告诉显卡：读 3 个 Float，步长 24 字节，跳过前 12 字节 (即跳过位置) 开始读
        GL20.glVertexAttribPointer(5, 3, GL11.GL_FLOAT, false, 24, 12);

        // 5. 关闭 4 号光照属性的数组读取，改用全局常量注入
        GL20.glDisableVertexAttribArray(4); // 告诉显卡：别去 VBO 里翻光照数据了！

        // 拆解 Minecraft 的 packedLight (高16位是天空光，低16位是方块光)
        int blockLight = packedLight & 0xFFFF;
        int skyLight = (packedLight >> 16) & 0xFFFF;
        // 注入全局静态属性，当前 DrawCall 的所有顶点都将自动使用这个动态光照
        org.lwjgl.opengl.GL30.glVertexAttribI2i(4, blockLight, skyLight);

        // ==========================================================
        // 发送绘制指令
        // ==========================================================
        org.joml.Matrix4f mvp = new org.joml.Matrix4f(RenderSystem.getModelViewMatrix());
        mvp.mul(matrixStack.peek().getPositionMatrix());

        // 此时画图，显卡会从我们的池子读位置和法线，从 mcVbo 读 UV 等数据
        flex.skinningPipeline.mcVbo.draw(mvp, RenderSystem.getProjectionMatrix(), RenderSystem.getShader());

        // 绘制完后，记得把 4 号属性重新启用，避免污染原版其他渲染
        GL20.glEnableVertexAttribArray(4);

        // 解绑我们强行挂载的自定义 VBO，把全局状态物归原主
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);

        // 彻底解绑当前被污染的VAO
        VertexBuffer.unbind();// 或者调用 GL30.glBindVertexArray(0);

        // 恢复环境
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().disable();
        RenderSystem.disableCull();

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}