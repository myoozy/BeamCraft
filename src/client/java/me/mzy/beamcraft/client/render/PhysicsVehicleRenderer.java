package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.GameRenderer;
import net.minecraft.client.render.VertexConsumerProvider;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.Identifier;
import org.lwjgl.opengl.GL30;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of(
            "beamcraft",
            "textures/entity/vehicle_default.png"
    );

    public PhysicsVehicleRenderer(EntityRendererFactory.Context context) {
        super(context);
    }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) {
        return DEFAULT_TEXTURE;
    }

    @Override
    public void render(
            PhysicsVehicleEntity entity,
            float entityYaw,
            float partialTicks,
            MatrixStack matrixStack,
            VertexConsumerProvider vertexConsumers,
            int packedLight
    ) {
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) {
            return;
        }

        FlexbodyContainer flex = vehicle.flexbodies;
        if (!flex.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(flex, vehicle);
        }
        if (flex.totalVertexCount == 0) {
            return;
        }

        if (!flex.skinningPipeline.isReady()
                && !flex.skinningPipeline.init(flex, vehicle.nodes.count)) {
            return;
        }
        if (!flex.skinningPipeline.hasValidOutput()) {
            return;
        }

        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.setShaderTexture(0, getTexture(entity));
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.enableCull();
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().enable();

        // The private VAO was configured once during pipeline initialization.
        // Position and normal come from the GPU skinning output while UV, color,
        // and overlay remain in Minecraft's static vertex buffer.
        flex.skinningPipeline.mcVbo.bind();

        int blockLight = packedLight & 0xFFFF;
        int skyLight = packedLight >>> 16 & 0xFFFF;
        GL30.glVertexAttribI2i(
                flex.skinningPipeline.getLightAttributeIndex(),
                blockLight,
                skyLight
        );

        org.joml.Matrix4f modelView = new org.joml.Matrix4f(RenderSystem.getModelViewMatrix());
        modelView.mul(matrixStack.peek().getPositionMatrix());
        flex.skinningPipeline.mcVbo.draw(
                modelView,
                RenderSystem.getProjectionMatrix(),
                RenderSystem.getShader()
        );

        VertexBuffer.unbind();
        MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager().disable();
        RenderSystem.disableCull();

        super.render(entity, entityYaw, partialTicks, matrixStack, vertexConsumers, packedLight);
    }
}
