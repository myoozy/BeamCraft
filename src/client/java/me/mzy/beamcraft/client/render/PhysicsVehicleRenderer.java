package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.client.render.*;
import net.minecraft.entity.Entity;
import net.minecraft.util.Identifier;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import org.joml.Matrix4f;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of("beamcraft", "textures/entity/vehicle_default.png");

    float[] interpNodeX = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeY = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeZ = new float[NodeContainer.INIT_NODE_CAP];

    public PhysicsVehicleRenderer(EntityRendererFactory.Context ctx) { super(ctx); }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) { return DEFAULT_TEXTURE; }

    /**
     * 🚀 极限性能加速器：快速平方根倒数 (Fast InvSqrt)
     * 消除高频着色指令阻塞，确保每帧数万个顶点法线重构全域丝滑。
     */
    public static float fastInvSqrt(float x) {
        float xhalf = 0.5f * x;
        int i = Float.floatToRawIntBits(x);
        i = 0x5f3759df - (i >> 1);
        x = Float.intBitsToFloat(i);
        x = x * (1.5f - xhalf * x * x);
        return x;
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
        if (vCount == 0) {
            BeamCraft.LOGGER.info(flex.vehicleNamespace + "vertex count = 0");
            return;
        }

        VehicleRenderBuffer vboBuffer = flex.vboBuffer;

        if (vboBuffer.isVboNull()) {
            vboBuffer.init();
        }

        int nodeCount = nodes.count;
        if (nodeCount > interpNodeX.length) {
            interpNodeX = Utility.expand(interpNodeX, nodeCount);
            interpNodeY = Utility.expand(interpNodeY, nodeCount);
            interpNodeZ = Utility.expand(interpNodeZ, nodeCount);
        }

        for (int n = 0; n < nodeCount; n++) {
            interpNodeX[n] = (float)(nodes.renderSnapPrevX[n] + (nodes.renderSnapCurrX[n] - nodes.renderSnapPrevX[n]) * partialTicks);
            interpNodeY[n] = (float)(nodes.renderSnapPrevY[n] + (nodes.renderSnapCurrY[n] - nodes.renderSnapPrevY[n]) * partialTicks);
            interpNodeZ[n] = (float)(nodes.renderSnapPrevZ[n] + (nodes.renderSnapCurrZ[n] - nodes.renderSnapPrevZ[n]) * partialTicks);
        }

        RenderSystem.setShaderTexture(0, DEFAULT_TEXTURE);
        // 使用你那边没报错的那个 Shader 名字（NoCull 的即可）
        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.disableCull();
        RenderSystem.enableBlend();

        // 🌟 一行代码完成上传显存 + 安全绘制！
        vboBuffer.updateAndDraw(flex, interpNodeX, interpNodeY, interpNodeZ, packedLight, matrixStack);

        RenderSystem.enableCull();
        RenderSystem.disableBlend();

        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}