package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.material.MaterialDefinition;
import me.mzy.beamcraft.client.material.MaterialLibrary;
import me.mzy.beamcraft.client.material.MaterialRenderPlan;
import me.mzy.beamcraft.client.material.MaterialRenderPlanner;
import me.mzy.beamcraft.client.material.TextureResource;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.render.ComputeSkinningPipeline.SubMeshRange;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.GameRenderer;
import net.minecraft.client.render.VertexConsumerProvider;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.Identifier;

import java.util.HashSet;
import java.util.Set;
import java.util.function.IntSupplier;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of(
            "minecraft",
            "textures/block/white_concrete.png"
    );

    // Rate-limited (once per key) diagnostics; never per-frame.
    private static final Set<String> WARNED_MISSING_MATERIALS = new HashSet<>();
    private static final Set<String> WARNED_UNRESOLVED_TEXTURES = new HashSet<>();

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

        if (!flex.skinningPipeline.hasValidOutput()) {
            return;
        }

        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.enableCull();
        var gameRenderer = MinecraftClient.getInstance().gameRenderer;
        var lightmap = gameRenderer.getLightmapTextureManager();
        var overlay = gameRenderer.getOverlayTexture();
        lightmap.enable();
        overlay.setupOverlayColor();
        try {
            org.joml.Matrix4f modelView = new org.joml.Matrix4f(RenderSystem.getModelViewMatrix());
            modelView.mul(matrixStack.peek().getPositionMatrix());
            org.joml.Matrix4f projection = RenderSystem.getProjectionMatrix();

            if (!flex.skinningPipeline.getSubMeshRanges().isEmpty()) {
                renderSubMeshes(flex, modelView, projection, packedLight);
            } else {
                // No per-material ranges computed: draw the whole mesh against the
                // shared white fallback texture, which is always a valid Sampler0.
                int previousTexture0 = RenderSystem.getShaderTexture(0);
                float[] previousColor = RenderSystem.getShaderColor();
                float prevR = previousColor[0], prevG = previousColor[1];
                float prevB = previousColor[2], prevA = previousColor[3];
                try {
                    RenderSystem.setShaderTexture(0, VehicleTextureUploader.INSTANCE.getWhiteTexture());
                    RenderSystem.setShaderColor(1.0f, 1.0f, 1.0f, 1.0f);
                    flex.skinningPipeline.draw(modelView, projection, RenderSystem.getShader(), packedLight);
                } finally {
                    RenderSystem.setShaderColor(prevR, prevG, prevB, prevA);
                    RenderSystem.setShaderTexture(0, previousTexture0);
                }
            }
        } finally {
            overlay.teardownOverlayColor();
            lightmap.disable();
            RenderSystem.disableCull();
        }

        super.render(entity, entityYaw, partialTicks, matrixStack, vertexConsumers, packedLight);
    }

    private void renderSubMeshes(FlexbodyContainer flex, org.joml.Matrix4f modelView,
                                 org.joml.Matrix4f projection, int packedLight) {
        for (SubMeshRange range : flex.skinningPipeline.getSubMeshRanges()) {
            String namespace = flex.vehicleNamespace;
            MaterialDefinition material = MaterialLibrary.getMaterial(namespace, range.materialName);
            if (material == null) {
                warnOnceMissingMaterial(namespace, range.materialName);
            }
            MaterialRenderPlan plan = MaterialRenderPlanner.plan(material);

            TextureResource resource = plan.hasTexture()
                    ? MaterialLibrary.resolveTexture(plan.diffusePath())
                    : null;
            if (plan.hasTexture() && resource == null) {
                warnOnceUnresolvedTexture(namespace, range.materialName, plan.diffusePath());
            }

            int textureId = resolveDiffuseTexture(plan, resource != null,
                    () -> VehicleTextureUploader.INSTANCE.getOrUpload(resource, namespace),
                    VehicleTextureUploader.INSTANCE::getWhiteTexture);

            flex.skinningPipeline.drawRange(range, textureId, plan, modelView, projection,
                    RenderSystem.getShader(), packedLight);
        }
    }

    /**
     * Pure per-sub-mesh decision for which GL texture to bind as vanilla
     * {@code Sampler0}. A textured plan whose texture resolved is uploaded; every
     * other case (no texture, or a texture that failed to resolve) binds the
     * uploader's shared white 1x1 fallback. This pins the Iris-fix contract: the
     * renderer never binds a missing/unregistered texture (the removed
     * {@code vehicle_default} placeholder) for any sub-mesh.
     */
    static int resolveDiffuseTexture(MaterialRenderPlan plan, boolean resolved,
                                     IntSupplier upload, IntSupplier white) {
        if (plan.hasTexture() && resolved) {
            return upload.getAsInt();
        }
        return white.getAsInt();
    }

    private static void warnOnceMissingMaterial(String namespace, String materialName) {
        String key = namespace + ":" + materialName;
        if (WARNED_MISSING_MATERIALS.add(key)) {
            BeamCraft.LOGGER.warn(
                    "BeamCraft: no material found for DAE submesh '{}' (namespace '{}'); rendering colour-only",
                    materialName, namespace);
        }
    }

    private static void warnOnceUnresolvedTexture(String namespace, String materialName, String path) {
        String key = namespace + ":" + materialName + ":" + path;
        if (WARNED_UNRESOLVED_TEXTURES.add(key)) {
            BeamCraft.LOGGER.warn(
                    "BeamCraft: material '{}' (namespace '{}') references texture '{}' that cannot be resolved; rendering white",
                    materialName, namespace, path);
        }
    }
}
