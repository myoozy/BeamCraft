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
import net.minecraft.client.gl.ShaderProgram;
import net.minecraft.client.render.GameRenderer;
import net.minecraft.client.render.VertexConsumerProvider;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.Identifier;

import java.util.HashSet;
import java.util.Set;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of(
            "beamcraft",
            "textures/entity/vehicle_default.png"
    );

    /**
     * The currently registered BeamCraft opaque diffuse program. Registered via
     * {@code CoreShaderRegistrationCallback} from {@code BeamCraftClient}; Fabric
     * owns the program lifecycle (resource reload replaces and closes the
     * previous instance), so the renderer only reads the latest registered
     * program and never closes it. Null until the first (re)load finishes, in
     * which case the renderer falls back to the vanilla entity cutout whole-mesh
     * draw, so the vehicle still renders.
     */
    private static volatile ShaderProgram diffuseProgram;

    /** Called by the core-shader registration callback with each freshly loaded program. */
    public static void setDiffuseProgram(ShaderProgram program) {
        diffuseProgram = program;
    }

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
        RenderSystem.setShaderTexture(0, getTexture(entity));
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.enableCull();
        var lightmap = MinecraftClient.getInstance().gameRenderer.getLightmapTextureManager();
        lightmap.enable();
        try {
            org.joml.Matrix4f modelView = new org.joml.Matrix4f(RenderSystem.getModelViewMatrix());
            modelView.mul(matrixStack.peek().getPositionMatrix());
            org.joml.Matrix4f projection = RenderSystem.getProjectionMatrix();

            ShaderProgram shader = getRegisteredDiffuseProgram();
            if (shader != null && !flex.skinningPipeline.getSubMeshRanges().isEmpty()) {
                renderSubMeshes(flex, modelView, projection, shader, packedLight);
            } else {
                // Vanilla fallback: single whole-mesh draw, unchanged behaviour
                // (also used when no sub-mesh ranges were computed).
                flex.skinningPipeline.draw(modelView, projection, RenderSystem.getShader(), packedLight);
            }
        } finally {
            lightmap.disable();
            RenderSystem.disableCull();
        }

        super.render(entity, entityYaw, partialTicks, matrixStack, vertexConsumers, packedLight);
    }

    private void renderSubMeshes(FlexbodyContainer flex, org.joml.Matrix4f modelView,
                                 org.joml.Matrix4f projection, ShaderProgram shader, int packedLight) {
        for (SubMeshRange range : flex.skinningPipeline.getSubMeshRanges()) {
            String namespace = flex.vehicleNamespace;
            MaterialDefinition material = MaterialLibrary.getMaterial(namespace, range.materialName);
            if (material == null) {
                warnOnceMissingMaterial(namespace, range.materialName);
            }
            MaterialRenderPlan plan = MaterialRenderPlanner.plan(material);

            int textureId;
            if (plan.hasTexture()) {
                TextureResource resource = MaterialLibrary.resolveTexture(plan.diffusePath());
                if (resource == null) {
                    warnOnceUnresolvedTexture(namespace, range.materialName, plan.diffusePath());
                    textureId = VehicleTextureUploader.INSTANCE.getWhiteTexture();
                } else {
                    textureId = VehicleTextureUploader.INSTANCE.getOrUpload(resource, namespace);
                }
            } else {
                textureId = VehicleTextureUploader.INSTANCE.getWhiteTexture();
            }

            flex.skinningPipeline.drawRange(range, textureId, plan, modelView, projection, shader, packedLight);
        }
    }

    private static ShaderProgram getRegisteredDiffuseProgram() {
        return diffuseProgram;
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
