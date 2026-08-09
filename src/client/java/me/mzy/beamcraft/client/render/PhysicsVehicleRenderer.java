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
import org.joml.Matrix4f;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL14;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
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
    private static final Set<String> WARNED_MISSING_OPACITY = new HashSet<>();

    /** One opaque/cutout or translucent draw: the range plus its resolved plan. */
    private record RangeDraw(SubMeshRange range, MaterialRenderPlan plan) {
    }

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

        boolean previousBlend = GL11.glIsEnabled(GL11.GL_BLEND);
        boolean previousDepthTest = GL11.glIsEnabled(GL11.GL_DEPTH_TEST);
        boolean previousDepthWrite = GL11.glGetBoolean(GL11.GL_DEPTH_WRITEMASK);
        int previousDepthFunc = GL11.glGetInteger(GL11.GL_DEPTH_FUNC);
        int previousSrcRgb = GL11.glGetInteger(GL14.GL_BLEND_SRC_RGB);
        int previousDstRgb = GL11.glGetInteger(GL14.GL_BLEND_DST_RGB);
        int previousSrcAlpha = GL11.glGetInteger(GL14.GL_BLEND_SRC_ALPHA);
        int previousDstAlpha = GL11.glGetInteger(GL14.GL_BLEND_DST_ALPHA);
        boolean previousCull = GL11.glIsEnabled(GL11.GL_CULL_FACE);
        RenderSystem.setShader(GameRenderer::getRenderTypeEntityCutoutProgram);
        RenderSystem.disableBlend();
        RenderSystem.enableDepthTest();
        RenderSystem.depthFunc(515);
        RenderSystem.depthMask(true);
        RenderSystem.enableCull();
        var gameRenderer = MinecraftClient.getInstance().gameRenderer;
        var lightmap = gameRenderer.getLightmapTextureManager();
        var overlay = gameRenderer.getOverlayTexture();
        lightmap.enable();
        overlay.setupOverlayColor();
        try {
            Matrix4f modelView = new Matrix4f(RenderSystem.getModelViewMatrix());
            modelView.mul(matrixStack.peek().getPositionMatrix());
            Matrix4f projection = RenderSystem.getProjectionMatrix();

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
            RenderSystem.depthMask(previousDepthWrite);
            RenderSystem.depthFunc(previousDepthFunc);
            if (previousDepthTest) {
                RenderSystem.enableDepthTest();
            } else {
                RenderSystem.disableDepthTest();
            }
            RenderSystem.blendFuncSeparate(previousSrcRgb, previousDstRgb,
                    previousSrcAlpha, previousDstAlpha);
            if (previousBlend) {
                RenderSystem.enableBlend();
            } else {
                RenderSystem.disableBlend();
            }
            if (previousCull) {
                RenderSystem.enableCull();
            } else {
                RenderSystem.disableCull();
            }
        }

        super.render(entity, entityYaw, partialTicks, matrixStack, vertexConsumers, packedLight);
    }

    /**
     * Splits the per-material ranges into an opaque/cutout pass (drawn first, in
     * index order, with depth writes on) and a translucent pass (drawn last,
     * back-to-front, blended). Opaque and cutout share the vanilla
     * {@code entity_cutout} shader and depth-write state; only their textures
     * differ (cutout materials carry an alpha map composed into the diffuse).
     */
    private void renderSubMeshes(FlexbodyContainer flex, Matrix4f modelView,
                                 Matrix4f projection, int packedLight) {
        List<RangeDraw> opaqueCutout = new ArrayList<>();
        List<RangeDraw> translucent = new ArrayList<>();
        for (SubMeshRange range : flex.skinningPipeline.getSubMeshRanges()) {
            MaterialRenderPlan plan = resolvePlan(flex, range);
            if (plan.mode() == MaterialRenderPlan.RenderMode.TRANSLUCENT) {
                translucent.add(new RangeDraw(range, plan));
            } else {
                opaqueCutout.add(new RangeDraw(range, plan));
            }
        }

        for (RangeDraw draw : opaqueCutout) {
            drawRangeWithPlan(flex, draw, modelView, projection, packedLight);
        }
        if (!translucent.isEmpty()) {
            drawTranslucentRanges(flex, translucent, modelView, projection, packedLight);
        }
    }

    /**
     * Draws the translucent sub-meshes with the vanilla
     * {@code entity_translucent} shader: blending enabled (normal alpha, or
     * additive for an explicitly declared "Additive" {@code translucentBlendOp}),
     * depth test enabled, depth writes disabled. Back-face culling is applied
     * per range, not globally: paired window glass (raw DAE symbol in the
     * {@code *_glass} family, e.g. exterior {@code glass} vs interior
     * {@code glass_int}) keeps culling on so each shell draws exactly once from
     * its outward side and translucent layers never stack to white, while
     * single-shell lamp lenses and covers (e.g. {@code *_headlightglass},
     * {@code *_signalglass}) draw double-sided so they never vanish from behind
     * (see {@link #isDoubleSidedTranslucentGlass}). Ranges are sorted
     * back-to-front by their model-space centroid projected into view space.
     * Every piece of GL state is read back first and restored afterwards, even on
     * failure.
     */
    private void drawTranslucentRanges(FlexbodyContainer flex, List<RangeDraw> draws,
                                       Matrix4f modelView,
                                       Matrix4f projection, int packedLight) {
        List<SubMeshRange> ranges = new ArrayList<>(draws.size());
        Map<SubMeshRange, MaterialRenderPlan> planByRange = new IdentityHashMap<>();
        for (RangeDraw draw : draws) {
            ranges.add(draw.range);
            planByRange.put(draw.range, draw.plan);
        }
        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(ranges, modelView);

        boolean previousBlend = GL11.glIsEnabled(GL11.GL_BLEND);
        boolean previousDepthWrite = GL11.glGetBoolean(GL11.GL_DEPTH_WRITEMASK);
        int previousSrcRgb = GL11.glGetInteger(GL14.GL_BLEND_SRC_RGB);
        int previousDstRgb = GL11.glGetInteger(GL14.GL_BLEND_DST_RGB);
        int previousSrcAlpha = GL11.glGetInteger(GL14.GL_BLEND_SRC_ALPHA);
        int previousDstAlpha = GL11.glGetInteger(GL14.GL_BLEND_DST_ALPHA);
        boolean previousCull = GL11.glIsEnabled(GL11.GL_CULL_FACE);
        ShaderProgram previousShader = RenderSystem.getShader();

        RenderSystem.setShader(GameRenderer::getRenderTypeEntityTranslucentProgram);
        try {
            RenderSystem.enableBlend();
            RenderSystem.depthMask(false);
            RenderSystem.enableCull();
            for (SubMeshRange range : sorted) {
                MaterialRenderPlan plan = planByRange.get(range);
                if (plan == null) {
                    continue;
                }
                if (isDoubleSidedTranslucentGlass(range.materialName)) {
                    RenderSystem.disableCull();
                } else {
                    RenderSystem.enableCull();
                }
                int[] blend = blendFuncFor(plan.blendOp());
                if (blend[0] == GL11.GL_SRC_ALPHA && blend[1] == GL11.GL_ONE_MINUS_SRC_ALPHA) {
                    RenderSystem.defaultBlendFunc();
                } else {
                    RenderSystem.blendFunc(blend[0], blend[1]);
                }
                int textureId = resolveSampler0TextureId(flex, range, plan);
                flex.skinningPipeline.drawRange(range, textureId, plan, modelView, projection,
                        RenderSystem.getShader(), packedLight);
            }
        } finally {
            RenderSystem.depthMask(previousDepthWrite);
            RenderSystem.blendFuncSeparate(previousSrcRgb, previousDstRgb,
                    previousSrcAlpha, previousDstAlpha);
            if (previousBlend) {
                RenderSystem.enableBlend();
            } else {
                RenderSystem.disableBlend();
            }
            if (previousCull) {
                RenderSystem.enableCull();
            } else {
                RenderSystem.disableCull();
            }
            if (previousShader != null) {
                RenderSystem.setShader(() -> previousShader);
            }
        }
    }

    private void drawRangeWithPlan(FlexbodyContainer flex, RangeDraw draw, Matrix4f modelView,
                                   Matrix4f projection, int packedLight) {
        int textureId = resolveSampler0TextureId(flex, draw.range, draw.plan);
        flex.skinningPipeline.drawRange(draw.range, textureId, draw.plan, modelView, projection,
                RenderSystem.getShader(), packedLight);
    }

    /**
     * Resolves the {@code Sampler0} texture for one sub-mesh plan. A translucent
     * or cutout plan with a resolvable opacity map binds the composed
     * diffuse+opacity texture; a missing opacity map degrades deterministically
     * to the diffuse texture's own baked alpha (with a one-time warning) rather
     * than disappearing the sub-mesh. Everything else follows
     * {@link #resolveSampler0Texture}.
     */
    private int resolveSampler0TextureId(FlexbodyContainer flex, SubMeshRange range, MaterialRenderPlan plan) {
        String namespace = flex.vehicleNamespace;
        TextureResource diffuse = plan.hasTexture() ? MaterialLibrary.resolveTexture(plan.diffusePath()) : null;
        if (plan.hasTexture() && diffuse == null) {
            warnOnceUnresolvedTexture(namespace, range.materialName, plan.diffusePath());
        }
        TextureResource opacity = null;
        if (plan.hasOpacity()) {
            opacity = MaterialLibrary.resolveTexture(plan.opacityPath());
            if (opacity == null) {
                warnOnceMissingOpacity(namespace, range.materialName, plan.opacityPath());
            }
        }
        // Final aliases so the suppliers below can capture them (opacity is
        // conditionally assigned above).
        final TextureResource capturedDiffuse = diffuse;
        final TextureResource capturedOpacity = opacity;
        boolean composedAvailable = diffuse != null && opacity != null;
        return resolveSampler0Texture(
                plan,
                diffuse != null,
                composedAvailable,
                () -> VehicleTextureUploader.INSTANCE.getOrUploadComposed(capturedDiffuse, capturedOpacity, namespace),
                () -> VehicleTextureUploader.INSTANCE.getOrUpload(capturedDiffuse, namespace),
                VehicleTextureUploader.INSTANCE::getWhiteTexture);
    }

    /**
     * Pure per-sub-mesh decision for which GL texture to bind as vanilla
     * {@code Sampler0}. An opacity-carrying plan whose diffuse <em>and</em>
     * opacity both resolved binds the composed texture; otherwise the decision
     * degrades to {@link #resolveDiffuseTexture} (diffuse when it resolved,
     * white otherwise). This pins the Iris-fix contract: the renderer never
     * binds a missing/unregistered texture for any sub-mesh, and a missing
     * opacity map can never take a whole vehicle down.
     */
    static int resolveSampler0Texture(MaterialRenderPlan plan, boolean diffuseResolved, boolean composedAvailable,
                                      IntSupplier composedUpload, IntSupplier diffuseUpload, IntSupplier white) {
        if (plan.hasOpacity() && composedAvailable) {
            return composedUpload.getAsInt();
        }
        return resolveDiffuseTexture(plan, diffuseResolved, diffuseUpload, white);
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

    /**
     * Pure, unit-tested blend pair for a BeamNG {@code translucentBlendOp}.
     * Only "Additive" is handled specially (src = SRC_ALPHA, dst = ONE); every
     * other value — "None", null, anything unknown — falls back to normal alpha
     * blending (SRC_ALPHA, ONE_MINUS_SRC_ALPHA). No other BeamNG blend mode is
     * guessed.
     */
    static int[] blendFuncFor(String blendOp) {
        if (blendOp != null && blendOp.trim().equalsIgnoreCase("Additive")) {
            return new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE};
        }
        return new int[]{GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA};
    }

    /**
     * Whether a translucent sub-mesh must be drawn double-sided (back-face
     * culling disabled) rather than with back-face culling.
     *
     * <p>BeamNG window glass ships as paired shells — an exterior
     * {@code *_glass} plus an interior {@code *_glass_int} — each with
     * outward-facing normals, so culling back-faces makes every triangle draw
     * exactly once and never stacks translucent layers to white. Lamp lenses,
     * signal/taillight covers and other single-shell glass carry a compound raw
     * DAE symbol ({@code *_headlightglass}, {@code *_signalglass},
     * {@code *_taillightglass}, {@code *_foglightglass}, {@code *_reverselightglass},
     * {@code *_lowbeamglass}, …) whose triangles face outward only; culling their
     * back-faces makes the cover vanish from behind.
     *
     * <p>Classification keys on the <em>raw DAE material identity</em> (the
     * sub-mesh provenance), never on the aliased target definition: the Sunburst
     * lamp covers alias to {@code sunburst2_glass} (a paired window-glass family
     * member), so resolving them first would wrongly cull them.
     *
     * @param rawMaterialName the sub-mesh's raw DAE material name (Assimp
     *                        {@code AI_MATKEY_NAME}), never the resolved alias
     * @return true when the range must be double-sided
     */
    static boolean isDoubleSidedTranslucentGlass(String rawMaterialName) {
        if (rawMaterialName == null) {
            return false;
        }
        String n = rawMaterialName.toLowerCase(Locale.ROOT);
        return n.contains("glass") && !isWindowGlassFamily(n);
    }

    /**
     * True when {@code lowerName} is a paired window-glass family member:
     * {@code *_glass}, {@code *_glass_int}, {@code *_glass_dmg}, {@code *_glass_on}
     * or {@code *_glass_on_intense}. These have an opposite shell in the DAE and
     * draw correctly with back-face culling.
     */
    private static boolean isWindowGlassFamily(String lowerName) {
        return lowerName.endsWith("_glass")
                || lowerName.endsWith("_glass_int")
                || lowerName.endsWith("_glass_dmg")
                || lowerName.endsWith("_glass_on")
                || lowerName.endsWith("_glass_on_intense");
    }

    private MaterialRenderPlan resolvePlan(FlexbodyContainer flex, SubMeshRange range) {
        String namespace = flex.vehicleNamespace;
        MaterialDefinition material = MaterialLibrary.getMaterial(namespace, range.materialName);
        if (material == null) {
            warnOnceMissingMaterial(namespace, range.materialName);
        }
        return MaterialRenderPlanner.plan(material);
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

    private static void warnOnceMissingOpacity(String namespace, String materialName, String path) {
        String key = namespace + ":" + materialName + ":opacity:" + path;
        if (WARNED_MISSING_OPACITY.add(key)) {
            BeamCraft.LOGGER.warn(
                    "BeamCraft: material '{}' (namespace '{}') references opacity map '{}' that cannot be resolved; "
                            + "falling back to the diffuse texture's baked alpha",
                    materialName, namespace, path);
        }
    }
}
