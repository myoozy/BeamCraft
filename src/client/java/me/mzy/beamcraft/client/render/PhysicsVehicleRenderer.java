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

    /**
     * One opaque/cutout or translucent draw: the range, its resolved plan, and
     * the resolved {@link MaterialDefinition} it was planned from. The material
     * is retained because the translucent culling decision consults resolved
     * material provenance (see {@link #isDoubleSidedTranslucentGlass}).
     */
    private record RangeDraw(SubMeshRange range, MaterialRenderPlan plan, MaterialDefinition material) {
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
            MaterialDefinition material = resolveMaterial(flex, range);
            MaterialRenderPlan plan = MaterialRenderPlanner.plan(material);
            if (plan.mode() == MaterialRenderPlan.RenderMode.TRANSLUCENT) {
                translucent.add(new RangeDraw(range, plan, material));
            } else {
                opaqueCutout.add(new RangeDraw(range, plan, material));
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
     * per range, not globally: paired window glass keeps culling on so each shell
     * draws exactly once from its outward side and translucent layers never stack
     * to white, while single-shell glass, windshields and lamp lenses draw
     * double-sided so they never vanish from behind. The per-range decision is
     * data-driven on the vehicle's raw DAE mesh provenance plus the resolved
     * material: a range must actually be a glass/lens/lamp-cover material
     * (raw name or resolved material mentioning {@code glass}/{@code windshield}/
     * {@code lens}) <em>and</em> have no {@code *_int} opposite shell in the same
     * vehicle to draw double-sided; every other translucent range (decals,
     * emissive/additive sheets, lamp housings, screens) keeps default back-face
     * culling (see {@link #isDoubleSidedTranslucentGlass}). Ranges are sorted
     * back-to-front by their model-space centroid projected into view space.
     * The double-sided <em>lamp lens/cover</em> ranges additionally switch the
     * depth test to LEQUAL (see {@link #isDoubleSidedLampLens}): their housing
     * reflector is now opaque and wrote depth in the opaque pass (see
     * {@code MaterialRenderPlanner#isEffectivelyOpaqueTranslucent}), and the
     * cover is exactly coplanar with it, so the plain GL_LESS test would reject
     * the cover everywhere its depth equals the housing's. LEQUAL lets the cover
     * pass at equality and draw over the housing, restoring the lens regardless
     * of back-to-front sort order. Every piece of GL state is read back first
     * and restored afterwards, even on failure.
     */
    private void drawTranslucentRanges(FlexbodyContainer flex, List<RangeDraw> draws,
                                       Matrix4f modelView,
                                       Matrix4f projection, int packedLight) {
        List<SubMeshRange> ranges = new ArrayList<>(draws.size());
        Map<SubMeshRange, MaterialRenderPlan> planByRange = new IdentityHashMap<>();
        Map<SubMeshRange, MaterialDefinition> materialByRange = new IdentityHashMap<>();
        for (RangeDraw draw : draws) {
            ranges.add(draw.range);
            planByRange.put(draw.range, draw.plan);
            materialByRange.put(draw.range, draw.material);
        }
        List<SubMeshRange> sorted = ComputeSkinningPipeline.sortTranslucentBackToFront(ranges, modelView);
        Set<String> meshMaterialNames = collectMeshMaterialNames(flex);

        boolean previousBlend = GL11.glIsEnabled(GL11.GL_BLEND);
        boolean previousDepthWrite = GL11.glGetBoolean(GL11.GL_DEPTH_WRITEMASK);
        int previousDepthFunc = GL11.glGetInteger(GL11.GL_DEPTH_FUNC);
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
                MaterialDefinition material = materialByRange.get(range);
                boolean doubleSided = isDoubleSidedTranslucentGlass(
                        range.materialName, meshMaterialNames, material);
                if (doubleSided) {
                    RenderSystem.disableCull();
                } else {
                    RenderSystem.enableCull();
                }
                // A double-sided lamp lens is exactly coplanar with its (now
                // opaque) housing reflector, which wrote depth in the opaque
                // pass. GL_LESS would reject the cover where depths are equal,
                // so the cover draws with LEQUAL and passes at equal depth.
                // Scoped to the double-sided lamp-lens set only — window glass,
                // windshields, housings and every other translucent range keep
                // GL_LESS (see isDoubleSidedLampLens).
                RenderSystem.depthFunc(isDoubleSidedLampLens(range.materialName, meshMaterialNames, material)
                        ? GL11.GL_LEQUAL : GL11.GL_LESS);
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
            RenderSystem.depthFunc(previousDepthFunc);
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
     * Lowercased set of every raw DAE material name present in the vehicle's
     * sub-mesh ranges. This is the culling-decision provenance: the decision must
     * key on what meshes actually exist, not on a name guess about whether a
     * paired opposite shell exists. Null names are skipped (they can never match
     * a pairing).
     */
    private static Set<String> collectMeshMaterialNames(FlexbodyContainer flex) {
        Set<String> names = new HashSet<>();
        for (SubMeshRange range : flex.skinningPipeline.getSubMeshRanges()) {
            if (range.materialName != null) {
                names.add(range.materialName.toLowerCase(Locale.ROOT));
            }
        }
        return names;
    }

    /**
     * Whether a translucent sub-mesh must be drawn double-sided (back-face
     * culling disabled) rather than with back-face culling.
     *
     * <p>Two independent signals gate the decision:
     * <ol>
     *   <li><em>Semantic guard</em> — the range must be an actual glass, lens,
     *       lamp-cover or windshield material. The raw DAE name <em>or</em> the
     *       resolved material definition's {@code mapTo}/{@code name} must
     *       mention {@code glass}, {@code windshield} or {@code lens}. This is
     *       what keeps unrelated translucent ranges — decals, emissive/additive
     *       sheets, lamp housings, screens, warning indicators — out of the
     *       double-sided path: they carry none of those markers and keep their
     *       default back-face culling.</li>
     *   <li><em>Provenance pairing</em> — the range must not be one shell of a
     *       paired window-glass pair: a {@code *_int} opposite shell present in
     *       the same vehicle's raw DAE mesh set ({@code meshMaterialNames}, the
     *       lowercased raw DAE material names of every sub-mesh in the vehicle).
     *       Paired shells each have outward-facing normals, so culling back-faces
     *       makes every triangle draw exactly once and never stacks translucent
     *       layers to white.</li>
     * </ol>
     *
     * <p>The semantic guard consults the <em>resolved</em> material (the alias
     * target) as well as the raw name: a lamp lens whose raw DAE name does not
     * spell {@code glass} still resolves via the JBeam {@code glowMap} alias to a
     * glass-named material (e.g. {@code pickup_lowbeamglass} → {@code pickup_lightglass}),
     * and a single-shell windshield raw name carries {@code windshield}
     * (e.g. {@code sunburst2_windshield}). The pairing check, by contrast, keys
     * strictly on the <em>raw DAE identity</em>: the Sunburst lamp covers alias
     * to {@code sunburst2_glass} (a paired window-glass shell), so using the
     * resolved name for pairing would wrongly cull them — the raw name is the
     * only thing that knows whether a {@code *_int} sibling actually exists.
     *
     * @param rawMaterialName   the sub-mesh's raw DAE material name (Assimp
     *                          {@code AI_MATKEY_NAME}), never the resolved alias
     * @param meshMaterialNames lowercased raw DAE material names of every
     *                          sub-mesh in the same vehicle (never the resolved
     *                          alias targets)
     * @param resolvedMaterial  the material {@code MaterialLibrary} resolves for
     *                          {@code rawMaterialName} (alias target included),
     *                          or null when nothing resolved
     * @return true when the range must be double-sided
     */
    static boolean isDoubleSidedTranslucentGlass(String rawMaterialName, Set<String> meshMaterialNames,
                                                 MaterialDefinition resolvedMaterial) {
        if (rawMaterialName == null) {
            return false;
        }
        if (!isGlassLensMaterial(rawMaterialName, resolvedMaterial)) {
            return false;
        }
        return !hasPairedOppositeShell(rawMaterialName.toLowerCase(Locale.ROOT), meshMaterialNames);
    }

    /**
     * Semantic guard: is this range actually glass/lens/windshield? True when the
     * raw DAE name mentions {@code glass}, {@code windshield} or {@code lens},
     * or — for raw names that do not — when the resolved material's
     * {@code mapTo}/{@code name} does (the glowMap alias provenance). A null
     * material is never proven glass, so unresolved or opaque ranges stay out of
     * the double-sided path.
     */
    private static boolean isGlassLensMaterial(String rawMaterialName, MaterialDefinition resolvedMaterial) {
        if (containsGlassLike(rawMaterialName)) {
            return true;
        }
        if (resolvedMaterial != null) {
            return containsGlassLike(resolvedMaterial.mapTo) || containsGlassLike(resolvedMaterial.name);
        }
        return false;
    }

    /** True when a non-empty string mentions glass, windshield or lens, case-insensitive. */
    private static boolean containsGlassLike(String s) {
        if (s == null || s.isEmpty()) {
            return false;
        }
        String n = s.toLowerCase(Locale.ROOT);
        return n.contains("glass") || n.contains("windshield") || n.contains("lens");
    }

    /**
     * Whether a translucent sub-mesh is a double-sided single-shell <em>lamp
     * lens/cover</em> — the set that draws with an equal-depth (LEQUAL) depth
     * test. A range qualifies when it is proven double-sided glass (see
     * {@link #isDoubleSidedTranslucentGlass}) <em>and</em> its raw DAE name or
     * resolved material identifies a light cover ({@link #isLampLensCover}).
     * This is exactly the geometry that is coplanar with the opaque lamp housing
     * reflector: the Covet/BX covers (e.g. {@code covet_headlightglass},
     * {@code bx_taillightglass}) share exact vertex positions with the housing
     * meshes, so GL_LESS depth rejection would make them disappear where the
     * housing's written depth is equal.
     *
     * @param rawMaterialName   the sub-mesh's raw DAE material name (never the
     *                          resolved alias)
     * @param meshMaterialNames lowercased raw DAE material names of every
     *                          sub-mesh in the same vehicle (the pairing
     *                          provenance)
     * @param resolvedMaterial  the resolved material, or null
     * @return true when the range is a double-sided lamp lens that must pass
     *         depth at equality
     */
    static boolean isDoubleSidedLampLens(String rawMaterialName, Set<String> meshMaterialNames,
                                         MaterialDefinition resolvedMaterial) {
        return isDoubleSidedTranslucentGlass(rawMaterialName, meshMaterialNames, resolvedMaterial)
                && isLampLensCover(rawMaterialName, resolvedMaterial);
    }

    /**
     * Semantic guard: is this glass/lens/windshield range a <em>lamp
     * cover</em>? True when the raw DAE name or the resolved material's
     * {@code mapTo}/{@code name} mentions one of the BeamNG lamp-cover markers
     * ({@code headlight}, {@code taillight}, {@code signal}, {@code brakelight},
     * {@code chmsl}, {@code foglight}, {@code reverselight}, {@code marker},
     * {@code lowbeam}/{@code highbeam}, {@code parkinglight}, {@code sidemarker},
     * {@code runninglight}, {@code lightglass}, {@code sealedbeam},
     * {@code halogen}, {@code lamp}, {@code lens}, {@code sign}, {@code gauge}).
     * This is what keeps window glass, windshields, housings and decals out of
     * the equal-depth path: {@code glass_invisible}, {@code van_glass} and
     * {@code sunburst2_windshield} carry none of those markers and keep the plain
     * GL_LESS test. It is consulted only for ranges already proven double-sided
     * glass (see {@link #isDoubleSidedLampLens}), so a housing like
     * {@code covet_lights} — which resolves to a light-named material but is not
     * glass — still keeps default depth.
     */
    static boolean isLampLensCover(String rawMaterialName, MaterialDefinition resolvedMaterial) {
        if (containsLampLensMarker(rawMaterialName)) {
            return true;
        }
        if (resolvedMaterial != null) {
            return containsLampLensMarker(resolvedMaterial.mapTo)
                    || containsLampLensMarker(resolvedMaterial.name);
        }
        return false;
    }

    /** True when a non-empty string mentions a BeamNG lamp-cover marker, case-insensitive. */
    private static boolean containsLampLensMarker(String s) {
        if (s == null || s.isEmpty()) {
            return false;
        }
        String n = s.toLowerCase(Locale.ROOT);
        for (String marker : LAMP_LENS_MARKERS) {
            if (n.contains(marker)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Lamp-cover markers seen in the actual raw DAE material names and resolved
     * materials of the bundled vehicles (covet/bx/sunburst2/pickup/etki/citybus/
     * common). Ordered longest-first so compound markers match before their
     * shorter substrings (matters only for readability).
     */
    private static final String[] LAMP_LENS_MARKERS = {
            "headlight", "taillight", "parkinglight", "reverselight", "runninglight",
            "brakelight", "sidemarker", "foglight", "signalglass", "lightglass",
            "lowbeam", "highbeam", "sealedbeam", "halogen", "chmsl", "marker",
            "signal", "lamp", "lens", "sign", "gauge", "cover"
    };

    /**
     * True when the vehicle's mesh set contains the paired opposite shell for
     * {@code lowerName}: for a {@code *_int} interior shell the partner is the
     * name stripped of {@code _int}; for every other name the partner is
     * {@code lowerName + "_int"}. A null/empty mesh set (never passed by the
     * renderer) yields false so the caller falls back to double-sided.
     */
    private static boolean hasPairedOppositeShell(String lowerName, Set<String> meshMaterialNames) {
        if (meshMaterialNames == null) {
            return false;
        }
        if (lowerName.endsWith(INTERIOR_SHELL_SUFFIX)) {
            return meshMaterialNames.contains(
                    lowerName.substring(0, lowerName.length() - INTERIOR_SHELL_SUFFIX.length()));
        }
        return meshMaterialNames.contains(lowerName + INTERIOR_SHELL_SUFFIX);
    }

    /** Suffix marking the interior shell of a paired window-glass pair. */
    private static final String INTERIOR_SHELL_SUFFIX = "_int";

    /**
     * Resolves the material definition for a sub-mesh's raw DAE name (through
     * the namespace's static glowMap aliases, then the common library). Returns
     * null when nothing resolves, after a one-time warning.
     */
    private MaterialDefinition resolveMaterial(FlexbodyContainer flex, SubMeshRange range) {
        String namespace = flex.vehicleNamespace;
        MaterialDefinition material = MaterialLibrary.getMaterial(namespace, range.materialName);
        if (material == null) {
            warnOnceMissingMaterial(namespace, range.materialName);
        }
        return material;
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
