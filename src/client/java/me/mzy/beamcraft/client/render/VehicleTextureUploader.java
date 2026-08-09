package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.platform.GlStateManager;
import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.material.MaterialLibrary;
import me.mzy.beamcraft.client.material.TextureResource;
import me.mzy.beamcraft.texture.DecodedImage;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL30;
import org.lwjgl.system.MemoryUtil;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Render-thread-only OpenGL owner/cache/uploader for decoded vehicle textures.
 *
 * <p>This is the small renderer-backend class that keeps GL concerns out of the
 * decoder/material model (requirement: a future Vulkan backend replaces this
 * class, not {@link MaterialLibrary} or {@link DecodedImage}). Every public
 * mutating method asserts the render thread; all GL objects are created and
 * deleted here, on that thread.
 *
 * <p><b>Lifecycle</b>: textures are keyed by {@link TextureResource} (opaque,
 * value-semantics). {@link #getOrUpload} decodes on first use via
 * {@link MaterialLibrary#acquireDecodedTexture}, uploads, then releases the
 * decoded image in a {@code finally} so no decoded bytes stay pinned after
 * upload. Re-uploads of the same resource are avoided by the cache. When the
 * vehicle namespace that owns a texture is released,
 * {@link MaterialLibrary} notifies this class (via
 * {@link MaterialLibrary#addNamespaceReleaseListener}) and the vehicle-only GL
 * textures are deleted; shared/common textures (ownership {@code null}, see
 * {@link MaterialLibrary#resolveTextureOwnership}) survive until {@link #close}.
 *
 * <p><b>Upload contract</b>: {@link DecodedImage} stores row 0 as the top row.
 * OpenGL 3.2 uploads memory row 0 at texture coordinate {@code v = 0}. BeamCraft
 * mesh UVs are top-left origin ({@code v = 0} = top of the image; see
 * {@code DaeMeshLoader}'s documented {@code 1.0f - uv.y()} flip against Assimp's
 * bottom-left UV convention), matching vanilla Minecraft's NativeImage path. The
 * rows are therefore uploaded in memory order with no vertical flip; the
 * decision is pinned by the pure {@link #requiresUploadFlip} function so an
 * orientation change is explicit and unit-tested rather than an invisible guess.
 * sRGB images are uploaded as {@code GL_SRGB8_ALPHA8} so the hardware performs
 * the sRGB→linear decode on sampling; other images use {@code GL_RGBA8}. Both
 * formats are core in OpenGL 3.2.
 *
 * <p><b>Failure fallback</b>: a missing/failed/unsupported texture never takes
 * a vehicle down. {@link #getOrUpload} returns the single shared white 1×1
 * texture ({@link #getWhiteTexture}) and logs once per resource (rate-limited,
 * not per frame).
 */
public final class VehicleTextureUploader {

    private static final class Entry {
        final int textureId;
        final String namespace; // ownership namespace, or null for shared/common

        Entry(int textureId, String namespace) {
            this.textureId = textureId;
            this.namespace = namespace;
        }
    }

    private final Map<TextureResource, Entry> textures = new HashMap<>();
    private final Map<String, Set<TextureResource>> byNamespace = new HashMap<>();
    private final Set<String> warnedResources = new HashSet<>();
    private int whiteTextureId = -1;

    private VehicleTextureUploader() {
    }

    /** Shared instance for the current (single) GL context. */
    public static final VehicleTextureUploader INSTANCE = new VehicleTextureUploader();

    static {
        MaterialLibrary.addNamespaceReleaseListener(INSTANCE::releaseNamespaceFromAnyThread);
    }

    /**
     * Pure orientation decision, exposed for tests. A vertical flip is required
     * only when the decoded image is not top-row origin, or the mesh UVs do not
     * place {@code v = 0} at the top of the image. BeamCraft uses top row origin
     * ({@link DecodedImage}) and top-origin mesh UVs, so the answer is false.
     */
    static boolean requiresUploadFlip(boolean imageTopRowOrigin, boolean meshUvTopOrigin) {
        return !imageTopRowOrigin || !meshUvTopOrigin;
    }

    /**
     * Returns the GL texture for {@code resource}, uploading (and decoding) it
     * on first use. The decoded image is released immediately after upload.
     * Falls back to the shared white texture on any failure, logging once per
     * resource.
     *
     * @param resource  opaque handle from {@link MaterialLibrary#resolveTexture}
     * @param namespace vehicle namespace owning the request, for lifecycle
     * @return a valid GL texture id, never -1
     */
    public int getOrUpload(TextureResource resource, String namespace) {
        RenderSystem.assertOnRenderThread();
        if (resource == null) {
            return getWhiteTexture();
        }
        Entry entry = textures.get(resource);
        if (entry != null) {
            return entry.textureId;
        }
        try {
            DecodedImage image = MaterialLibrary.acquireDecodedTexture(resource, namespace);
            int textureId;
            try {
                textureId = upload(image);
            } finally {
                MaterialLibrary.releaseDecodedTexture(resource);
            }
            String ownership = MaterialLibrary.resolveTextureOwnership(resource, namespace);
            textures.put(resource, new Entry(textureId, ownership));
            if (ownership != null) {
                byNamespace.computeIfAbsent(ownership, k -> new HashSet<>()).add(resource);
            }
            return textureId;
        } catch (Exception e) {
            warnOnce(resource, e);
            return getWhiteTexture();
        }
    }

    /** The shared 1×1 white texture, created lazily on the render thread. */
    public int getWhiteTexture() {
        RenderSystem.assertOnRenderThread();
        if (whiteTextureId == -1) {
            int candidate = GL11.glGenTextures();
            try {
                bindTextureAndRestore(candidate, () -> {
                    ByteBuffer pixel = MemoryUtil.memAlloc(4);
                    try {
                        pixel.put((byte) 0xFF).put((byte) 0xFF).put((byte) 0xFF).put((byte) 0xFF);
                        pixel.flip();
                        uploadPixels(GL11.GL_RGBA8, 1, 1, pixel);
                        applyFilteringAndWrap();
                    } finally {
                        MemoryUtil.memFree(pixel);
                    }
                });
                whiteTextureId = candidate;
            } catch (RuntimeException e) {
                GlStateManager._deleteTexture(candidate);
                throw e;
            }
        }
        return whiteTextureId;
    }

    /** Number of GL textures currently cached (diagnostic). */
    public int cachedTextureCount() {
        return textures.size();
    }

    /**
     * Deletes every GL texture owned by {@code namespace} (vehicle-only).
     * Called from the namespace-release listener; safe to call for a namespace
     * with no cached textures. Shared/common entries are never removed here.
     */
    public void releaseNamespace(String namespace) {
        RenderSystem.assertOnRenderThread();
        if (namespace == null) {
            return;
        }
        Set<TextureResource> owned = byNamespace.remove(namespace);
        if (owned == null) {
            return;
        }
        for (TextureResource resource : owned) {
            Entry entry = textures.remove(resource);
            if (entry != null) {
                GlStateManager._deleteTexture(entry.textureId);
            }
        }
    }

    /** Deletes all cached GL textures, including the white fallback. Shutdown only. */
    public void close() {
        RenderSystem.assertOnRenderThread();
        for (Entry entry : textures.values()) {
            GlStateManager._deleteTexture(entry.textureId);
        }
        textures.clear();
        byNamespace.clear();
        if (whiteTextureId != -1) {
            GlStateManager._deleteTexture(whiteTextureId);
            whiteTextureId = -1;
        }
        warnedResources.clear();
    }

    /** Idempotent shutdown entry point; runs on the render thread when possible. */
    public void closeFromAnyThread() {
        if (RenderSystem.isOnRenderThread()) {
            close();
        } else {
            RenderSystem.recordRenderCall(this::close);
        }
    }

    private void releaseNamespaceFromAnyThread(String namespace) {
        if (RenderSystem.isOnRenderThread()) {
            releaseNamespace(namespace);
        } else {
            RenderSystem.recordRenderCall(() -> releaseNamespace(namespace));
        }
    }

    private int upload(DecodedImage image) {
        int textureId = GL11.glGenTextures();
        int internalFormat = image.isSrgb() ? GL30.GL_SRGB8_ALPHA8 : GL11.GL_RGBA8;
        try {
            bindTextureAndRestore(textureId, () -> {
                // The upload buffer is sized from the image's own pixel array so
                // no width*height*4 arithmetic (with its unchecked overflow
                // risk) decides the allocation.
                byte[] data = image.copyPixelData();
                ByteBuffer pixels = MemoryUtil.memAlloc(data.length);
                try {
                    pixels.put(data);
                    pixels.flip();
                    uploadPixels(internalFormat, image.width(), image.height(), pixels);
                } finally {
                    MemoryUtil.memFree(pixels);
                }
                applyFilteringAndWrap();
            });
            return textureId;
        } catch (RuntimeException e) {
            GlStateManager._deleteTexture(textureId);
            throw e;
        }
    }

    /**
     * Runs {@code work} with {@code textureId} bound on the caller's current
     * active texture unit, restoring the previous {@code GL_TEXTURE_2D} binding
     * and active unit through the tracked wrappers afterwards (also on failure).
     * No assumption is made about which unit is active: the binding read and the
     * final rebind both use the active unit, so a caller's prior texture state
     * is never clobbered.
     */
    private void bindTextureAndRestore(int textureId, Runnable work) {
        int previousActiveTexture = GlStateManager._getActiveTexture();
        int previousBinding = GL11.glGetInteger(GL11.GL_TEXTURE_BINDING_2D);
        try {
            RenderSystem.bindTexture(textureId);
            work.run();
        } finally {
            RenderSystem.bindTexture(previousBinding);
            RenderSystem.activeTexture(previousActiveTexture);
        }
    }

    /**
     * Uploads packed RGBA8 rows into the (already bound) texture. Uses the
     * tracked {@link GlStateManager} wrappers (two-step: allocate storage with
     * no data, then upload rows) so {@code GlStateManager}'s per-unit
     * GL_TEXTURE_2D binding tracking stays consistent with the real GL state —
     * raw {@code glBindTexture} would leave a stale tracked binding behind and
     * later make Minecraft skip a needed rebind.
     */
    private void uploadPixels(int internalFormat, int width, int height, ByteBuffer pixels) {
        GlStateManager._texImage2D(GL11.GL_TEXTURE_2D, 0, internalFormat, width, height, 0,
                GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, (IntBuffer) null);
        GlStateManager._texSubImage2D(GL11.GL_TEXTURE_2D, 0, 0, 0, width, height,
                GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, MemoryUtil.memAddress(pixels));
    }

    /**
     * Sets predictable filtering/wrapping for opaque diffuse sampling. The
     * texture is already bound by the caller; the previous unit binding is
     * restored there, so this method never binds or unbinds itself.
     */
    private void applyFilteringAndWrap() {
        GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
        GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_LINEAR);
        GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_S, GL11.GL_REPEAT);
        GlStateManager._texParameter(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_WRAP_T, GL11.GL_REPEAT);
    }

    private void warnOnce(TextureResource resource, Exception e) {
        if (warnedResources.add(resource.describe())) {
            BeamCraft.LOGGER.warn(
                    "BeamCraft: cannot upload texture {} ({}); rendering white fallback",
                    resource.describe(), e.getMessage());
        }
    }
}
