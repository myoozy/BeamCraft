package me.mzy.beamcraft.texture;

import java.util.Arrays;
import java.util.Objects;

/**
 * Immutable, renderer-backend-neutral decoded RGBA8 image.
 *
 * <p>This is the currency of the texture decode/cache layer. It deliberately
 * depends on nothing but the Java standard library: no OpenGL, no Vulkan, no
 * Minecraft {@code NativeImage}, no LWJGL GL APIs. A later renderer backend
 * copies the pixels out with {@link #copyPixelData} and uploads them its own
 * way.
 *
 * <p><b>Layout contract</b> (documented once, relied on by every consumer):
 * <ul>
 *   <li><b>Row origin</b>: row 0 is the <em>top</em> row (the first row stored
 *       in the DDS surface). Pixel {@code (x, y)} lives at byte offset
 *       {@code (y * width + x) * 4}.</li>
 *   <li><b>Channel order</b>: the pixel array is packed
 *       {@code R, G, B, A} per pixel, 1 byte per channel, no padding, row
 *       major.</li>
 *   <li><b>Color space</b>: {@link #isSrgb()} reports whether the bytes are
 *       sRGB-encoded as stored (no conversion is ever performed here; the flag
 *       only tells a later uploader which texture format to create). Single
 *       channel/mask images (BC4, A8, L8) are always {@code srgb == false} and
 *       must not be color-space converted; their meaningful value is the red
 *       channel (see {@link TextureCompositor}).</li>
 * </ul>
 *
 * <p>The backing array is private and never exposed directly; external callers
 * get a defensive copy via {@link #copyPixelData()}. Instances are immutable
 * and safe to share across threads.
 */
public final class DecodedImage {

    private final int width;
    private final int height;
    private final boolean srgb;
    private final byte[] rgba;

    /**
     * Private: {@link #of} validates and copies; the package-private
     * {@link #ofOwned} wraps an array the caller just built (used by the
     * compositor, which constructs fresh output arrays).
     */
    private DecodedImage(int width, int height, byte[] rgba, boolean srgb) {
        this.width = width;
        this.height = height;
        this.rgba = Objects.requireNonNull(rgba, "rgba");
        this.srgb = srgb;
    }

    /**
     * Creates an image from a caller-owned array. The array is defensively
     * copied; the input array is not retained.
     *
     * @param width  pixel width, must be &gt; 0
     * @param height pixel height, must be &gt; 0
     * @param rgba   packed {@code R,G,B,A} pixels, must have exactly
     *               {@code width * height * 4} entries
     * @param srgb   whether the bytes are sRGB-encoded (see the class javadoc)
     * @throws IllegalArgumentException on non-positive dimensions or a length
     *                                  mismatch
     */
    public static DecodedImage of(int width, int height, byte[] rgba, boolean srgb) {
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("width and height must be positive: " + width + "x" + height);
        }
        long expected = (long) width * height * 4;
        if (rgba == null || rgba.length != expected) {
            throw new IllegalArgumentException("pixel array length " + (rgba == null ? "null" : rgba.length)
                    + " does not match " + width + "x" + height + " RGBA8 (expected " + expected + ")");
        }
        return new DecodedImage(width, height, rgba.clone(), srgb);
    }

    /**
     * Package-private: wraps a freshly built array without copying. Used only
     * for images constructed by this package (decoders, compositor) whose array
     * is already private to them.
     */
    static DecodedImage ofOwned(int width, int height, byte[] rgba, boolean srgb) {
        if (width <= 0 || height <= 0 || rgba == null || rgba.length != (long) width * height * 4) {
            throw new IllegalArgumentException("invalid image dimensions/length: " + width + "x" + height);
        }
        return new DecodedImage(width, height, rgba, srgb);
    }

    /** Pixel width. */
    public int width() {
        return width;
    }

    /** Pixel height. */
    public int height() {
        return height;
    }

    /**
     * Number of pixels ({@code width * height}); for diagnostics.
     */
    public long pixelCount() {
        return (long) width * height;
    }

    /**
     * True when the pixel bytes are sRGB-encoded as stored; false for linear or
     * for single-channel mask images. See the class javadoc for the contract.
     */
    public boolean isSrgb() {
        return srgb;
    }

    /**
     * Copies the full packed {@code R,G,B,A} pixel array (length
     * {@code width * height * 4}) into a freshly allocated array. Safe to call
     * any time; the returned array is a snapshot, not a view.
     */
    public byte[] copyPixelData() {
        return rgba.clone();
    }

    /**
     * Copies a slice of the pixel array into {@code dest}.
     *
     * @param srcOffset start index in the internal array
     * @param dest      destination array (must be non-null)
     * @param destPos   start index in {@code dest}
     * @param length    number of bytes to copy
     * @throws IndexOutOfBoundsException on any out-of-range access
     */
    public void copyPixelData(int srcOffset, byte[] dest, int destPos, int length) {
        System.arraycopy(rgba, srcOffset, dest, destPos, length);
    }

    /**
     * Reads a single pixel as a packed {@code 0xAARRGGBB} int. Convenience for
     * tests and diagnostics; bulk upload should use {@link #copyPixelData}.
     */
    public int getPixelRgba(int x, int y) {
        if (x < 0 || x >= width || y < 0 || y >= height) {
            throw new IndexOutOfBoundsException("(" + x + ", " + y + ") outside " + width + "x" + height);
        }
        int i = (y * width + x) * 4;
        return ((rgba[i + 3] & 0xFF) << 24)
                | ((rgba[i] & 0xFF) << 16)
                | ((rgba[i + 1] & 0xFF) << 8)
                | (rgba[i + 2] & 0xFF);
    }

    /**
     * Package-private: direct read access for same-package consumers (the
     * compositor). Never exposed publicly.
     */
    byte[] pixelData() {
        return rgba;
    }

    @Override
    public String toString() {
        return "DecodedImage[" + width + "x" + height + ", srgb=" + srgb + "]";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof DecodedImage that)) {
            return false;
        }
        return width == that.width && height == that.height && srgb == that.srgb
                && Arrays.equals(rgba, that.rgba);
    }

    @Override
    public int hashCode() {
        int result = 31 * width + height;
        result = 31 * result + (srgb ? 1 : 0);
        result = 31 * result + Arrays.hashCode(rgba);
        return result;
    }
}
