package me.mzy.beamcraft.texture;

/**
 * Surface formats the local decoder can expand to RGBA8. The enum carries the
 * sizing information needed to compute a surface's byte size from its width and
 * height.
 *
 * <p>Block-compressed formats (BC1-BC7) are tiled in 4x4 pixel blocks, each
 * block compressed to {@code bytesPerBlock}. Uncompressed formats are tightly
 * packed rows of {@code width * bytesPerPixel}.
 *
 * <p>This enum is intentionally limited to what the decoder implements: diffuse
 * color from BC1/DXT1, BC2/DXT3, BC3/DXT5, BC7 and common uncompressed RGB(A),
 * plus the single-channel opacity formats BC4/ATI1, A8 and L8. Anything else
 * (BC5, BC6H, 10-bit packed, paletted, ...) surfaces as
 * {@link UnsupportedDdsFormatException}.
 */
enum DdsFormat {

    /** BC1 / DXT1. Color only; may embed a transparent-black index. */
    BC1(8, 4, 4),
    /** BC2 / DXT3. 4-bit alpha + BC1-style 4-color color block. */
    BC2(16, 4, 4),
    /** BC3 / DXT5. 8-bit interpolated alpha + BC1-style 4-color color block. */
    BC3(16, 4, 4),
    /** BC4 / ATI1. Single-channel (red) 8-bit interpolated values. Opacity. */
    BC4(8, 4, 4),
    /** BC7 / BPTC (two- or three-subset). Full RGBA. */
    BC7(16, 4, 4),

    /** 32-bit tightly packed RGBA or BGRA (component order resolved per header). */
    UNCOMPRESSED_RGBA8(4, 1, 1),
    /** 24-bit tightly packed RGB. */
    UNCOMPRESSED_RGB8(3, 1, 1),
    /** 8-bit single channel (L8 or A8); value replicated to RGB, alpha = 255. */
    UNCOMPRESSED_R8(1, 1, 1),
    /** 16-bit luminance+alpha (A8L8): R = luma, G = alpha. */
    UNCOMPRESSED_RG8(2, 1, 1);

    final int bytesPerBlock;
    final int blockWidth;
    final int blockHeight;

    DdsFormat(int bytesPerBlock, int blockWidth, int blockHeight) {
        this.bytesPerBlock = bytesPerBlock;
        this.blockWidth = blockWidth;
        this.blockHeight = blockHeight;
    }

    boolean isBlockCompressed() {
        return blockWidth > 1;
    }

    int bytesPerPixel() {
        return blockWidth == 1 ? bytesPerBlock : 0;
    }

    /**
     * Number of blocks along X for a given width (ceil division; blocks extend
     * past the edge for non-multiple-of-4 widths).
     */
    int blocksX(int width) {
        return (width + blockWidth - 1) / blockWidth;
    }

    int blocksY(int height) {
        return (height + blockHeight - 1) / blockHeight;
    }

    /**
     * Byte size of the base surface for the given dimensions.
     *
     * <p>Uncompressed surfaces use tightly packed rows ({@code width * bpp} per
     * row). Block-compressed surfaces use {@code blocksX * blocksY * bytesPerBlock}.
     * All arithmetic is done in {@code long} with {@code (long)} casts before
     * any addition/ceil so a hostile header (e.g. {@code width} near
     * {@code Integer.MAX_VALUE}) cannot overflow into a small or negative size
     * and slip past the truncation check.
     */
    long surfaceSize(int width, int height) {
        if (isBlockCompressed()) {
            long blocksX = ((long) width + blockWidth - 1) / blockWidth;
            long blocksY = ((long) height + blockHeight - 1) / blockHeight;
            return blocksX * blocksY * bytesPerBlock;
        }
        return (long) width * bytesPerPixel() * height;
    }
}
