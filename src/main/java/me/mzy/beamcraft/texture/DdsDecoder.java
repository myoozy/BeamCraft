package me.mzy.beamcraft.texture;

/**
 * Pure-Java DDS (DirectDraw Surface) decoder that expands a surface to RGBA8
 * {@link DecodedImage}. It has no dependencies on OpenGL, Vulkan, Minecraft
 * {@code NativeImage}, RenderSystem, or any LWJGL GL API, so it runs unchanged
 * on the current Java 21 build and on a future Java 25/Vulkan backend.
 *
 * <p><b>Supported formats</b> (this is the whole list; anything else fails with
 * {@link UnsupportedDdsFormatException}):
 * <ul>
 *   <li>Diffuse: BC1/DXT1, BC2/DXT3, BC3/DXT5, BC7 (both UNORM and _SRGB DX10
 *       tags) and uncompressed 32-bit RGBA/BGRA and 24-bit RGB.</li>
 *   <li>Opacity: BC4/ATI1 (single channel) plus uncompressed 8-bit single
 *       channel (A8, L8, R8) and A8L8. Single-channel values are replicated to
 *       RGB with alpha 255; consumers read the red channel as the meaningful
 *       value (see {@link TextureCompositor}).</li>
 * </ul>
 * DXT2/DXT4 (the premultiplied-alpha siblings of DXT3/DXT5) are <em>not</em>
 * decoded: treating them as straight alpha would render already-multiplied
 * channels too dark, and correctly unpremultiplying is out of scope, so they
 * are rejected with {@link UnsupportedDdsFormatException}.
 *
 * <p><b>Behaviour on bad input</b>: structurally invalid or truncated files
 * throw {@link DdsDecodeException}; well-formed files in an unimplemented
 * format throw {@link UnsupportedDdsFormatException}. Pixels are never
 * fabricated, so a wrong format can never silently produce corrupt colours.
 * Only the base (largest) mip level is decoded; trailing mip/face data, when
 * present, is ignored. Dimensions that would need more than
 * {@code Integer.MAX_VALUE / 4} pixels (the cap for a single Java {@code byte[]})
 * are rejected up front, so a malicious header can never trigger an integer
 * overflow or a huge allocation -- only {@link DdsDecodeException}.
 *
 * <p><b>Color-space flag</b>: {@link DecodedImage#isSrgb()} is true for colour
 * formats declared {@code _SRGB} in a DX10 header; false for single-channel
 * masks and for DX10 {@code _UNORM} formats. Legacy (pre-DX10) colour formats
 * default to {@code srgb = true} because BeamNG diffuse assets are authored
 * sRGB and legacy DDS predates SRGB tags (mirroring how Minecraft treats
 * diffuse textures as sRGB).
 *
 * <p>The block algorithms follow the D3D11/Khronos BPTC functional spec,
 * cross-checked against the bcdec reference implementation (S. Kudlai,
 * MIT/Unlicense) and Microsoft DirectXTex. The BC7 endpoint interpolation and
 * single-channel (BC4/BC3-alpha) interpolation use the spec's round-to-nearest
 * fixed-point rules.
 */
public final class DdsDecoder {

    // ------------------------------------------------------------------
    // DDS header constants
    // ------------------------------------------------------------------

    private static final int DDS_MAGIC = 0x20534444; // "DDS "

    private static final int DDSD_CAPS = 0x1;
    private static final int DDSD_HEIGHT = 0x2;
    private static final int DDSD_WIDTH = 0x4;
    private static final int DDSD_PITCH = 0x8;
    private static final int DDSD_PIXELFORMAT = 0x1000;

    private static final int DDPF_ALPHAPIXELS = 0x1;
    private static final int DDPF_ALPHA = 0x2;
    private static final int DDPF_FOURCC = 0x4;
    private static final int DDPF_PAL8 = 0x20;
    private static final int DDPF_RGB = 0x40;
    private static final int DDPF_LUMINANCE = 0x20000;

    private static final int DDSCAPS2_CUBEMAP = 0x200;
    private static final int DDSCAPS2_VOLUME = 0x200000;
    private static final int D3D10_RESOURCE_MISC_TEXTURECUBE = 0x4;
    /** D3D10_RESOURCE_DIMENSION_TEXTURE2D (the only dimension this decoder accepts). */
    private static final int D3D10_RESOURCE_DIMENSION_TEXTURE2D = 3;

    private static final int FOURCC_DXT1 = make4cc("DXT1");
    private static final int FOURCC_DXT2 = make4cc("DXT2");
    private static final int FOURCC_DXT3 = make4cc("DXT3");
    private static final int FOURCC_DXT4 = make4cc("DXT4");
    private static final int FOURCC_DXT5 = make4cc("DXT5");
    private static final int FOURCC_ATI1 = make4cc("ATI1");
    private static final int FOURCC_BC4U = make4cc("BC4U");
    private static final int FOURCC_DX10 = make4cc("DX10");

    // DXGI_FORMAT_* values (verified against the io.github.ititus:dds source).
    private static final int DXGI_R8G8B8A8_UNORM = 28;
    private static final int DXGI_R8G8B8A8_UNORM_SRGB = 29;
    private static final int DXGI_R8G8_UNORM = 49;
    private static final int DXGI_R8_UNORM = 61;
    private static final int DXGI_A8_UNORM = 65;
    private static final int DXGI_BC1_UNORM = 71;
    private static final int DXGI_BC1_UNORM_SRGB = 72;
    private static final int DXGI_BC2_UNORM = 74;
    private static final int DXGI_BC2_UNORM_SRGB = 75;
    private static final int DXGI_BC3_UNORM = 77;
    private static final int DXGI_BC3_UNORM_SRGB = 78;
    private static final int DXGI_BC4_UNORM = 80;
    private static final int DXGI_B8G8R8A8_UNORM = 87;
    private static final int DXGI_B8G8R8X8_UNORM = 88;
    private static final int DXGI_B8G8R8A8_UNORM_SRGB = 91;
    private static final int DXGI_B8G8R8X8_UNORM_SRGB = 93;
    private static final int DXGI_BC7_UNORM = 98;
    private static final int DXGI_BC7_UNORM_SRGB = 99;

    /** Byte layout of an uncompressed pixel inside the tightly packed rows. */
    private enum UncompressedLayout {
        /** Memory bytes are already R, G, B, A. */
        RGBA,
        /** Memory bytes are B, G, R, A (the classic D3D A8R8G8B8 layout). */
        BGRA,
        /** 24-bit: memory bytes are B, G, R. */
        RGB_24,
        /** 24-bit: memory bytes are R, G, B. */
        BGR_24
    }

    private static final class ResolvedFormat {
        final DdsFormat format;
        final UncompressedLayout layout;
        final boolean srgb;
        /**
         * True for uncompressed colour surfaces that carry no real alpha
         * channel (X8 variants, 24-bit RGB): the output alpha is forced to 255
         * instead of reading the padding byte.
         */
        final boolean forceOpaque;

        ResolvedFormat(DdsFormat format, UncompressedLayout layout, boolean srgb, boolean forceOpaque) {
            this.format = format;
            this.layout = layout;
            this.srgb = srgb;
            this.forceOpaque = forceOpaque;
        }
    }

    private DdsDecoder() {
    }

    // ------------------------------------------------------------------
    // Public entry point
    // ------------------------------------------------------------------

    /**
     * Decodes the first surface of a DDS file to an RGBA8 image.
     *
     * @param data the full DDS file bytes (magic + header + surface data)
     * @return an immutable decoded image (never null)
     * @throws UnsupportedDdsFormatException if the file is well formed but its
     *                                       surface format is not implemented
     * @throws DdsDecodeException            if the file is invalid or truncated
     */
    public static DecodedImage decode(byte[] data) throws UnsupportedDdsFormatException, DdsDecodeException {
        if (data == null) {
            throw new DdsDecodeException("null DDS byte array");
        }
        if (data.length < 128) {
            throw new DdsDecodeException("DDS file truncated: " + data.length + " bytes, need >= 128");
        }
        if (readIntLE(data, 0) != DDS_MAGIC) {
            throw new DdsDecodeException("invalid DDS magic (not a DDS file?)");
        }
        if (readIntLE(data, 4) != 124) {
            throw new DdsDecodeException("invalid DDS header size " + readIntLE(data, 4) + " (must be 124)");
        }

        int flags = readIntLE(data, 8);
        int height = readIntLE(data, 12);
        int width = readIntLE(data, 16);
        int pitchOrLinearSize = readIntLE(data, 20);
        int depth = readIntLE(data, 24);
        int caps2 = readIntLE(data, 112);

        if ((flags & DDSD_WIDTH) == 0 || (flags & DDSD_HEIGHT) == 0 || (flags & DDSD_PIXELFORMAT) == 0) {
            throw new DdsDecodeException("DDS header missing required WIDTH/HEIGHT/PIXELFORMAT flags");
        }
        if (width <= 0 || height <= 0) {
            throw new DdsDecodeException("invalid DDS dimensions " + width + "x" + height);
        }
        // Bound the pixel count up front (long arithmetic) so every later size
        // computation is overflow-free and the RGBA allocation can never exceed
        // Integer.MAX_VALUE. A malicious header therefore fails here, not with a
        // NegativeArraySizeException or OOME.
        long pixels = (long) width * height;
        if (pixels > Integer.MAX_VALUE / 4) {
            throw new DdsDecodeException("decoded surface too large for a Java byte[]: " + width + "x" + height
                    + " (" + pixels + " pixels)");
        }
        if (depth > 1 || (caps2 & (DDSCAPS2_CUBEMAP | DDSCAPS2_VOLUME)) != 0) {
            throw new UnsupportedDdsFormatException(
                    "volume/cubemap textures are not supported (" + width + "x" + height + "x" + depth + ")");
        }

        int pfFlags = readIntLE(data, 80);
        int fourCC = readIntLE(data, 84);
        if (readIntLE(data, 76) != 32) {
            throw new DdsDecodeException("invalid DDS pixel format size " + readIntLE(data, 76) + " (must be 32)");
        }

        ResolvedFormat resolved;
        int surfaceStart;
        if (fourCC == FOURCC_DX10) {
            if (data.length < 148) {
                throw new DdsDecodeException("DDS file truncated: missing DX10 header");
            }
            int dxgiFormat = readIntLE(data, 128);
            int resourceDimension = readIntLE(data, 132);
            int miscFlag = readIntLE(data, 136);
            int arraySize = readIntLE(data, 140);
            if (arraySize != 1) {
                throw new UnsupportedDdsFormatException(
                        "DDS texture arrays are not supported (arraySize=" + arraySize + ", only 1 allowed)");
            }
            if (resourceDimension != D3D10_RESOURCE_DIMENSION_TEXTURE2D) {
                throw new UnsupportedDdsFormatException("only D3D10 2D textures are supported (resourceDimension="
                        + resourceDimension + ", expected TEXTURE2D=" + D3D10_RESOURCE_DIMENSION_TEXTURE2D + ")");
            }
            if ((miscFlag & D3D10_RESOURCE_MISC_TEXTURECUBE) != 0) {
                throw new UnsupportedDdsFormatException("DDS cubemap textures are not supported");
            }
            resolved = resolveDx10(dxgiFormat);
            surfaceStart = 148;
        } else {
            resolved = resolveLegacy(pfFlags, fourCC,
                    readIntLE(data, 88), readIntLE(data, 92), readIntLE(data, 96), readIntLE(data, 100), readIntLE(data, 104));
            surfaceStart = 128;
        }

        DdsFormat format = resolved.format;
        long expected = format.surfaceSize(width, height);
        if ((long) surfaceStart + expected > data.length) {
            throw new DdsDecodeException("DDS surface truncated: need " + expected
                    + " bytes at offset " + surfaceStart + " but file has " + data.length);
        }

        byte[] rgba = new byte[(int) (pixels * 4)];
        if (format.isBlockCompressed()) {
            decodeBlocks(format, data, surfaceStart, rgba, width, height);
        } else {
            int stride = rowStride(data, flags, pitchOrLinearSize, width, height, format, surfaceStart);
            decodeUncompressed(data, surfaceStart, stride, resolved.layout, resolved.forceOpaque, format, rgba, width, height);
        }
        return DecodedImage.ofOwned(width, height, rgba, resolved.srgb);
    }

    // ------------------------------------------------------------------
    // Format resolution
    // ------------------------------------------------------------------

    private static ResolvedFormat resolveDx10(int dxgiFormat) throws UnsupportedDdsFormatException {
        return switch (dxgiFormat) {
            case DXGI_BC1_UNORM -> new ResolvedFormat(DdsFormat.BC1, null, false, false);
            case DXGI_BC1_UNORM_SRGB -> new ResolvedFormat(DdsFormat.BC1, null, true, false);
            case DXGI_BC2_UNORM -> new ResolvedFormat(DdsFormat.BC2, null, false, false);
            case DXGI_BC2_UNORM_SRGB -> new ResolvedFormat(DdsFormat.BC2, null, true, false);
            case DXGI_BC3_UNORM -> new ResolvedFormat(DdsFormat.BC3, null, false, false);
            case DXGI_BC3_UNORM_SRGB -> new ResolvedFormat(DdsFormat.BC3, null, true, false);
            case DXGI_BC4_UNORM -> new ResolvedFormat(DdsFormat.BC4, null, false, false);
            case DXGI_BC7_UNORM -> new ResolvedFormat(DdsFormat.BC7, null, false, false);
            case DXGI_BC7_UNORM_SRGB -> new ResolvedFormat(DdsFormat.BC7, null, true, false);
            case DXGI_R8G8B8A8_UNORM -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.RGBA, false, false);
            case DXGI_R8G8B8A8_UNORM_SRGB -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.RGBA, true, false);
            case DXGI_B8G8R8A8_UNORM -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.BGRA, false, false);
            case DXGI_B8G8R8A8_UNORM_SRGB -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.BGRA, true, false);
            case DXGI_B8G8R8X8_UNORM -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.BGRA, false, true);
            case DXGI_B8G8R8X8_UNORM_SRGB -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.BGRA, true, true);
            case DXGI_R8_UNORM, DXGI_A8_UNORM -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_R8, null, false, false);
            case DXGI_R8G8_UNORM -> new ResolvedFormat(DdsFormat.UNCOMPRESSED_RG8, null, false, false);
            default -> throw new UnsupportedDdsFormatException("unsupported DXGI format " + dxgiFormat
                    + " (0x" + Integer.toHexString(dxgiFormat) + ")");
        };
    }

    private static ResolvedFormat resolveLegacy(int pfFlags, int fourCC, int bitCount,
                                                int rMask, int gMask, int bMask, int aMask) throws UnsupportedDdsFormatException {
        if ((pfFlags & DDPF_FOURCC) != 0) {
            if (fourCC == FOURCC_DXT1) {
                return new ResolvedFormat(DdsFormat.BC1, null, true, false);
            }
            if (fourCC == FOURCC_DXT2) {
                throw new UnsupportedDdsFormatException(
                        "DXT2 premultiplied-alpha is not supported; decode as DXT3 and unpremultiply separately");
            }
            if (fourCC == FOURCC_DXT3) {
                return new ResolvedFormat(DdsFormat.BC2, null, true, false);
            }
            if (fourCC == FOURCC_DXT4) {
                throw new UnsupportedDdsFormatException(
                        "DXT4 premultiplied-alpha is not supported; decode as DXT5 and unpremultiply separately");
            }
            if (fourCC == FOURCC_DXT5) {
                return new ResolvedFormat(DdsFormat.BC3, null, true, false);
            }
            if (fourCC == FOURCC_ATI1 || fourCC == FOURCC_BC4U) {
                return new ResolvedFormat(DdsFormat.BC4, null, false, false);
            }
            throw new UnsupportedDdsFormatException("unsupported DDS FourCC 0x" + Integer.toHexString(fourCC)
                    + " (" + fourccString(fourCC) + ")");
        }
        if ((pfFlags & DDPF_PAL8) != 0) {
            throw new UnsupportedDdsFormatException("paletted DDS surfaces are not supported");
        }

        if ((pfFlags & DDPF_RGB) != 0) {
            switch (bitCount) {
                case 32:
                    // Classic D3D A8R8G8B8 / X8R8G8B8 (memory bytes B, G, R, A).
                    if (rMask == 0x00ff0000 && gMask == 0x0000ff00 && bMask == 0x000000ff) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.BGRA, true,
                                aMask == 0);
                    }
                    // D3D A8B8G8R8 / X8B8G8R8 (memory bytes already R, G, B, A).
                    if (rMask == 0x000000ff && gMask == 0x0000ff00 && bMask == 0x00ff0000) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGBA8, UncompressedLayout.RGBA, true,
                                aMask == 0);
                    }
                    throw new UnsupportedDdsFormatException("unsupported 32-bit RGB layout "
                            + masks(rMask, gMask, bMask, aMask));
                case 24:
                    if (rMask == 0xff0000 && gMask == 0x00ff00 && bMask == 0x0000ff) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGB8, UncompressedLayout.RGB_24, true, true);
                    }
                    if (rMask == 0x0000ff && gMask == 0x00ff00 && bMask == 0xff0000) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_RGB8, UncompressedLayout.BGR_24, true, true);
                    }
                    throw new UnsupportedDdsFormatException("unsupported 24-bit RGB layout "
                            + masks(rMask, gMask, bMask, aMask));
                case 8:
                    if (rMask == 0xff && gMask == 0 && bMask == 0) {
                        // NVTT wrote 8-bit greyscale as RGB; treat as single channel.
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_R8, null, false, false);
                    }
                    throw new UnsupportedDdsFormatException("unsupported 8-bit RGB layout "
                            + masks(rMask, gMask, bMask, aMask));
                default:
                    throw new UnsupportedDdsFormatException("unsupported uncompressed DDS bit count " + bitCount);
            }
        }
        if ((pfFlags & DDPF_LUMINANCE) != 0) {
            switch (bitCount) {
                case 8:
                    if (rMask == 0xff && aMask == 0) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_R8, null, false, false);
                    }
                    throw new UnsupportedDdsFormatException("unsupported 8-bit luminance layout "
                            + masks(rMask, gMask, bMask, aMask));
                case 16:
                    if (rMask == 0x00ff && aMask == 0xff00) {
                        return new ResolvedFormat(DdsFormat.UNCOMPRESSED_RG8, null, false, false);
                    }
                    throw new UnsupportedDdsFormatException("unsupported 16-bit luminance layout "
                            + masks(rMask, gMask, bMask, aMask));
                default:
                    throw new UnsupportedDdsFormatException("unsupported luminance bit count " + bitCount);
            }
        }
        if ((pfFlags & DDPF_ALPHA) != 0) {
            if (bitCount == 8) {
                return new ResolvedFormat(DdsFormat.UNCOMPRESSED_R8, null, false, false);
            }
            throw new UnsupportedDdsFormatException("unsupported alpha-only bit count " + bitCount);
        }
        throw new UnsupportedDdsFormatException("unrecognised DDS pixel format flags 0x" + Integer.toHexString(pfFlags));
    }

    private static String masks(int r, int g, int b, int a) {
        return "r=0x" + Integer.toHexString(r) + " g=0x" + Integer.toHexString(g)
                + " b=0x" + Integer.toHexString(b) + " a=0x" + Integer.toHexString(a);
    }

    private static String fourccString(int fourCC) {
        StringBuilder sb = new StringBuilder(4);
        for (int i = 0; i < 4; i++) {
            int c = (fourCC >>> (8 * i)) & 0xFF;
            sb.append(c >= 0x20 && c <= 0x7E ? (char) c : '?');
        }
        return sb.toString();
    }

    // ------------------------------------------------------------------
    // Surface walking
    // ------------------------------------------------------------------

    /**
     * Row stride (in bytes) for uncompressed surfaces. Uses the header's pitch
     * when it is set, plausible, and the padded rows actually fit in the file;
     * otherwise rows are tightly packed (which {@code decode} has already
     * validated against the file length).
     */
    private static int rowStride(byte[] data, int flags, int pitchOrLinearSize, int width, int height,
                                 DdsFormat format, int surfaceStart) {
        int rowBytes = width * format.bytesPerPixel();
        if ((flags & DDSD_PITCH) != 0
                && pitchOrLinearSize >= rowBytes
                && pitchOrLinearSize <= rowBytes * 8
                && (long) surfaceStart + (long) (height - 1) * pitchOrLinearSize + rowBytes <= (long) data.length) {
            return pitchOrLinearSize;
        }
        return rowBytes;
    }

    private static void decodeBlocks(DdsFormat format, byte[] src, int start, byte[] out,
                                     int width, int height) throws DdsDecodeException {
        int blocksX = format.blocksX(width);
        int blocksY = format.blocksY(height);
        int offset = start;
        int bytesPerBlock = format.bytesPerBlock;
        for (int by = 0; by < blocksY; by++) {
            for (int bx = 0; bx < blocksX; bx++) {
                if (offset + bytesPerBlock > src.length) {
                    throw new DdsDecodeException("DDS surface truncated inside block data");
                }
                switch (format) {
                    case BC1 -> decodeBC1Block(src, offset, out, width, height, bx * 4, by * 4);
                    case BC2 -> decodeBC2Block(src, offset, out, width, height, bx * 4, by * 4);
                    case BC3 -> decodeBC3Block(src, offset, out, width, height, bx * 4, by * 4);
                    case BC4 -> decodeBC4Block(src, offset, out, width, height, bx * 4, by * 4);
                    case BC7 -> decodeBC7Block(src, offset, out, width, height, bx * 4, by * 4);
                    default -> throw new AssertionError(format);
                }
                offset += bytesPerBlock;
            }
        }
    }

    private static void decodeUncompressed(byte[] src, int start, int stride, UncompressedLayout layout,
                                           boolean forceOpaque, DdsFormat format, byte[] out, int width, int height) {
        int bpp = format.bytesPerPixel();
        for (int y = 0; y < height; y++) {
            int row = start + y * stride;
            for (int x = 0; x < width; x++) {
                int i = row + x * bpp;
                int o = (y * width + x) * 4;
                switch (format) {
                    case UNCOMPRESSED_RGBA8 -> {
                        int a = forceOpaque ? 0xFF : (src[i + 3] & 0xFF);
                        switch (layout) {
                            case RGBA -> {
                                out[o] = src[i];
                                out[o + 1] = src[i + 1];
                                out[o + 2] = src[i + 2];
                                out[o + 3] = (byte) a;
                            }
                            case BGRA -> {
                                out[o] = src[i + 2];
                                out[o + 1] = src[i + 1];
                                out[o + 2] = src[i];
                                out[o + 3] = (byte) a;
                            }
                            default -> throw new AssertionError(layout);
                        }
                    }
                    case UNCOMPRESSED_RGB8 -> {
                        switch (layout) {
                            case RGB_24 -> {
                                out[o] = src[i + 2];
                                out[o + 1] = src[i + 1];
                                out[o + 2] = src[i];
                                out[o + 3] = (byte) 0xFF;
                            }
                            case BGR_24 -> {
                                out[o] = src[i];
                                out[o + 1] = src[i + 1];
                                out[o + 2] = src[i + 2];
                                out[o + 3] = (byte) 0xFF;
                            }
                            default -> throw new AssertionError(layout);
                        }
                    }
                    case UNCOMPRESSED_R8 -> {
                        byte v = src[i];
                        out[o] = v;
                        out[o + 1] = v;
                        out[o + 2] = v;
                        out[o + 3] = (byte) 0xFF;
                    }
                    case UNCOMPRESSED_RG8 -> {
                        byte v = src[i];
                        out[o] = v;
                        out[o + 1] = v;
                        out[o + 2] = v;
                        out[o + 3] = src[i + 1];
                    }
                    default -> throw new AssertionError(format);
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // BC1 / BC2 / BC3 / BC4 block decoders
    // ------------------------------------------------------------------

    /** 5-bit to 8-bit expansion. */
    private static int expand5(int v) {
        return (v * 527 + 23) >> 6;
    }

    /** 6-bit to 8-bit expansion. */
    private static int expand6(int v) {
        return (v * 259 + 33) >> 6;
    }

    /**
     * BC1 colour block written as RGBA bytes. When {@code onlyOpaqueMode} is
     * true (BC2/BC3 colour parts) the 4-colour mode is forced; otherwise the
     * transparent-black 3-colour mode is used when {@code c0 <= c1}.
     */
    private static void decodeBC1ColorBlock(byte[] src, int off, byte[] out, int width, int height,
                                            int baseX, int baseY, boolean onlyOpaqueMode) {
        int c0 = readShortLE(src, off);
        int c1 = readShortLE(src, off + 2);
        int r0 = (c0 >> 11) & 0x1F, g0 = (c0 >> 5) & 0x3F, b0 = c0 & 0x1F;
        int r1 = (c1 >> 11) & 0x1F, g1 = (c1 >> 5) & 0x3F, b1 = c1 & 0x1F;

        int[] ref = new int[4]; // packed 0xAABBGGRR, extracted as RGBA below
        ref[0] = packColor(expand5(r0), expand6(g0), expand5(b0));
        ref[1] = packColor(expand5(r1), expand6(g1), expand5(b1));
        if (c0 > c1 || onlyOpaqueMode) {
            ref[2] = packColor(((2 * r0 + r1) * 351 + 61) >> 7, ((2 * g0 + g1) * 2763 + 1039) >> 11,
                    ((2 * b0 + b1) * 351 + 61) >> 7);
            ref[3] = packColor(((r0 + 2 * r1) * 351 + 61) >> 7, ((g0 + 2 * g1) * 2763 + 1039) >> 11,
                    ((b0 + 2 * b1) * 351 + 61) >> 7);
        } else {
            ref[2] = packColor(((r0 + r1) * 1053 + 125) >> 8, ((g0 + g1) * 4145 + 1019) >> 11,
                    ((b0 + b1) * 1053 + 125) >> 8);
            ref[3] = 0; // transparent black
        }

        int colorIndices = readIntLE(src, off + 4);
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int idx = colorIndices & 0x03;
                colorIndices >>>= 2;
                int px = baseX + x, py = baseY + y;
                if (px >= width || py >= height) {
                    continue;
                }
                int c = ref[idx];
                int o = (py * width + px) * 4;
                out[o] = (byte) (c & 0xFF);
                out[o + 1] = (byte) ((c >>> 8) & 0xFF);
                out[o + 2] = (byte) ((c >>> 16) & 0xFF);
                out[o + 3] = (byte) ((c >>> 24) & 0xFF);
            }
        }
    }

    private static int packColor(int r, int g, int b) {
        return 0xFF000000 | (b << 16) | (g << 8) | r;
    }

    private static void decodeBC1Block(byte[] src, int off, byte[] out, int width, int height,
                                       int baseX, int baseY) {
        decodeBC1ColorBlock(src, off, out, width, height, baseX, baseY, false);
    }

    private static void decodeBC2Block(byte[] src, int off, byte[] out, int width, int height,
                                       int baseX, int baseY) {
        decodeBC1ColorBlock(src, off + 8, out, width, height, baseX, baseY, true);
        for (int y = 0; y < 4; y++) {
            int word = readShortLE(src, off + y * 2);
            for (int x = 0; x < 4; x++) {
                int nibble = (word >>> (4 * x)) & 0x0F;
                int px = baseX + x, py = baseY + y;
                if (px >= width || py >= height) {
                    continue;
                }
                out[(py * width + px) * 4 + 3] = (byte) (nibble * 17);
            }
        }
    }

    private static void decodeBC3Block(byte[] src, int off, byte[] out, int width, int height,
                                       int baseX, int baseY) {
        decodeBC1ColorBlock(src, off + 8, out, width, height, baseX, baseY, true);
        int[] alpha = alphaPalette(src[off] & 0xFF, src[off + 1] & 0xFF);
        long indices = readLongLE6(src, off + 2);
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int px = baseX + x, py = baseY + y;
                if (px >= width || py >= height) {
                    continue;
                }
                int idx = (int) ((indices >>> (3 * (y * 4 + x))) & 0x07);
                out[(py * width + px) * 4 + 3] = (byte) alpha[idx];
            }
        }
    }

    private static void decodeBC4Block(byte[] src, int off, byte[] out, int width, int height,
                                       int baseX, int baseY) {
        int[] values = alphaPalette(src[off] & 0xFF, src[off + 1] & 0xFF);
        long indices = readLongLE6(src, off + 2);
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int px = baseX + x, py = baseY + y;
                if (px >= width || py >= height) {
                    continue;
                }
                int idx = (int) ((indices >>> (3 * (y * 4 + x))) & 0x07);
                byte v = (byte) values[idx];
                int o = (py * width + px) * 4;
                out[o] = v;
                out[o + 1] = v;
                out[o + 2] = v;
                out[o + 3] = (byte) 0xFF;
            }
        }
    }

    /**
     * 8-value single-channel palette (BC3 alpha / BC4) using the D3D11 spec's
     * round-to-nearest fixed-point weights. When {@code v0 <= v1} the palette
     * is 6 interpolated values plus the two special 0 and 255 entries.
     */
    private static int[] alphaPalette(int v0, int v1) {
        int[] a = new int[8];
        a[0] = v0;
        a[1] = v1;
        if (v0 > v1) {
            a[2] = (W6[5] * v0 + W6[0] * v1 + 32768) >>> 16;
            a[3] = (W6[4] * v0 + W6[1] * v1 + 32768) >>> 16;
            a[4] = (W6[3] * v0 + W6[2] * v1 + 32768) >>> 16;
            a[5] = (W6[2] * v0 + W6[3] * v1 + 32768) >>> 16;
            a[6] = (W6[1] * v0 + W6[4] * v1 + 32768) >>> 16;
            a[7] = (W6[0] * v0 + W6[5] * v1 + 32768) >>> 16;
        } else {
            a[2] = (W4[3] * v0 + W4[0] * v1 + 32768) >>> 16;
            a[3] = (W4[2] * v0 + W4[1] * v1 + 32768) >>> 16;
            a[4] = (W4[1] * v0 + W4[2] * v1 + 32768) >>> 16;
            a[5] = (W4[0] * v0 + W4[3] * v1 + 32768) >>> 16;
            a[6] = 0;
            a[7] = 255;
        }
        return a;
    }

    private static final int[] W6 = {9363, 18724, 28086, 37450, 46812, 56173};
    private static final int[] W4 = {13107, 26215, 39321, 52429};

    // ------------------------------------------------------------------
    // BC7 block decoder (BPTC)
    // ------------------------------------------------------------------

    /** RGB endpoint precision per mode 0-7. */
    private static final int[] BC7_RGB_PREC = {4, 6, 5, 7, 5, 7, 7, 5};
    /** Alpha endpoint precision per mode 0-7 (0 = opaque mode). */
    private static final int[] BC7_ALPHA_PREC = {0, 0, 0, 0, 6, 8, 7, 5};
    /** Modes that carry P-bits. */
    private static final int BC7_MODE_HAS_PBITS = 0b11001011;

    private static final int[] BC7_W2 = {0, 21, 43, 64};
    private static final int[] BC7_W3 = {0, 9, 18, 27, 37, 46, 55, 64};
    private static final int[] BC7_W4 = {0, 4, 9, 13, 17, 21, 26, 30, 34, 38, 43, 47, 51, 55, 60, 64};

    private static final class BitStream {
        private long low;
        private long high;

        BitStream(long low, long high) {
            this.low = low;
            this.high = high;
        }

        int readBits(int numBits) {
            long mask = (1L << numBits) - 1;
            int bits = (int) (low & mask);
            low >>>= numBits;
            low |= (high & mask) << (64 - numBits);
            high >>>= numBits;
            return bits;
        }

        int readBit() {
            return readBits(1);
        }
    }

    private static int bc7Interpolate(int a, int b, int[] weights, int index) {
        return (a * (64 - weights[index]) + b * weights[index] + 32) >> 6;
    }

    private static void decodeBC7Block(byte[] src, int off, byte[] out, int width, int height,
                                       int baseX, int baseY) {
        BitStream bs = new BitStream(readLongLE(src, off), readLongLE(src, off + 8));

        int mode = 0;
        while (mode < 8 && bs.readBit() == 0) {
            mode++;
        }
        if (mode >= 8) {
            // Reserved mode: transparent black per spec.
            fillBlock(out, width, height, baseX, baseY, 0, 0, 0, 0);
            return;
        }

        int partition = 0;
        int numPartitions = 1;
        int rotation = 0;
        int indexSelectionBit = 0;
        if (mode == 0 || mode == 1 || mode == 2 || mode == 3 || mode == 7) {
            numPartitions = (mode == 0 || mode == 2) ? 3 : 2;
            partition = bs.readBits(mode == 0 ? 4 : 6);
        }
        int numEndpoints = numPartitions * 2;
        if (mode == 4 || mode == 5) {
            rotation = bs.readBits(2);
            if (mode == 4) {
                indexSelectionBit = bs.readBit();
            }
        }

        int[][] endpoints = new int[6][4];
        for (int channel = 0; channel < 3; channel++) {
            for (int e = 0; e < numEndpoints; e++) {
                endpoints[e][channel] = bs.readBits(BC7_RGB_PREC[mode]);
            }
        }
        if (BC7_ALPHA_PREC[mode] > 0) {
            for (int e = 0; e < numEndpoints; e++) {
                endpoints[e][3] = bs.readBits(BC7_ALPHA_PREC[mode]);
            }
        }

        if (mode == 0 || mode == 1 || mode == 3 || mode == 6 || mode == 7) {
            for (int e = 0; e < numEndpoints; e++) {
                endpoints[e][0] <<= 1;
                endpoints[e][1] <<= 1;
                endpoints[e][2] <<= 1;
                endpoints[e][3] <<= 1;
            }
            if (mode == 1) {
                int p0 = bs.readBit();
                int p1 = bs.readBit();
                for (int k = 0; k < 3; k++) {
                    endpoints[0][k] |= p0;
                    endpoints[1][k] |= p0;
                    endpoints[2][k] |= p1;
                    endpoints[3][k] |= p1;
                }
            } else if ((BC7_MODE_HAS_PBITS & (1 << mode)) != 0) {
                for (int e = 0; e < numEndpoints; e++) {
                    int p = bs.readBit();
                    endpoints[e][0] |= p;
                    endpoints[e][1] |= p;
                    endpoints[e][2] |= p;
                    endpoints[e][3] |= p;
                }
            }
        }

        for (int e = 0; e < numEndpoints; e++) {
            int rgbPrec = BC7_RGB_PREC[mode] + ((BC7_MODE_HAS_PBITS >> mode) & 1);
            for (int k = 0; k < 3; k++) {
                endpoints[e][k] = endpoints[e][k] << (8 - rgbPrec);
                endpoints[e][k] |= endpoints[e][k] >> rgbPrec;
            }
            int alphaPrec = BC7_ALPHA_PREC[mode] + ((BC7_MODE_HAS_PBITS >> mode) & 1);
            endpoints[e][3] = endpoints[e][3] << (8 - alphaPrec);
            endpoints[e][3] |= endpoints[e][3] >> alphaPrec;
        }
        if (BC7_ALPHA_PREC[mode] == 0) {
            for (int e = 0; e < numEndpoints; e++) {
                endpoints[e][3] = 0xFF;
            }
        }

        int indexBits = (mode == 0 || mode == 1) ? 3 : (mode == 6 ? 4 : 2);
        int indexBits2 = mode == 4 ? 3 : (mode == 5 ? 2 : 0);
        int[] weights = indexBits == 2 ? BC7_W2 : (indexBits == 3 ? BC7_W3 : BC7_W4);
        int[] weights2 = indexBits2 == 2 ? BC7_W2 : BC7_W3;

        int[][][] partitions = mode == 0 || mode == 2 ? PARTITIONS_3_SUBSET : PARTITIONS_2_SUBSET;
        int[][][] partitionTable = numPartitions == 1 ? null : partitions;

        // Pass 1: primary colour indices (fix-up index has one fewer bit).
        int[] indices = new int[16];
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int ps = numPartitions == 1 ? ((x | y) != 0 ? 0 : 128)
                        : partitionTable[partition][y][x];
                int bits = indexBits;
                if ((ps & 0x80) != 0) {
                    bits--;
                }
                indices[y * 4 + x] = bs.readBits(bits);
            }
        }

        // Pass 2: secondary indices (if any), interpolation and rotation.
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int ps = numPartitions == 1 ? ((x | y) != 0 ? 0 : 128)
                        : partitionTable[partition][y][x];
                int subset = ps & 0x03;
                int index = indices[y * 4 + x];

                int r;
                int g;
                int b;
                int a;
                if (indexBits2 == 0) {
                    r = bc7Interpolate(endpoints[subset * 2][0], endpoints[subset * 2 + 1][0], weights, index);
                    g = bc7Interpolate(endpoints[subset * 2][1], endpoints[subset * 2 + 1][1], weights, index);
                    b = bc7Interpolate(endpoints[subset * 2][2], endpoints[subset * 2 + 1][2], weights, index);
                    a = bc7Interpolate(endpoints[subset * 2][3], endpoints[subset * 2 + 1][3], weights, index);
                } else {
                    int bits2 = (x | y) != 0 ? indexBits2 : indexBits2 - 1;
                    int index2 = bs.readBits(bits2);
                    if (indexSelectionBit == 0) {
                        r = bc7Interpolate(endpoints[subset * 2][0], endpoints[subset * 2 + 1][0], weights, index);
                        g = bc7Interpolate(endpoints[subset * 2][1], endpoints[subset * 2 + 1][1], weights, index);
                        b = bc7Interpolate(endpoints[subset * 2][2], endpoints[subset * 2 + 1][2], weights, index);
                        a = bc7Interpolate(endpoints[subset * 2][3], endpoints[subset * 2 + 1][3], weights2, index2);
                    } else {
                        r = bc7Interpolate(endpoints[subset * 2][0], endpoints[subset * 2 + 1][0], weights2, index2);
                        g = bc7Interpolate(endpoints[subset * 2][1], endpoints[subset * 2 + 1][1], weights2, index2);
                        b = bc7Interpolate(endpoints[subset * 2][2], endpoints[subset * 2 + 1][2], weights2, index2);
                        a = bc7Interpolate(endpoints[subset * 2][3], endpoints[subset * 2 + 1][3], weights, index);
                    }
                }

                switch (rotation) {
                    case 1 -> {
                        int t = a;
                        a = r;
                        r = t;
                    }
                    case 2 -> {
                        int t = a;
                        a = g;
                        g = t;
                    }
                    case 3 -> {
                        int t = a;
                        a = b;
                        b = t;
                    }
                    default -> { }
                }

                int px = baseX + x, py = baseY + y;
                if (px < width && py < height) {
                    int o = (py * width + px) * 4;
                    out[o] = (byte) r;
                    out[o + 1] = (byte) g;
                    out[o + 2] = (byte) b;
                    out[o + 3] = (byte) a;
                }
            }
        }
    }

    private static void fillBlock(byte[] out, int width, int height, int baseX, int baseY,
                                  int r, int g, int b, int a) {
        for (int y = baseY; y < baseY + 4 && y < height; y++) {
            for (int x = baseX; x < baseX + 4 && x < width; x++) {
                int o = (y * width + x) * 4;
                out[o] = (byte) r;
                out[o + 1] = (byte) g;
                out[o + 2] = (byte) b;
                out[o + 3] = (byte) a;
            }
        }
    }

    // ------------------------------------------------------------------
    // BC7 partition tables (transcribed from the D3D11/BPTC spec; fix-up
    // entries carry the 0x80 bit, subset index is the low 2 bits).
    // ------------------------------------------------------------------

    private static final int[][][] PARTITIONS_2_SUBSET = {
        { {128, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 129} }, // 0
        { {128, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 129} }, // 1
        { {128, 1, 1, 1}, {0, 1, 1, 1}, {0, 1, 1, 1}, {0, 1, 1, 129} }, // 2
        { {128, 0, 0, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 1, 1, 129} }, // 3
        { {128, 0, 0, 0}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 1, 129} }, // 4
        { {128, 0, 1, 1}, {0, 1, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 129} }, // 5
        { {128, 0, 0, 1}, {0, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 129} }, // 6
        { {128, 0, 0, 0}, {0, 0, 0, 1}, {0, 0, 1, 1}, {0, 1, 1, 129} }, // 7
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}, {0, 0, 1, 129} }, // 8
        { {128, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 129} }, // 9
        { {128, 0, 0, 0}, {0, 0, 0, 1}, {0, 1, 1, 1}, {1, 1, 1, 129} }, // 10
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 1}, {0, 1, 1, 129} }, // 11
        { {128, 0, 0, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 129} }, // 12
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 129} }, // 13
        { {128, 0, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 129} }, // 14
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {1, 1, 1, 129} }, // 15
        { {128, 0, 0, 0}, {1, 0, 0, 0}, {1, 1, 1, 0}, {1, 1, 1, 129} }, // 16
        { {128, 1, 129, 1}, {0, 0, 0, 1}, {0, 0, 0, 0}, {0, 0, 0, 0} }, // 17
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {129, 0, 0, 0}, {1, 1, 1, 0} }, // 18
        { {128, 1, 129, 1}, {0, 0, 1, 1}, {0, 0, 0, 1}, {0, 0, 0, 0} }, // 19
        { {128, 0, 129, 1}, {0, 0, 0, 1}, {0, 0, 0, 0}, {0, 0, 0, 0} }, // 20
        { {128, 0, 0, 0}, {1, 0, 0, 0}, {129, 1, 0, 0}, {1, 1, 1, 0} }, // 21
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {129, 0, 0, 0}, {1, 1, 0, 0} }, // 22
        { {128, 1, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 1}, {0, 0, 0, 129} }, // 23
        { {128, 0, 129, 1}, {0, 0, 0, 1}, {0, 0, 0, 1}, {0, 0, 0, 0} }, // 24
        { {128, 0, 0, 0}, {1, 0, 0, 0}, {129, 0, 0, 0}, {1, 1, 0, 0} }, // 25
        { {128, 1, 129, 0}, {0, 1, 1, 0}, {0, 1, 1, 0}, {0, 1, 1, 0} }, // 26
        { {128, 0, 129, 1}, {0, 1, 1, 0}, {0, 1, 1, 0}, {1, 1, 0, 0} }, // 27
        { {128, 0, 0, 1}, {0, 1, 1, 1}, {129, 1, 1, 0}, {1, 0, 0, 0} }, // 28
        { {128, 0, 0, 0}, {1, 1, 1, 1}, {129, 1, 1, 1}, {0, 0, 0, 0} }, // 29
        { {128, 1, 129, 1}, {0, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 1, 0} }, // 30
        { {128, 0, 129, 1}, {1, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 0} }, // 31
        { {128, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 129} }, // 32
        { {128, 0, 0, 0}, {1, 1, 1, 1}, {0, 0, 0, 0}, {1, 1, 1, 129} }, // 33
        { {128, 1, 0, 1}, {1, 0, 129, 0}, {0, 1, 0, 1}, {1, 0, 1, 0} }, // 34
        { {128, 0, 1, 1}, {0, 0, 1, 1}, {129, 1, 0, 0}, {1, 1, 0, 0} }, // 35
        { {128, 0, 129, 1}, {1, 1, 0, 0}, {0, 0, 1, 1}, {1, 1, 0, 0} }, // 36
        { {128, 1, 0, 1}, {0, 1, 0, 1}, {129, 0, 1, 0}, {1, 0, 1, 0} }, // 37
        { {128, 1, 1, 0}, {1, 0, 0, 1}, {0, 1, 1, 0}, {1, 0, 0, 129} }, // 38
        { {128, 1, 0, 1}, {1, 0, 1, 0}, {1, 0, 1, 0}, {0, 1, 0, 129} }, // 39
        { {128, 1, 129, 1}, {0, 0, 1, 1}, {1, 1, 0, 0}, {1, 1, 1, 0} }, // 40
        { {128, 0, 0, 1}, {0, 0, 1, 1}, {129, 1, 0, 0}, {1, 0, 0, 0} }, // 41
        { {128, 0, 129, 1}, {0, 0, 1, 0}, {0, 1, 0, 0}, {1, 1, 0, 0} }, // 42
        { {128, 0, 129, 1}, {1, 0, 1, 1}, {1, 1, 0, 1}, {1, 1, 0, 0} }, // 43
        { {128, 1, 129, 0}, {1, 0, 0, 1}, {1, 0, 0, 1}, {0, 1, 1, 0} }, // 44
        { {128, 0, 1, 1}, {1, 1, 0, 0}, {1, 1, 0, 0}, {0, 0, 1, 129} }, // 45
        { {128, 1, 1, 0}, {0, 1, 1, 0}, {1, 0, 0, 1}, {1, 0, 0, 129} }, // 46
        { {128, 0, 0, 0}, {0, 1, 129, 0}, {0, 1, 1, 0}, {0, 0, 0, 0} }, // 47
        { {128, 1, 0, 0}, {1, 1, 129, 0}, {0, 1, 0, 0}, {0, 0, 0, 0} }, // 48
        { {128, 0, 129, 0}, {0, 1, 1, 1}, {0, 0, 1, 0}, {0, 0, 0, 0} }, // 49
        { {128, 0, 0, 0}, {0, 0, 129, 0}, {0, 1, 1, 1}, {0, 0, 1, 0} }, // 50
        { {128, 0, 0, 0}, {0, 1, 0, 0}, {129, 1, 1, 0}, {0, 1, 0, 0} }, // 51
        { {128, 1, 1, 0}, {1, 1, 0, 0}, {1, 0, 0, 1}, {0, 0, 1, 129} }, // 52
        { {128, 0, 1, 1}, {0, 1, 1, 0}, {1, 1, 0, 0}, {1, 0, 0, 129} }, // 53
        { {128, 1, 129, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}, {1, 1, 0, 0} }, // 54
        { {128, 0, 129, 1}, {1, 0, 0, 1}, {1, 1, 0, 0}, {0, 1, 1, 0} }, // 55
        { {128, 1, 1, 0}, {1, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 0, 129} }, // 56
        { {128, 1, 1, 0}, {0, 0, 1, 1}, {0, 0, 1, 1}, {1, 0, 0, 129} }, // 57
        { {128, 1, 1, 1}, {1, 1, 1, 0}, {1, 0, 0, 0}, {0, 0, 0, 129} }, // 58
        { {128, 0, 0, 1}, {1, 0, 0, 0}, {1, 1, 1, 0}, {0, 1, 1, 129} }, // 59
        { {128, 0, 0, 0}, {1, 1, 1, 1}, {0, 0, 1, 1}, {0, 0, 1, 129} }, // 60
        { {128, 0, 129, 1}, {0, 0, 1, 1}, {1, 1, 1, 1}, {0, 0, 0, 0} }, // 61
        { {128, 0, 129, 0}, {0, 0, 1, 0}, {1, 1, 1, 0}, {1, 1, 1, 0} }, // 62
        { {128, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 1}, {0, 1, 1, 129} }, // 63
    };

    private static final int[][][] PARTITIONS_3_SUBSET = {
        { {128, 0, 1, 129}, {0, 0, 1, 1}, {0, 2, 2, 1}, {2, 2, 2, 130} }, // 0
        { {128, 0, 0, 129}, {0, 0, 1, 1}, {130, 2, 1, 1}, {2, 2, 2, 1} }, // 1
        { {128, 0, 0, 0}, {2, 0, 0, 1}, {130, 2, 1, 1}, {2, 2, 1, 129} }, // 2
        { {128, 2, 2, 130}, {0, 0, 2, 2}, {0, 0, 1, 1}, {0, 1, 1, 129} }, // 3
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {129, 1, 2, 2}, {1, 1, 2, 130} }, // 4
        { {128, 0, 1, 129}, {0, 0, 1, 1}, {0, 0, 2, 2}, {0, 0, 2, 130} }, // 5
        { {128, 0, 2, 130}, {0, 0, 2, 2}, {1, 1, 1, 1}, {1, 1, 1, 129} }, // 6
        { {128, 0, 1, 1}, {0, 0, 1, 1}, {130, 2, 1, 1}, {2, 2, 1, 129} }, // 7
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {129, 1, 1, 1}, {2, 2, 2, 130} }, // 8
        { {128, 0, 0, 0}, {1, 1, 1, 1}, {129, 1, 1, 1}, {2, 2, 2, 130} }, // 9
        { {128, 0, 0, 0}, {1, 1, 129, 1}, {2, 2, 2, 2}, {2, 2, 2, 130} }, // 10
        { {128, 0, 1, 2}, {0, 0, 129, 2}, {0, 0, 1, 2}, {0, 0, 1, 130} }, // 11
        { {128, 1, 1, 2}, {0, 1, 129, 2}, {0, 1, 1, 2}, {0, 1, 1, 130} }, // 12
        { {128, 1, 2, 2}, {0, 129, 2, 2}, {0, 1, 2, 2}, {0, 1, 2, 130} }, // 13
        { {128, 0, 1, 129}, {0, 1, 1, 2}, {1, 1, 2, 2}, {1, 2, 2, 130} }, // 14
        { {128, 0, 1, 129}, {2, 0, 0, 1}, {130, 2, 0, 0}, {2, 2, 2, 0} }, // 15
        { {128, 0, 0, 129}, {0, 0, 1, 1}, {0, 1, 1, 2}, {1, 1, 2, 130} }, // 16
        { {128, 1, 1, 129}, {0, 0, 1, 1}, {130, 0, 0, 1}, {2, 2, 0, 0} }, // 17
        { {128, 0, 0, 0}, {1, 1, 2, 2}, {129, 1, 2, 2}, {1, 1, 2, 130} }, // 18
        { {128, 0, 2, 130}, {0, 0, 2, 2}, {0, 0, 2, 2}, {1, 1, 1, 129} }, // 19
        { {128, 1, 1, 129}, {0, 1, 1, 1}, {0, 2, 2, 2}, {0, 2, 2, 130} }, // 20
        { {128, 0, 0, 129}, {0, 0, 0, 1}, {130, 2, 2, 1}, {2, 2, 2, 1} }, // 21
        { {128, 0, 0, 0}, {0, 0, 129, 1}, {0, 1, 2, 2}, {0, 1, 2, 130} }, // 22
        { {128, 0, 0, 0}, {1, 1, 0, 0}, {130, 2, 129, 0}, {2, 2, 1, 0} }, // 23
        { {128, 1, 2, 130}, {0, 129, 2, 2}, {0, 0, 1, 1}, {0, 0, 0, 0} }, // 24
        { {128, 0, 1, 2}, {0, 0, 1, 2}, {129, 1, 2, 2}, {2, 2, 2, 130} }, // 25
        { {128, 1, 1, 0}, {1, 2, 130, 1}, {129, 2, 2, 1}, {0, 1, 1, 0} }, // 26
        { {128, 0, 0, 0}, {0, 1, 129, 0}, {1, 2, 130, 1}, {1, 2, 2, 1} }, // 27
        { {128, 0, 2, 2}, {1, 1, 0, 2}, {129, 1, 0, 2}, {0, 0, 2, 130} }, // 28
        { {128, 1, 1, 0}, {0, 129, 1, 0}, {2, 0, 0, 2}, {2, 2, 2, 130} }, // 29
        { {128, 0, 1, 1}, {0, 1, 2, 2}, {0, 1, 130, 2}, {0, 0, 1, 129} }, // 30
        { {128, 0, 0, 0}, {2, 0, 0, 0}, {130, 2, 1, 1}, {2, 2, 2, 129} }, // 31
        { {128, 0, 0, 0}, {0, 0, 0, 2}, {129, 1, 2, 2}, {1, 2, 2, 130} }, // 32
        { {128, 2, 2, 130}, {0, 0, 2, 2}, {0, 0, 1, 2}, {0, 0, 1, 129} }, // 33
        { {128, 0, 1, 129}, {0, 0, 1, 2}, {0, 0, 2, 2}, {0, 2, 2, 130} }, // 34
        { {128, 1, 2, 0}, {0, 129, 2, 0}, {0, 1, 130, 0}, {0, 1, 2, 0} }, // 35
        { {128, 0, 0, 0}, {1, 1, 129, 1}, {2, 2, 130, 2}, {0, 0, 0, 0} }, // 36
        { {128, 1, 2, 0}, {1, 2, 0, 1}, {130, 0, 129, 2}, {0, 1, 2, 0} }, // 37
        { {128, 1, 2, 0}, {2, 0, 1, 2}, {129, 130, 0, 1}, {0, 1, 2, 0} }, // 38
        { {128, 0, 1, 1}, {2, 2, 0, 0}, {1, 1, 130, 2}, {0, 0, 1, 129} }, // 39
        { {128, 0, 1, 1}, {1, 1, 130, 2}, {2, 2, 0, 0}, {0, 0, 1, 129} }, // 40
        { {128, 1, 0, 129}, {0, 1, 0, 1}, {2, 2, 2, 2}, {2, 2, 2, 130} }, // 41
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {130, 1, 2, 1}, {2, 1, 2, 129} }, // 42
        { {128, 0, 2, 2}, {1, 129, 2, 2}, {0, 0, 2, 2}, {1, 1, 2, 130} }, // 43
        { {128, 0, 2, 130}, {0, 0, 1, 1}, {0, 0, 2, 2}, {0, 0, 1, 129} }, // 44
        { {128, 2, 2, 0}, {1, 2, 130, 1}, {0, 2, 2, 0}, {1, 2, 2, 129} }, // 45
        { {128, 1, 0, 1}, {2, 2, 130, 2}, {2, 2, 2, 2}, {0, 1, 0, 129} }, // 46
        { {128, 0, 0, 0}, {2, 1, 2, 1}, {130, 1, 2, 1}, {2, 1, 2, 129} }, // 47
        { {128, 1, 0, 129}, {0, 1, 0, 1}, {0, 1, 0, 1}, {2, 2, 2, 130} }, // 48
        { {128, 2, 2, 130}, {0, 1, 1, 1}, {0, 2, 2, 2}, {0, 1, 1, 129} }, // 49
        { {128, 0, 0, 2}, {1, 129, 1, 2}, {0, 0, 0, 2}, {1, 1, 1, 130} }, // 50
        { {128, 0, 0, 0}, {2, 129, 1, 2}, {2, 1, 1, 2}, {2, 1, 1, 130} }, // 51
        { {128, 2, 2, 2}, {0, 129, 1, 1}, {0, 1, 1, 1}, {0, 2, 2, 130} }, // 52
        { {128, 0, 0, 2}, {1, 1, 1, 2}, {129, 1, 1, 2}, {0, 0, 0, 130} }, // 53
        { {128, 1, 1, 0}, {0, 129, 1, 0}, {0, 1, 1, 0}, {2, 2, 2, 130} }, // 54
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {2, 1, 129, 2}, {2, 1, 1, 130} }, // 55
        { {128, 1, 1, 0}, {0, 129, 1, 0}, {2, 2, 2, 2}, {2, 2, 2, 130} }, // 56
        { {128, 0, 2, 2}, {0, 0, 1, 1}, {0, 0, 129, 1}, {0, 0, 2, 130} }, // 57
        { {128, 0, 2, 2}, {1, 1, 2, 2}, {129, 1, 2, 2}, {0, 0, 2, 130} }, // 58
        { {128, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {2, 129, 1, 130} }, // 59
        { {128, 0, 0, 130}, {0, 0, 0, 1}, {0, 0, 0, 2}, {0, 0, 0, 129} }, // 60
        { {128, 2, 2, 2}, {1, 2, 2, 2}, {0, 2, 2, 2}, {129, 2, 2, 130} }, // 61
        { {128, 1, 0, 129}, {2, 2, 2, 2}, {2, 2, 2, 2}, {2, 2, 2, 130} }, // 62
        { {128, 1, 1, 129}, {2, 0, 1, 1}, {130, 2, 0, 1}, {2, 2, 2, 0} }, // 63
    };

    // ------------------------------------------------------------------
    // Little-endian read helpers
    // ------------------------------------------------------------------

    private static int make4cc(String s) {
        return s.charAt(0) | (s.charAt(1) << 8) | (s.charAt(2) << 16) | (s.charAt(3) << 24);
    }

    private static int readIntLE(byte[] src, int off) {
        return (src[off] & 0xFF) | ((src[off + 1] & 0xFF) << 8)
                | ((src[off + 2] & 0xFF) << 16) | ((src[off + 3] & 0xFF) << 24);
    }

    private static int readShortLE(byte[] src, int off) {
        return (src[off] & 0xFF) | ((src[off + 1] & 0xFF) << 8);
    }

    private static long readLongLE(byte[] src, int off) {
        long v = 0;
        for (int i = 0; i < 8; i++) {
            v |= (long) (src[off + i] & 0xFF) << (8 * i);
        }
        return v;
    }

    /** Reads 6 bytes little-endian into a long (BC3/BC4 index stream). */
    private static long readLongLE6(byte[] src, int off) {
        long v = 0;
        for (int i = 0; i < 6; i++) {
            v |= (long) (src[off + i] & 0xFF) << (8 * i);
        }
        return v;
    }
}
