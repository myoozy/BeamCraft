package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

/**
 * Decoder tests against hand-computed expected pixels. Expected values were
 * derived from the D3D11/BCn spec (565 expansion, 1/3 and 2/3 interpolation, the
 * round-to-nearest single-channel palettes, BC7 mode 6 weight table), not from
 * the implementation under test.
 */
class DdsDecoderTest {

    private static final int DDPF_RGB = 0x40;
    private static final int DDPF_ALPHAPIXELS = 0x1;
    private static final int DDPF_FOURCC = 0x4;
    private static final int DDPF_LUMINANCE = 0x20000;
    private static final int DDPF_ALPHA = 0x2;
    private static final int DDPF_PAL8 = 0x20;

    // ------------------------------------------------------------------
    // Uncompressed
    // ------------------------------------------------------------------

    @Test
    void decodesLegacyA8R8G8B8() throws Exception {
        // Memory bytes [B, G, R, A] per pixel (classic D3D A8R8G8B8).
        byte[] surface = {0x10, 0x20, 0x30, 0x40, (byte) 0xAA, (byte) 0xBB, (byte) 0xCC, (byte) 0xFF};
        byte[] dds = DdsFixtures.legacyDds(2, 1, "", DDPF_RGB | DDPF_ALPHAPIXELS, 32,
                0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000, surface);
        DecodedImage img = DdsDecoder.decode(dds);
        assertEquals(2, img.width());
        assertEquals(1, img.height());
        assertTrue(img.isSrgb(), "legacy colour defaults to sRGB");
        assertEquals(0x40302010, img.getPixelRgba(0, 0));
        assertEquals(0xFFCCBBAA, img.getPixelRgba(1, 0));
    }

    @Test
    void decodesLegacyA8B8G8R8() throws Exception {
        // Memory bytes [R, G, B, A] per pixel (D3D A8B8G8R8).
        byte[] surface = {0x10, 0x20, 0x30, 0x40};
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB | DDPF_ALPHAPIXELS, 32,
                0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000, surface);
        assertEquals(0x40102030, DdsDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void forcesOpaqueForX8WithoutAlphaChannel() throws Exception {
        // X8R8G8B8: alpha mask 0, alpha byte is padding garbage.
        byte[] surface = {0x10, 0x20, 0x30, 0x55};
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB, 32,
                0x00ff0000, 0x0000ff00, 0x000000ff, 0, surface);
        assertEquals(0xFF302010, DdsDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void decodes24BitRgb() throws Exception {
        byte[] surface = {0x10, 0x20, 0x30};
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB, 24,
                0xff0000, 0x00ff00, 0x0000ff, 0, surface);
        assertEquals(0xFF302010, DdsDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void decodesLuminanceSingleChannelReplicated() throws Exception {
        byte[] surface = {(byte) 0x99};
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_LUMINANCE, 8, 0xff, 0, 0, 0, surface);
        assertEquals(0xFF999999, DdsDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void decodesAlphaOnlyAsSingleChannel() throws Exception {
        byte[] surface = {0x40};
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_ALPHA, 8, 0, 0, 0, 0, surface);
        assertEquals(0xFF404040, DdsDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void decodesDx10Rgba8AndFlagsSrgb() throws Exception {
        byte[] surface = {0x10, 0x20, 0x30, 0x40};
        DecodedImage nonSrgb = DdsDecoder.decode(DdsFixtures.dx10Dds(1, 1, 28, surface)); // R8G8B8A8_UNORM
        assertFalse(nonSrgb.isSrgb());
        assertEquals(0x40102030, nonSrgb.getPixelRgba(0, 0));

        DecodedImage srgb = DdsDecoder.decode(DdsFixtures.dx10Dds(1, 1, 29, surface)); // _UNORM_SRGB
        assertTrue(srgb.isSrgb());
        assertEquals(0x40102030, srgb.getPixelRgba(0, 0));
    }

    @Test
    void decodesDx10Bc4WithSrgbFalse() throws Exception {
        byte[] surface = {0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // one BC4 block, index 0
        DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(4, 4, 80, surface)); // BC4_UNORM
        assertFalse(img.isSrgb());
        assertEquals(0xFF7F7F7F, img.getPixelRgba(0, 0));
    }

    @Test
    void decodesDx10R8SingleChannel() throws Exception {
        byte[] surface = {(byte) 0x99};
        DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(1, 1, 61, surface)); // R8_UNORM
        assertEquals(0xFF999999, img.getPixelRgba(0, 0));
        assertFalse(img.isSrgb());
    }

    // ------------------------------------------------------------------
    // BC1 / DXT1
    // ------------------------------------------------------------------

    @Test
    void decodesBc1FourColourMode() throws Exception {
        // c0=white, c1=black, c0>c1 -> 4-colour, opaque.
        // ref0=(255,255,255) ref1=(0,0,0) ref2=(170,170,170) ref3=(85,85,85)
        byte[] block = DdsFixtures.bc1Block(0xFFFF, 0x0000, 0xFF001BE4);
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertTrue(img.isSrgb(), "legacy DXT1 defaults to sRGB");
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 0));
        assertEquals(0xFF000000, img.getPixelRgba(1, 0));
        assertEquals(0xFFAAAAAA, img.getPixelRgba(2, 0));
        assertEquals(0xFF555555, img.getPixelRgba(3, 0));
        assertEquals(0xFF555555, img.getPixelRgba(0, 1));
        assertEquals(0xFFAAAAAA, img.getPixelRgba(1, 1));
        assertEquals(0xFF000000, img.getPixelRgba(2, 1));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(3, 1));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 2));
        assertEquals(0xFF555555, img.getPixelRgba(3, 3));
    }

    @Test
    void decodesBc1ThreeColourTransparentMode() throws Exception {
        // c0=black, c1=white, c0<=c1 -> 3-colour + transparent black.
        byte[] block = DdsFixtures.bc1Block(0x0000, 0xFFFF, 0xFFFFFFFF); // all index 3
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(0x00000000, img.getPixelRgba(0, 0));
        assertEquals(0x00000000, img.getPixelRgba(3, 3));
    }

    @Test
    void decodesBc1PartialEdgeBlocks() throws Exception {
        // 3x3 uses a single 4x4 block; texels past the edge must be skipped.
        byte[] block = DdsFixtures.bc1Block(0xFFFF, 0x0000, 0); // all index 0 -> white
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(3, 3, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(3, img.width());
        assertEquals(3, img.height());
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 0));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(2, 2));
    }

    // ------------------------------------------------------------------
    // BC2 / DXT3 and BC3 / DXT5
    // ------------------------------------------------------------------

    @Test
    void decodesBc2ExplicitAlpha() throws Exception {
        int[] alphaWords = {0x8765, 0xFFF0, 0xFF0F, 0xFFFF};
        // row0 nibbles 5,6,7,8 -> alphas 85,102,119,136; colour block all white.
        byte[] block = DdsFixtures.bc2Block(alphaWords, 0xFFFF, 0x0000, 0);
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT3", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(0x55FFFFFF, img.getPixelRgba(0, 0));
        assertEquals(0x66FFFFFF, img.getPixelRgba(1, 0));
        assertEquals(0x77FFFFFF, img.getPixelRgba(2, 0));
        assertEquals(0x88FFFFFF, img.getPixelRgba(3, 0));
        assertEquals(0x00FFFFFF, img.getPixelRgba(0, 1)); // nibble 0 -> alpha 0
        assertEquals(0xFFFFFFFF, img.getPixelRgba(1, 1));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 2));
        assertEquals(0x00FFFFFF, img.getPixelRgba(1, 2));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(3, 3));
    }

    @Test
    void decodesBc3SmoothAlpha() throws Exception {
        // a0=255, a1=0 -> palette [255,0,219,182,146,109,73,36] (round-to-nearest).
        int[] alphaIndices = {2, 3, 4, 5, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        byte[] block = DdsFixtures.bc3Block(0xFF, 0x00, DdsFixtures.packIndex3(alphaIndices), 0xFFFF, 0x0000, 0);
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT5", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(0xDBFFFFFF, img.getPixelRgba(0, 0)); // 219
        assertEquals(0xB6FFFFFF, img.getPixelRgba(1, 0)); // 182
        assertEquals(0x92FFFFFF, img.getPixelRgba(2, 0)); // 146
        assertEquals(0x6DFFFFFF, img.getPixelRgba(3, 0)); // 109
        assertEquals(0x49FFFFFF, img.getPixelRgba(0, 1)); // 73
        assertEquals(0x24FFFFFF, img.getPixelRgba(1, 1)); // 36
        assertEquals(0xFFFFFFFF, img.getPixelRgba(3, 3));
    }

    // ------------------------------------------------------------------
    // BC4 / ATI1 (opacity)
    // ------------------------------------------------------------------

    @Test
    void decodesBc4InterpolatedPalette() throws Exception {
        // v0=255 > v1=0 -> palette [255,0,219,182,146,109,73,36], replicated to RGB, alpha 255.
        int[] indices = {7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 7, 7, 7, 7};
        byte[] block = DdsFixtures.bc4Block(0xFF, 0x00, DdsFixtures.packIndex3(indices));
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "ATI1", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertFalse(img.isSrgb(), "BC4 mask is never sRGB");
        assertEquals(0xFF242424, img.getPixelRgba(0, 0)); // 36
        assertEquals(0xFF494949, img.getPixelRgba(1, 0)); // 73
        assertEquals(0xFF6D6D6D, img.getPixelRgba(2, 0)); // 109
        assertEquals(0xFF929292, img.getPixelRgba(3, 0)); // 146
        assertEquals(0xFFB6B6B6, img.getPixelRgba(0, 1)); // 182
        assertEquals(0xFFDBDBDB, img.getPixelRgba(1, 1)); // 219
        assertEquals(0xFF000000, img.getPixelRgba(2, 1)); // 0
        assertEquals(0xFFFFFFFF, img.getPixelRgba(3, 1)); // 255
        assertEquals(0xFF242424, img.getPixelRgba(3, 3)); // 36
    }

    @Test
    void decodesBc4SpecialEndpointMode() throws Exception {
        // v0=0 <= v1=255 -> palette [0,255,51,102,153,204,0,255].
        int[] indices = {6, 7, 2, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        byte[] block = DdsFixtures.bc4Block(0x00, 0xFF, DdsFixtures.packIndex3(indices));
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "BC4U", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(0xFF000000, img.getPixelRgba(0, 0));
        assertEquals(0xFFFFFFFF, img.getPixelRgba(1, 0));
        assertEquals(0xFF333333, img.getPixelRgba(2, 0)); // 51
        assertEquals(0xFF666666, img.getPixelRgba(3, 0)); // 102
        assertEquals(0xFF999999, img.getPixelRgba(0, 1)); // 153
        assertEquals(0xFFCCCCCC, img.getPixelRgba(1, 1)); // 204
    }

    @Test
    void decodesBc4SinglePixel() throws Exception {
        // 1x1 uses one block; only texel (0,0) is emitted.
        byte[] block = DdsFixtures.bc4Block(0xFF, 0x00, 0L); // index 0 -> 255
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(1, 1, "ATI1", DDPF_FOURCC, 0, 0, 0, 0, 0, block));
        assertEquals(1, img.width());
        assertEquals(1, img.height());
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 0));
    }

    // ------------------------------------------------------------------
    // BC7
    // ------------------------------------------------------------------

    @Test
    void decodesBc7Mode6() throws Exception {
        // Endpoint0 = white (alpha 255), endpoint1 = black (alpha 0), P-bits 1.
        // W4 weights: idx7=30 -> 135, idx8=34 -> 120, idx15=64 -> 0.
        int[] indices = {0, 15, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        byte[] block = DdsFixtures.bc7Mode6Block(indices);
        DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(4, 4, 98, block)); // BC7_UNORM
        assertEquals(0xFFFFFFFF, img.getPixelRgba(0, 0));
        assertEquals(0x00000000, img.getPixelRgba(1, 0));
        assertEquals(0x87878787, img.getPixelRgba(2, 0)); // 135
        assertEquals(0x78787878, img.getPixelRgba(3, 0)); // 120
        assertEquals(0xFFFFFFFF, img.getPixelRgba(3, 3));
    }

    @Test
    void decodesBc7Mode0Opaque() throws Exception {
        // All endpoints 15 with P-bit 1 -> white, opaque, all subsets.
        byte[] block = DdsFixtures.bc7Mode0Block(15);
        DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(4, 4, 99, block)); // BC7_UNORM_SRGB
        assertTrue(img.isSrgb());
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                assertEquals(0xFFFFFFFF, img.getPixelRgba(x, y));
            }
        }
    }

    @Test
    void decodesBc7ReservedModeAsTransparentBlack() throws Exception {
        DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(4, 4, 98, DdsFixtures.bc7ReservedBlock()));
        assertEquals(0x00000000, img.getPixelRgba(0, 0));
        assertEquals(0x00000000, img.getPixelRgba(3, 3));
    }

    /**
     * Cross-validates the Java BC7 decoder against independent reference
     * vectors for every mode 0-7 (all rotations / index selections for the
     * rotation modes). The expected pixels were produced by the bcdec reference
     * decoder, not by this implementation; see {@link Bc7ReferenceVectors} and
     * {@code tools/bc7-reference-vectors/} for provenance.
     */
    @Test
    void bc7MatchesBcdecReferenceVectors() throws Exception {
        for (Bc7ReferenceVectors.ReferenceVector v : Bc7ReferenceVectors.VECTORS) {
            DecodedImage img = DdsDecoder.decode(DdsFixtures.dx10Dds(4, 4, 98, v.block)); // BC7_UNORM
            for (int i = 0; i < 16; i++) {
                assertEquals(v.expected[i], img.getPixelRgba(i % 4, i / 4),
                        "vector " + v.name + " texel (" + (i % 4) + "," + (i / 4) + ")");
            }
        }
    }

    /**
     * The bcdec-generated mode-6 block must be byte-identical to the
     * hand-computed {@link DdsFixtures#bc7Mode6Block} fixture. That ties the
     * reference toolchain to the independent spec-derived expectations asserted
     * by {@link #decodesBc7Mode6}.
     */
    @Test
    void mode6ReferenceBlockMatchesHandFixture() {
        assertArrayEquals(DdsFixtures.bc7Mode6Block(new int[] {0, 15, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}),
                Bc7ReferenceVectors.MODE6_BLOCK);
    }

    // ------------------------------------------------------------------
    // Structural safety (hostile headers)
    // ------------------------------------------------------------------

    @Test
    void rejectsHeaderSizeNot124() {
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB, 32,
                0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000, new byte[4]);
        DdsFixtures.writeInt(dds, 4, 123);
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsPixelFormatSizeNot32() {
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB, 32,
                0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000, new byte[4]);
        DdsFixtures.writeInt(dds, 76, 16);
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsDx10ArraySizeNotExactlyOne() {
        // arraySize 0 and 2 are both invalid for this 2D-only decoder.
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 3, 0, 0, new byte[4 * 4 * 4])));
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 3, 0, 2, new byte[4 * 4 * 4])));
    }

    @Test
    void rejectsDx10NonTexture2dDimension() {
        // Only D3D10_RESOURCE_DIMENSION_TEXTURE2D (3) is accepted.
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 1, 0, 1, new byte[4 * 4 * 4]))); // BUFFER
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 2, 0, 1, new byte[4 * 4 * 4]))); // TEXTURE1D
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 4, 0, 1, new byte[4 * 4 * 4]))); // TEXTURE3D
    }

    @Test
    void rejectsDx10CubemapMiscFlag() {
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.dx10DdsRaw(4, 4, 28, 3, 0x4, 1, new byte[4 * 4 * 4])));
    }

    @Test
    void rejectsLegacyVolumeCaps() {
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.legacyDdsWithCaps2(4, 4, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0,
                        0x200000, new byte[8]))); // DDSCAPS2_VOLUME
    }

    /**
     * A header claiming huge dimensions must fail with a clean
     * {@link DdsDecodeException}, never a {@link NegativeArraySizeException} or
     * OOME from integer overflow, and the tiny fixture must not allocate a huge
     * surface.
     */
    @Test
    void rejectsHugeDimensionsWithoutAllocating() {
        byte[] dds = DdsFixtures.dx10DdsRaw(0x7FFFFFFF, 0x7FFFFFFF, 28, 3, 0, 1, new byte[0]);
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(dds));
        byte[] wide = DdsFixtures.dx10DdsRaw(0x7FFFFFFF, 4, 28, 3, 0, 1, new byte[0]);
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(wide));
    }

    @Test
    void rejectsDxt2AndDxt4PremultipliedAlpha() throws Exception {
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT2", DDPF_FOURCC, 0, 0, 0, 0, 0, new byte[16])));
        assertThrows(UnsupportedDdsFormatException.class,
                () -> DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT4", DDPF_FOURCC, 0, 0, 0, 0, 0, new byte[16])));
        // The straight-alpha siblings DXT3/DXT5 still decode.
        DecodedImage dxt3 = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT3", DDPF_FOURCC, 0, 0, 0, 0, 0,
                DdsFixtures.bc2Block(new int[] {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}, 0xFFFF, 0x0000, 0)));
        assertEquals(4 * 4 * 4, dxt3.copyPixelData().length);
        DecodedImage dxt5 = DdsDecoder.decode(DdsFixtures.legacyDds(4, 4, "DXT5", DDPF_FOURCC, 0, 0, 0, 0, 0,
                DdsFixtures.bc3Block(0xFF, 0x00, 0L, 0xFFFF, 0x0000, 0)));
        assertEquals(4 * 4 * 4, dxt5.copyPixelData().length);
    }

    // ------------------------------------------------------------------
    // Unsupported / malformed
    // ------------------------------------------------------------------

    @Test
    void rejectsUnsupportedFourCc() {
        byte[] dds = DdsFixtures.legacyDds(4, 4, "ATI2", DDPF_FOURCC, 0, 0, 0, 0, 0, new byte[8]);
        assertThrows(UnsupportedDdsFormatException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsUnsupportedDxgiFormat() {
        byte[] dds = DdsFixtures.dx10Dds(4, 4, 83, new byte[8]); // BC5_UNORM
        assertThrows(UnsupportedDdsFormatException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsPaletted() {
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_PAL8, 8, 0, 0, 0, 0, new byte[1]);
        assertThrows(UnsupportedDdsFormatException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsUnknown32BitLayout() {
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", DDPF_RGB, 32,
                0x3ff00000, 0x000ffc00, 0x000003ff, 0xc0000000, new byte[4]); // 10:10:10:2
        assertThrows(UnsupportedDdsFormatException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsCubemap() {
        byte[] dds = DdsFixtures.cubemapDds(4, 4, new byte[8]);
        assertThrows(UnsupportedDdsFormatException.class, () -> DdsDecoder.decode(dds));
    }

    @Test
    void rejectsTruncatedSurface() {
        byte[] block = DdsFixtures.bc1Block(0xFFFF, 0x0000, 0);
        byte[] dds = DdsFixtures.legacyDds(4, 4, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0, block);
        byte[] truncated = new byte[dds.length - 2];
        System.arraycopy(dds, 0, truncated, 0, truncated.length);
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(truncated));
    }

    @Test
    void rejectsBadMagic() {
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(DdsFixtures.badMagic(8)));
    }

    @Test
    void rejectsNullAndShortInput() {
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(null));
        assertThrows(DdsDecodeException.class, () -> DdsDecoder.decode(new byte[16]));
    }

    @Test
    void copyPixelDataReturnsSnapshot() throws Exception {
        byte[] surface = {0x10, 0x20, 0x30, 0x40};
        DecodedImage img = DdsDecoder.decode(DdsFixtures.legacyDds(1, 1, "", DDPF_RGB | DDPF_ALPHAPIXELS, 32,
                0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000, surface));
        byte[] copy = img.copyPixelData();
        assertEquals(4, copy.length);
        assertArrayEquals(new byte[] {0x30, 0x20, 0x10, 0x40}, copy);
        copy[0] = 0x00; // must not affect the image
        assertEquals(0x40302010, img.getPixelRgba(0, 0));
    }

    @Test
    void rejectsInvalidImageDimensions() {
        assertThrows(IllegalArgumentException.class, () -> DecodedImage.of(0, 1, new byte[4], false));
        assertThrows(IllegalArgumentException.class, () -> DecodedImage.of(1, 1, new byte[3], false));
    }

    @Test
    void decodeAllModesSmokeNoExceptions() throws Exception {
        // Ensure every supported legacy format at least parses and yields a sane-size image.
        byte[] bc1 = DdsFixtures.legacyDds(4, 4, "DXT1", DDPF_FOURCC, 0, 0, 0, 0, 0,
                DdsFixtures.bc1Block(0xFFFF, 0x0000, 0));
        assertEquals(4 * 4 * 4, DdsDecoder.decode(bc1).copyPixelData().length);

        byte[] bc2 = DdsFixtures.legacyDds(4, 4, "DXT3", DDPF_FOURCC, 0, 0, 0, 0, 0,
                DdsFixtures.bc2Block(new int[] {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}, 0xFFFF, 0x0000, 0));
        assertEquals(4 * 4 * 4, DdsDecoder.decode(bc2).copyPixelData().length);
        try {
            byte[] bc7 = DdsFixtures.dx10Dds(4, 4, 98, DdsFixtures.bc7Mode6Block(new int[16]));
            assertEquals(4 * 4 * 4, DdsDecoder.decode(bc7).copyPixelData().length);
        } catch (Throwable t) {
            fail("BC7 smoke test failed: " + t);
        }
    }
}
