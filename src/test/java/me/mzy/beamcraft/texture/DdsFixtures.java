package me.mzy.beamcraft.texture;

/**
 * Deterministic in-memory DDS builders for the unit tests. No real files are
 * touched; every fixture is produced entirely in memory and handed to
 * {@link DdsDecoder} as a {@code byte[]}.
 *
 * <p>Bit packing for BC7 mirrors the decoder's LSB-first {@code BitStream}:
 * the first bit written is bit 0 of byte 0, exactly as DDS stores a 128-bit
 * BC7 block. The hand-computed expected pixels in the tests therefore pin the
 * spec behaviour, not just internal consistency.
 */
final class DdsFixtures {

    private static final int DDSD_CAPS = 0x1;
    private static final int DDSD_HEIGHT = 0x2;
    private static final int DDSD_WIDTH = 0x4;
    private static final int DDSD_PITCH = 0x8;
    private static final int DDSD_PIXELFORMAT = 0x1000;
    private static final int DDSD_LINEARSIZE = 0x80000;
    private static final int DDSCAPS_TEXTURE = 0x1000;
    private static final int DDSCAPS2_CUBEMAP = 0x200;
    private static final int DDSCAPS2_VOLUME = 0x200000;

    private DdsFixtures() {
    }

    static int fourcc(String s) {
        return s.charAt(0) | (s.charAt(1) << 8) | (s.charAt(2) << 16) | (s.charAt(3) << 24);
    }

    // ------------------------------------------------------------------
    // Whole-file builders
    // ------------------------------------------------------------------

    /**
     * Legacy (pre-DX10) DDS file. {@code fourCC} may be a compressed FourCC
     * (e.g. {@code "DXT1"}) or 0 for mask-described uncompressed formats.
     */
    static byte[] legacyDds(int width, int height, String fourCC, int pfFlags,
                            int bitCount, int rMask, int gMask, int bMask, int aMask,
                            byte[] surface) {
        boolean compressed = fourCC != null && !fourCC.isEmpty();
        int surfaceSize = surface.length;
        int pitch = compressed ? surfaceSize : width * (bitCount / 8);
        int flags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH | DDSD_PIXELFORMAT
                | (compressed ? DDSD_LINEARSIZE : DDSD_PITCH);

        byte[] dds = new byte[128 + surfaceSize];
        writeInt(dds, 0, 0x20534444); // "DDS "
        writeInt(dds, 4, 124);
        writeInt(dds, 8, flags);
        writeInt(dds, 12, height);
        writeInt(dds, 16, width);
        writeInt(dds, 20, pitch);
        writeInt(dds, 24, 1); // depth
        writeInt(dds, 76, 32);
        writeInt(dds, 80, pfFlags);
        writeInt(dds, 84, compressed ? fourcc(fourCC) : 0);
        writeInt(dds, 88, bitCount);
        writeInt(dds, 92, rMask);
        writeInt(dds, 96, gMask);
        writeInt(dds, 100, bMask);
        writeInt(dds, 104, aMask);
        writeInt(dds, 108, DDSCAPS_TEXTURE);
        writeInt(dds, 112, 0);
        System.arraycopy(surface, 0, dds, 128, surfaceSize);
        return dds;
    }

    /** DX10 DDS file (DXGI format). Assumes a 2D texture, array size 1. */
    static byte[] dx10Dds(int width, int height, int dxgiFormat, byte[] surface) {
        return dx10DdsRaw(width, height, dxgiFormat, 3 /* D3D10_RESOURCE_DIMENSION_TEXTURE2D */, 0, 1, surface);
    }

    /**
     * DX10 DDS file with an explicit resource dimension, misc flag and array
     * size (for rejection tests). {@code width}/{@code height} may be hostile
     * header values; only a {@code 148 + surface.length} byte file is built.
     */
    static byte[] dx10DdsRaw(int width, int height, int dxgiFormat, int resourceDimension,
                             int miscFlag, int arraySize, byte[] surface) {
        byte[] dds = new byte[148 + surface.length];
        writeInt(dds, 0, 0x20534444);
        writeInt(dds, 4, 124);
        writeInt(dds, 8, DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH | DDSD_PIXELFORMAT | DDSD_LINEARSIZE);
        writeInt(dds, 12, height);
        writeInt(dds, 16, width);
        writeInt(dds, 20, surface.length);
        writeInt(dds, 24, 1);
        writeInt(dds, 76, 32);
        writeInt(dds, 80, 0x4); // DDPF_FOURCC
        writeInt(dds, 84, fourcc("DX10"));
        writeInt(dds, 88, 0);
        writeInt(dds, 108, DDSCAPS_TEXTURE);
        writeInt(dds, 128, dxgiFormat);
        writeInt(dds, 132, resourceDimension);
        writeInt(dds, 136, miscFlag);
        writeInt(dds, 140, arraySize);
        writeInt(dds, 144, 0); // miscFlags2
        System.arraycopy(surface, 0, dds, 148, surface.length);
        return dds;
    }

    /** DDS file with the cubemap capability bit set (should be rejected). */
    static byte[] cubemapDds(int width, int height, byte[] surface) {
        return legacyDdsWithCaps2(width, height, "DXT1", 0x4, 0, 0, 0, 0, 0, DDSCAPS2_CUBEMAP, surface);
    }

    /** DDS file with a caller-chosen caps2 (e.g. {@code DDSCAPS2_VOLUME}). */
    static byte[] legacyDdsWithCaps2(int width, int height, String fourCC, int pfFlags, int bitCount,
                                     int rMask, int gMask, int bMask, int aMask, int caps2, byte[] surface) {
        byte[] dds = legacyDds(width, height, fourCC, pfFlags, bitCount, rMask, gMask, bMask, aMask, surface);
        writeInt(dds, 112, caps2);
        return dds;
    }

    /** DDS file whose first four bytes are not the DDS magic. */
    static byte[] badMagic(int length) {
        byte[] dds = new byte[Math.max(length, 4)];
        dds[0] = 'N';
        dds[1] = 'O';
        dds[2] = 'P';
        dds[3] = 'E';
        return dds;
    }

    // ------------------------------------------------------------------
    // BC block packers
    // ------------------------------------------------------------------

    static byte[] bc1Block(int c0, int c1, int indices) {
        byte[] b = new byte[8];
        writeShort(b, 0, c0);
        writeShort(b, 2, c1);
        writeInt(b, 4, indices);
        return b;
    }

    static byte[] bc2Block(int[] rowAlphaWords, int c0, int c1, int colorIndices) {
        byte[] b = new byte[16];
        for (int y = 0; y < 4; y++) {
            writeShort(b, y * 2, rowAlphaWords[y]);
        }
        writeShort(b, 8, c0);
        writeShort(b, 10, c1);
        writeInt(b, 12, colorIndices);
        return b;
    }

    static byte[] bc3Block(int a0, int a1, long alphaIndices, int c0, int c1, int colorIndices) {
        byte[] b = new byte[16];
        b[0] = (byte) a0;
        b[1] = (byte) a1;
        for (int i = 0; i < 6; i++) {
            b[2 + i] = (byte) ((alphaIndices >>> (8 * i)) & 0xFF);
        }
        writeShort(b, 8, c0);
        writeShort(b, 10, c1);
        writeInt(b, 12, colorIndices);
        return b;
    }

    static byte[] bc4Block(int v0, int v1, long indices) {
        byte[] b = new byte[8];
        b[0] = (byte) v0;
        b[1] = (byte) v1;
        for (int i = 0; i < 6; i++) {
            b[2 + i] = (byte) ((indices >>> (8 * i)) & 0xFF);
        }
        return b;
    }

    /**
     * Packs 16 3-bit indices (BC3/BC4) in row-major, LSB-first order:
     * texel (y,x) occupies bits {@code 3*(y*4+x)}.
     */
    static long packIndex3(int[] indices16) {
        long v = 0;
        for (int i = 0; i < 16; i++) {
            v |= (long) (indices16[i] & 0x07) << (3 * i);
        }
        return v;
    }

    // ------------------------------------------------------------------
    // BC7 bit writer
    // ------------------------------------------------------------------

    /** LSB-first 128-bit writer matching the decoder's BitStream. */
    static final class BitWriter {
        private long low;
        private long high;
        private int pos;

        void write(int value, int bits) {
            for (int i = 0; i < bits; i++) {
                long bit = ((value >>> i) & 1L);
                if (pos < 64) {
                    low |= bit << pos;
                } else {
                    high |= bit << (pos - 64);
                }
                pos++;
            }
        }

        void writeBit(int bit) {
            write(bit, 1);
        }

        byte[] toBlock() {
            byte[] b = new byte[16];
            for (int i = 0; i < 8; i++) {
                b[i] = (byte) ((low >>> (8 * i)) & 0xFF);
            }
            for (int i = 0; i < 8; i++) {
                b[8 + i] = (byte) ((high >>> (8 * i)) & 0xFF);
            }
            return b;
        }
    }

    /**
     * BC7 block in mode 6: single partition, 7-bit RGB + 7-bit alpha
     * endpoints, one P-bit per endpoint, 4-bit indices. Endpoint 0 = white
     * (P-bit 1) with alpha 255; endpoint 1 = black with alpha 0. Returns the
     * 16-byte block; callers then select per-texel indices.
     */
    static byte[] bc7Mode6Block(int[] indices16) {
        BitWriter w = new BitWriter();
        for (int i = 0; i < 6; i++) { // mode 6 = six leading zeros then 1
            w.writeBit(0);
        }
        w.writeBit(1);
        // RGB endpoints: channel-major, 7 bits each.
        for (int channel = 0; channel < 3; channel++) {
            w.write(127, 7); // endpoint 0
            w.write(0, 7);   // endpoint 1
        }
        // Alpha endpoints.
        w.write(127, 7);
        w.write(0, 7);
        // P-bits: endpoint0 keeps 127<<1|1 == 255; endpoint1 keeps 0<<1|0 == 0.
        w.writeBit(1);
        w.writeBit(0);
        // Indices: texel (0,0) is the fix-up index (one fewer bit).
        w.write(indices16[0] & 0x07, 3);
        for (int i = 1; i < 16; i++) {
            w.write(indices16[i] & 0x0F, 4);
        }
        return w.toBlock();
    }

    /**
     * BC7 block in mode 0: three partitions, 4-bit RGB endpoints, one P-bit
     * per endpoint (no alpha; opaque). Partition index 0, all six endpoints
     * set to the same value {@code rgb4} with P-bit 1 so every texel maps to
     * the same colour regardless of its subset.
     */
    static byte[] bc7Mode0Block(int rgb4) {
        BitWriter w = new BitWriter();
        w.writeBit(0); // mode 0
        w.write(0, 4); // partition = 0
        for (int channel = 0; channel < 3; channel++) {
            for (int e = 0; e < 6; e++) {
                w.write(rgb4, 4);
            }
        }
        for (int e = 0; e < 6; e++) {
            w.writeBit(1);
        }
        // Indices: fix-up per subset is one bit shorter (texels 0,1,2 for 3 subsets).
        int[] fixups = {0, 1, 2};
        for (int i = 0; i < 16; i++) {
            boolean fixup = false;
            for (int f : fixups) {
                if (f == i) {
                    fixup = true;
                    break;
                }
            }
            w.write(0, fixup ? 2 : 3);
        }
        return w.toBlock();
    }

    /** BC7 block with all-zero mode bits (reserved) -> transparent black. */
    static byte[] bc7ReservedBlock() {
        byte[] b = new byte[16];
        for (int i = 0; i < 9; i++) {
            b[i / 8] |= (byte) (0 << (i % 8)); // 9+ leading zeros -> invalid mode
        }
        // Explicitly zero all 16 bytes (already zero).
        return b;
    }

    // ------------------------------------------------------------------
    // Raw little-endian helpers
    // ------------------------------------------------------------------

    static void writeInt(byte[] dst, int off, int v) {
        dst[off] = (byte) v;
        dst[off + 1] = (byte) (v >>> 8);
        dst[off + 2] = (byte) (v >>> 16);
        dst[off + 3] = (byte) (v >>> 24);
    }

    static void writeShort(byte[] dst, int off, int v) {
        dst[off] = (byte) v;
        dst[off + 1] = (byte) (v >>> 8);
    }
}
