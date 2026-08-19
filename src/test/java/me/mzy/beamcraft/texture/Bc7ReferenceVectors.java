package me.mzy.beamcraft.texture;

/**
 * Independent BC7 reference vectors used to cross-validate {@link DdsDecoder}.
 *
 * <p>Each vector is a literal 16-byte BC7 block plus the 16 RGBA pixels (packed
 * {@code 0xAARRGGBB}) that a reference decoder produces for it. The blocks cover
 * every mode 0-7, including all four rotations and both index-selection values
 * of the rotation modes (4/5). The expected pixels were <em>not</em> computed by
 * this decoder or by a translation of its algorithm; they are the output of the
 * independent bcdec reference implementation (Sergii "iOrange" Kudlai, dual
 * MIT/Unlicense, https://github.com/iOrange/bcdec, v0.97), which implements the
 * D3D11/Khronos BPTC functional spec.
 *
 * <p>Provenance: {@code tools/bc7-reference-vectors/bc7-reference-vectors.c}
 * builds each block and decodes it with the vendored {@code bcdec.h}; the
 * emitted values are pasted here verbatim (see that directory's README for the
 * exact regeneration command). The partition tables are the
 * ARB_texture_compression_bptc tables, which the Khronos spec text is known to
 * get wrong. The mode-6 vector is byte-identical to the hand-computed
 * {@link DdsFixtures#bc7Mode6Block} fixture and its expected pixels equal the
 * hand-derived values asserted by {@link DdsDecoderTest#decodesBc7Mode6}, which
 * ties the reference generator to independent spec-derived expectations.
 */
final class Bc7ReferenceVectors {

    private Bc7ReferenceVectors() {
    }

    static final class ReferenceVector {
        final String name;
        final int mode;
        final byte[] block;
        final int[] expected;

        ReferenceVector(String name, int mode, byte[] block, int[] expected) {
            this.name = name;
            this.mode = mode;
            this.block = block;
            this.expected = expected;
        }
    }

    /** The mode-6 block bytes (matches {@link DdsFixtures#bc7Mode6Block}). */
static final byte[] MODE6_BLOCK = {(byte) 192,(byte) 63,(byte) 224,(byte) 15,(byte) 248,(byte) 3,(byte) 254,(byte) 128,(byte) 240,(byte) 135,(byte) 0,(byte) 0,(byte) 0,(byte) 0,(byte) 0,(byte) 0,};
static final ReferenceVector[] VECTORS = {
    new ReferenceVector("mode0_part0", 0,
        new byte[] {(byte) 224,(byte) 1,(byte) 30,(byte) 1,(byte) 30,(byte) 30,(byte) 1,(byte) 224,(byte) 31,(byte) 1,(byte) 32,(byte) 154,(byte) 245,(byte) 239,(byte) 114,(byte) 10,},
        new int[] {0x9A294F00,0x4678E151,0xC30208F8,0x6F519AF8,0xC30208F8,0xC30208F8,0xC30208A7,0xC30208F8,0xC30208A7,0xC3020800,0x9A294FF8,0xC3020851,0x9A294FA7,0x4678E1A7,0xC3020800,0x4678E100}),
    new ReferenceVector("mode1_part0", 1,
        new byte[] {(byte) 2,(byte) 63,(byte) 0,(byte) 252,(byte) 192,(byte) 15,(byte) 252,(byte) 0,(byte) 240,(byte) 3,(byte) 18,(byte) 141,(byte) 245,(byte) 17,(byte) 141,(byte) 245,},
        new int[] {0xFFFD0000,0xFFD92400,0xFF4949B8,0xFF6D6D94,0xFF6B9200,0xFF47B600,0xFFDBDB26,0xFFFFFF02,0xFFFD0000,0xFFD92400,0xFF4949B8,0xFF6D6D94,0xFF6B9200,0xFF47B600,0xFFDBDB26,0xFF6D6D94}),
    new ReferenceVector("mode2_part0", 2,
        new byte[] {(byte) 4,(byte) 62,(byte) 0,(byte) 31,(byte) 124,(byte) 240,(byte) 193,(byte) 255,(byte) 0,(byte) 128,(byte) 15,(byte) 254,(byte) 151,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0000,0xFFAB5400,0xFFABAB54,0xFF5454AB,0xFFFF0000,0xFFAB5400,0xFFABAB54,0xFFFFFF00,0xFFFF0000,0xFF54ABFF,0xFFAB54FF,0xFFFFFF00,0xFF00FFFF,0xFF54ABFF,0xFFAB54FF,0xFF54ABFF}),
    new ReferenceVector("mode3_part0", 3,
        new byte[] {(byte) 8,(byte) 252,(byte) 1,(byte) 128,(byte) 63,(byte) 224,(byte) 15,(byte) 248,(byte) 3,(byte) 0,(byte) 127,(byte) 64,(byte) 202,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFFFFFF01,0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFFFFFF01,0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFFFFFF01,0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFF5454AB}),
    new ReferenceVector("mode4_r0_i0", 4,
        new byte[] {(byte) 16,(byte) 31,(byte) 128,(byte) 15,(byte) 0,(byte) 240,(byte) 201,(byte) 201,(byte) 201,(byte) 201,(byte) 17,(byte) 157,(byte) 245,(byte) 16,(byte) 157,(byte) 245,},
        new int[] {0x00FF0000,0x23AB5400,0x4854AB00,0x6B00FF00,0x12FF0000,0x35AB5400,0x5A54AB00,0x7D00FF00,0x00FF0000,0x23AB5400,0x4854AB00,0x6B00FF00,0x12FF0000,0x35AB5400,0x5A54AB00,0x7D00FF00}),
    new ReferenceVector("mode4_r1_i0", 4,
        new byte[] {(byte) 48,(byte) 31,(byte) 128,(byte) 15,(byte) 0,(byte) 240,(byte) 201,(byte) 201,(byte) 201,(byte) 201,(byte) 17,(byte) 157,(byte) 245,(byte) 16,(byte) 157,(byte) 245,},
        new int[] {0xFF000000,0xAB235400,0x5448AB00,0x006BFF00,0xFF120000,0xAB355400,0x545AAB00,0x007DFF00,0xFF000000,0xAB235400,0x5448AB00,0x006BFF00,0xFF120000,0xAB355400,0x545AAB00,0x007DFF00}),
    new ReferenceVector("mode4_r2_i1", 4,
        new byte[] {(byte) 208,(byte) 31,(byte) 128,(byte) 15,(byte) 0,(byte) 240,(byte) 201,(byte) 201,(byte) 201,(byte) 201,(byte) 17,(byte) 157,(byte) 245,(byte) 16,(byte) 157,(byte) 245,},
        new int[] {0x00FF0000,0x48B72900,0x936C5400,0xDB247D00,0x24DB0000,0x6C932900,0xB7485400,0xFF007D00,0x00FF0000,0x48B72900,0x936C5400,0xDB247D00,0x24DB0000,0x6C932900,0xB7485400,0xFF007D00}),
    new ReferenceVector("mode4_r3_i1", 4,
        new byte[] {(byte) 240,(byte) 31,(byte) 128,(byte) 15,(byte) 0,(byte) 240,(byte) 201,(byte) 201,(byte) 201,(byte) 201,(byte) 17,(byte) 157,(byte) 245,(byte) 16,(byte) 157,(byte) 245,},
        new int[] {0x00FF0000,0x00B74829,0x006C9354,0x0024DB7D,0x00DB2400,0x00936C29,0x0048B754,0x0000FF7D,0x00FF0000,0x00B74829,0x006C9354,0x0024DB7D,0x00DB2400,0x00936C29,0x0048B754,0x0000FF7D}),
    new ReferenceVector("mode5_r0", 5,
        new byte[] {(byte) 32,(byte) 127,(byte) 0,(byte) 224,(byte) 15,(byte) 0,(byte) 0,(byte) 252,(byte) 203,(byte) 201,(byte) 201,(byte) 201,(byte) 157,(byte) 99,(byte) 201,(byte) 54,},
        new int[] {0x00FF0000,0xFFAB5400,0x5454AB00,0xAB00FF00,0xFFFF0000,0x00AB5400,0xAB54AB00,0x5400FF00,0x54FF0000,0xABAB5400,0x0054AB00,0xFF00FF00,0xABFF0000,0x54AB5400,0xFF54AB00,0x0000FF00}),
    new ReferenceVector("mode5_r1", 5,
        new byte[] {(byte) 96,(byte) 127,(byte) 0,(byte) 224,(byte) 15,(byte) 0,(byte) 0,(byte) 252,(byte) 203,(byte) 201,(byte) 201,(byte) 201,(byte) 157,(byte) 99,(byte) 201,(byte) 54,},
        new int[] {0xFF000000,0xABFF5400,0x5454AB00,0x00ABFF00,0xFFFF0000,0xAB005400,0x54ABAB00,0x0054FF00,0xFF540000,0xABAB5400,0x5400AB00,0x00FFFF00,0xFFAB0000,0xAB545400,0x54FFAB00,0x0000FF00}),
    new ReferenceVector("mode5_r2", 5,
        new byte[] {(byte) 160,(byte) 127,(byte) 0,(byte) 224,(byte) 15,(byte) 0,(byte) 0,(byte) 252,(byte) 203,(byte) 201,(byte) 201,(byte) 201,(byte) 157,(byte) 99,(byte) 201,(byte) 54,},
        new int[] {0x00FF0000,0x54ABFF00,0xAB545400,0xFF00AB00,0x00FFFF00,0x54AB0000,0xAB54AB00,0xFF005400,0x00FF5400,0x54ABAB00,0xAB540000,0xFF00FF00,0x00FFAB00,0x54AB5400,0xAB54FF00,0xFF000000}),
    new ReferenceVector("mode5_r3", 5,
        new byte[] {(byte) 224,(byte) 127,(byte) 0,(byte) 224,(byte) 15,(byte) 0,(byte) 0,(byte) 252,(byte) 203,(byte) 201,(byte) 201,(byte) 201,(byte) 157,(byte) 99,(byte) 201,(byte) 54,},
        new int[] {0x00FF0000,0x00AB54FF,0x0054AB54,0x0000FFAB,0x00FF00FF,0x00AB5400,0x0054ABAB,0x0000FF54,0x00FF0054,0x00AB54AB,0x0054AB00,0x0000FFFF,0x00FF00AB,0x00AB5454,0x0054ABFF,0x0000FF00}),
    new ReferenceVector("mode6", 6,
        new byte[] {(byte) 192,(byte) 63,(byte) 224,(byte) 15,(byte) 248,(byte) 3,(byte) 254,(byte) 128,(byte) 240,(byte) 135,(byte) 0,(byte) 0,(byte) 0,(byte) 0,(byte) 0,(byte) 0,},
        new int[] {0xFFFFFFFF,0x00000000,0x87878787,0x78787878,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF}),
    new ReferenceVector("mode7_part0", 7,
        new byte[] {(byte) 128,(byte) 192,(byte) 7,(byte) 224,(byte) 131,(byte) 15,(byte) 62,(byte) 0,(byte) 31,(byte) 124,(byte) 0,(byte) 126,(byte) 202,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0404,0xABAB5503,0xABABAB55,0xFFFFFF04,0xFFFF0404,0xABAB5503,0xABABAB55,0xFFFFFF04,0xFFFF0404,0xABAB5503,0xABABAB55,0xFFFFFF04,0xFFFF0404,0xABAB5503,0xABABAB55,0x545454AA}),
    new ReferenceVector("mode0_part5", 0,
        new byte[] {(byte) 234,(byte) 1,(byte) 30,(byte) 1,(byte) 30,(byte) 30,(byte) 1,(byte) 224,(byte) 31,(byte) 1,(byte) 32,(byte) 154,(byte) 245,(byte) 239,(byte) 114,(byte) 10,},
        new int[] {0xFF047881,0xFF313131,0xFF1D1D1D,0xFF0A0A0A,0xFF1D1D1D,0xFF131313,0xFF0A0A0A,0xFFE1E1FD,0xFF000000,0xFFC2D2EC,0xFFA3C3DA,0xFF84B5C9,0xFF61A4B5,0xFF4296A4,0xFF238792,0xFF444444}),
    new ReferenceVector("mode1_part12", 1,
        new byte[] {(byte) 50,(byte) 63,(byte) 0,(byte) 252,(byte) 192,(byte) 15,(byte) 252,(byte) 0,(byte) 240,(byte) 3,(byte) 18,(byte) 141,(byte) 245,(byte) 17,(byte) 141,(byte) 245,},
        new int[] {0xFFFD0000,0xFFD92400,0xFFB64700,0xFF6D6D94,0xFF6B9200,0xFFB8B849,0xFFDBDB26,0xFFFFFF02,0xFF0202FF,0xFF2626DB,0xFF4949B8,0xFF6D6D94,0xFF94946D,0xFFB8B849,0xFFDBDB26,0xFF6D6D94}),
    new ReferenceVector("mode2_part33", 2,
        new byte[] {(byte) 12,(byte) 63,(byte) 0,(byte) 31,(byte) 124,(byte) 240,(byte) 193,(byte) 255,(byte) 0,(byte) 128,(byte) 15,(byte) 254,(byte) 151,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0000,0xFF54ABFF,0xFFAB54FF,0xFF54ABFF,0xFFFF0000,0xFFAB5400,0xFFAB54FF,0xFFFF00FF,0xFFFF0000,0xFFAB5400,0xFFABAB54,0xFFFF00FF,0xFFFF0000,0xFFAB5400,0xFFABAB54,0xFF5454AB}),
    new ReferenceVector("mode3_part60", 3,
        new byte[] {(byte) 200,(byte) 255,(byte) 1,(byte) 128,(byte) 63,(byte) 224,(byte) 15,(byte) 248,(byte) 3,(byte) 0,(byte) 127,(byte) 64,(byte) 202,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0101,0xFFAB5401,0xFF54AB00,0xFF00FE00,0xFF0000FE,0xFF5454AB,0xFFABAB54,0xFFFFFF01,0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFFFFFF01,0xFFFF0101,0xFFAB5401,0xFFABAB54,0xFF5454AB}),
    new ReferenceVector("mode7_part63", 7,
        new byte[] {(byte) 128,(byte) 255,(byte) 7,(byte) 224,(byte) 131,(byte) 15,(byte) 62,(byte) 0,(byte) 31,(byte) 124,(byte) 0,(byte) 126,(byte) 202,(byte) 201,(byte) 201,(byte) 201,},
        new int[] {0xFFFF0404,0x545454AA,0x5454AA01,0x0000FB00,0xFFFF0404,0x545454AA,0x5454AA01,0x0000FB00,0xFFFF0404,0x545454AA,0xABABAB55,0xFFFFFF04,0xFFFF0404,0x545454AA,0xABABAB55,0x545454AA}),
};
}
