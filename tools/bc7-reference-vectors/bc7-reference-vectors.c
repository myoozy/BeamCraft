/*
 * BC7 reference-vector generator for BeamCraft's DdsDecoderTest.
 *
 * Builds one BC7 block per test vector (modes 0-7, all rotations / index
 * selections for the rotation modes), decodes each with the independent
 * bcdec reference decoder, and emits a Java source file containing the
 * literal block bytes and the bcdec-derived expected RGBA pixels.
 *
 * Provenance: expected pixels are produced by bcdec v0.97 (Sergii "iOrange"
 * Kudlai, MIT/Unlicense; https://github.com/iOrange/bcdec), a well-known
 * independent implementation of the D3D11/Khronos BPTC spec. The partition
 * tables embedded below are transcribed from bcdec's source (which in turn
 * follows the ARB_texture_compression_bptc extension tables, NOT the
 * Khronos spec text which is known to contain errors).
 *
 * The Java decoder under test is NOT used here and the expected values are
 * not derived from it. This is a cross-validation, not a self-check.
 */
#include <stdio.h>
#include <string.h>

#define BCDEC_IMPLEMENTATION
#include "bcdec.h"

/* ---- partition tables extracted from bcdec v0.97 ------------------------ */
static const unsigned char PART2[64][16] = {
#include "part2.inc"
};
static const unsigned char PART3[64][16] = {
#include "part3.inc"
};

/* ---- LSB-first 128-bit bit writer (same layout DDS stores a BC7 block) -- */
typedef struct { unsigned long long low, high; int pos; } BW;

static void bw_init(BW *w) { w->low = 0; w->high = 0; w->pos = 0; }

static void bw_write(BW *w, unsigned int value, int bits) {
    for (int i = 0; i < bits; i++) {
        unsigned long long bit = (value >> i) & 1ULL;
        if (w->pos < 64) w->low |= bit << w->pos;
        else             w->high |= bit << (w->pos - 64);
        w->pos++;
    }
}

static void bw_bit(BW *w, int b) { bw_write(w, b, 1); }

static void bw_block(BW *w, unsigned char out[16]) {
    for (int i = 0; i < 8; i++) out[i]     = (unsigned char)((w->low >> (8 * i)) & 0xFF);
    for (int i = 0; i < 8; i++) out[8 + i] = (unsigned char)((w->high >> (8 * i)) & 0xFF);
}

/* partition-table value (0x80 = fix-up) for texel (y,x) */
static int partition_value(int numPartitions, int partition, int y, int x) {
    if (numPartitions == 1) return (y == 0 && x == 0) ? 0x80 : 0;
    if (numPartitions == 2) return PART2[partition][y * 4 + x];
    return PART3[partition][y * 4 + x];
}

static void write_indices(BW *w, int numPartitions, int partition, int indexBits, const int *idx) {
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) {
            int v = partition_value(numPartitions, partition, y, x);
            bw_write(w, idx[y * 4 + x], indexBits - ((v & 0x80) ? 1 : 0));
        }
}

/* secondary index stream (modes 4/5): fix-up texel is always (0,0) */
static void write_indices2(BW *w, int indexBits2, const int *idx) {
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) {
            int fixup = (y == 0 && x == 0);
            bw_write(w, idx[y * 4 + x], indexBits2 - (fixup ? 1 : 0));
        }
}

/* ---- per-mode block builders ------------------------------------------- */

/* mode 0: 3 subsets, 4-bit RGB, p-bit per endpoint, 3-bit indices */
static void mode0(unsigned char out[16], int part, const int ep[6][3], const int *pbits, const int *idx) {
    BW w; bw_init(&w);
    bw_bit(&w, 0);
    bw_write(&w, part, 4);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 6; e++) bw_write(&w, ep[e][c], 4);
    for (int e = 0; e < 6; e++) bw_bit(&w, pbits[e]);
    write_indices(&w, 3, part, 3, idx);
    bw_block(&w, out);
}

/* mode 1: 2 subsets, 6-bit RGB, 2 shared p-bits, 3-bit indices */
static void mode1(unsigned char out[16], int part, const int ep[4][3], int p0, int p1, const int *idx) {
    BW w; bw_init(&w);
    bw_bit(&w, 0); bw_bit(&w, 1);
    bw_write(&w, part, 6);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 4; e++) bw_write(&w, ep[e][c], 6);
    bw_bit(&w, p0); bw_bit(&w, p1);
    write_indices(&w, 2, part, 3, idx);
    bw_block(&w, out);
}

/* mode 2: 3 subsets, 5-bit RGB, no p-bits, 2-bit indices */
static void mode2(unsigned char out[16], int part, const int ep[6][3], const int *idx) {
    BW w; bw_init(&w);
    bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 1);
    bw_write(&w, part, 6);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 6; e++) bw_write(&w, ep[e][c], 5);
    write_indices(&w, 3, part, 2, idx);
    bw_block(&w, out);
}

/* mode 3: 2 subsets, 7-bit RGB, p-bit per endpoint, 2-bit indices */
static void mode3(unsigned char out[16], int part, const int ep[4][3], const int *pbits, const int *idx) {
    BW w; bw_init(&w);
    bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 1);
    bw_write(&w, part, 6);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 4; e++) bw_write(&w, ep[e][c], 7);
    for (int e = 0; e < 4; e++) bw_bit(&w, pbits[e]);
    write_indices(&w, 2, part, 2, idx);
    bw_block(&w, out);
}

/* mode 4: 1 subset, 5-bit RGB + 6-bit alpha, no p-bits, rotation + index sel,
 * 2-bit primary / 3-bit secondary indices */
static void mode4(unsigned char out[16], int rot, int isb, const int ep[2][3], const int aep[2],
                  const int *pri, const int *sec) {
    BW w; bw_init(&w);
    bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 0); bw_bit(&w, 1);
    bw_write(&w, rot, 2);
    bw_bit(&w, isb);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 2; e++) bw_write(&w, ep[e][c], 5);
    for (int e = 0; e < 2; e++) bw_write(&w, aep[e], 6);
    write_indices(&w, 1, 0, 2, pri);
    write_indices2(&w, 3, sec);
    bw_block(&w, out);
}

/* mode 5: 1 subset, 7-bit RGB + 8-bit alpha, no p-bits, rotation,
 * 2-bit primary / 2-bit secondary indices */
static void mode5(unsigned char out[16], int rot, const int ep[2][3], const int aep[2],
                  const int *pri, const int *sec) {
    BW w; bw_init(&w);
    for (int i = 0; i < 5; i++) bw_bit(&w, 0);
    bw_bit(&w, 1);
    bw_write(&w, rot, 2);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 2; e++) bw_write(&w, ep[e][c], 7);
    for (int e = 0; e < 2; e++) bw_write(&w, aep[e], 8);
    write_indices(&w, 1, 0, 2, pri);
    write_indices2(&w, 2, sec);
    bw_block(&w, out);
}

/* mode 6: 1 subset, 7-bit RGB + 7-bit alpha, p-bit per endpoint, 4-bit indices */
static void mode6(unsigned char out[16], const int ep[2][3], const int aep[2], const int *pbits, const int *idx) {
    BW w; bw_init(&w);
    for (int i = 0; i < 6; i++) bw_bit(&w, 0);
    bw_bit(&w, 1);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 2; e++) bw_write(&w, ep[e][c], 7);
    for (int e = 0; e < 2; e++) bw_write(&w, aep[e], 7);
    for (int e = 0; e < 2; e++) bw_bit(&w, pbits[e]);
    write_indices(&w, 1, 0, 4, idx);
    bw_block(&w, out);
}

/* mode 7: 2 subsets, 5-bit RGB + 5-bit alpha, p-bit per endpoint, 2-bit indices */
static void mode7(unsigned char out[16], int part, const int ep[4][3], const int aep[4],
                  const int *pbits, const int *idx) {
    BW w; bw_init(&w);
    for (int i = 0; i < 7; i++) bw_bit(&w, 0);
    bw_bit(&w, 1);
    bw_write(&w, part, 6);
    for (int c = 0; c < 3; c++) for (int e = 0; e < 4; e++) bw_write(&w, ep[e][c], 5);
    for (int e = 0; e < 4; e++) bw_write(&w, aep[e], 5);
    for (int e = 0; e < 4; e++) bw_bit(&w, pbits[e]);
    write_indices(&w, 2, part, 2, idx);
    bw_block(&w, out);
}

/* ---- vector table ------------------------------------------------------- */

typedef struct {
    const char *name;
    int mode;
    void (*build)(unsigned char *out);
} Vec;

#define VEC(name_, mode_, fn_) { name_, mode_, fn_ }

static const int IDX_CHECKER[16] = { 0,1,2,3, 0,1,2,3, 0,1,2,3, 0,1,2,3 };
static const int IDX_UP4[16]     = { 0,1,2,3, 4,5,6,7, 7,6,5,4, 3,2,1,0 };
static const int IDX_ALT[16]     = { 0,1,2,3, 4,5,6,7, 0,1,2,3, 4,5,6,7 };
static const int SEC4_ALT[16]    = { 0,2,4,6, 1,3,5,7, 0,2,4,6, 1,3,5,7 };
static const int SEC5_ALT[16]    = { 0,3,1,2, 3,0,2,1, 1,2,0,3, 2,1,3,0 };

static void b_mode0(unsigned char *o) {
    const int ep[6][3] = { {15,0,0}, {0,15,0}, {0,0,15}, {15,15,15}, {8,8,8}, {0,0,0} };
    const int pb[6] = { 0,0,0,0,0,0 };
    mode0(o, 0, ep, pb, IDX_UP4);
}
static void b_mode1(unsigned char *o) {
    const int ep[4][3] = { {63,0,0}, {0,63,0}, {0,0,63}, {63,63,0} };
    mode1(o, 0, ep, 0, 1, IDX_ALT);
}
static void b_mode2(unsigned char *o) {
    const int ep[6][3] = { {31,0,0}, {0,31,0}, {0,0,31}, {31,31,0}, {0,31,31}, {31,0,31} };
    mode2(o, 0, ep, IDX_CHECKER);
}
static void b_mode3(unsigned char *o) {
    const int ep[4][3] = { {127,0,0}, {0,127,0}, {0,0,127}, {127,127,0} };
    const int pb[4] = { 1,0,0,1 };
    mode3(o, 0, ep, pb, IDX_CHECKER);
}
static void b_mode4_r0i0(unsigned char *o) {
    const int ep[2][3] = { {31,0,0}, {0,31,0} };
    const int aep[2] = { 0, 31 };
    mode4(o, 0, 0, ep, aep, IDX_CHECKER, SEC4_ALT);
}
static void b_mode4_r1i0(unsigned char *o) {
    const int ep[2][3] = { {31,0,0}, {0,31,0} };
    const int aep[2] = { 0, 31 };
    mode4(o, 1, 0, ep, aep, IDX_CHECKER, SEC4_ALT);
}
static void b_mode4_r2i1(unsigned char *o) {
    const int ep[2][3] = { {31,0,0}, {0,31,0} };
    const int aep[2] = { 0, 31 };
    mode4(o, 2, 1, ep, aep, IDX_CHECKER, SEC4_ALT);
}
static void b_mode4_r3i1(unsigned char *o) {
    const int ep[2][3] = { {31,0,0}, {0,31,0} };
    const int aep[2] = { 0, 31 };
    mode4(o, 3, 1, ep, aep, IDX_CHECKER, SEC4_ALT);
}
static void b_mode5_r0(unsigned char *o) {
    const int ep[2][3] = { {127,0,0}, {0,127,0} };
    const int aep[2] = { 0, 255 };
    mode5(o, 0, ep, aep, IDX_CHECKER, SEC5_ALT);
}
static void b_mode5_r1(unsigned char *o) {
    const int ep[2][3] = { {127,0,0}, {0,127,0} };
    const int aep[2] = { 0, 255 };
    mode5(o, 1, ep, aep, IDX_CHECKER, SEC5_ALT);
}
static void b_mode5_r2(unsigned char *o) {
    const int ep[2][3] = { {127,0,0}, {0,127,0} };
    const int aep[2] = { 0, 255 };
    mode5(o, 2, ep, aep, IDX_CHECKER, SEC5_ALT);
}
static void b_mode5_r3(unsigned char *o) {
    const int ep[2][3] = { {127,0,0}, {0,127,0} };
    const int aep[2] = { 0, 255 };
    mode5(o, 3, ep, aep, IDX_CHECKER, SEC5_ALT);
}
static void b_mode6(unsigned char *o) {
    /* matches DdsFixtures.bc7Mode6Block(indices {0,15,7,8,0..0}) */
    const int ep[2][3] = { {127,127,127}, {0,0,0} };
    const int aep[2] = { 127, 0 };
    const int pb[2] = { 1, 0 };
    const int idx[16] = { 0,15,7,8, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
    mode6(o, ep, aep, pb, idx);
}
static void b_mode7(unsigned char *o) {
    const int ep[4][3] = { {31,0,0}, {0,31,0}, {0,0,31}, {31,31,0} };
    const int aep[4] = { 31, 0, 0, 31 };
    const int pb[4] = { 1,0,0,1 };
    mode7(o, 0, ep, aep, pb, IDX_CHECKER);
}

/* Non-zero partition variants, to exercise the partition tables (and their
 * fix-up texels) at shapes other than partition 0. */
static void b_mode0_p5(unsigned char *o) {
    const int ep[6][3] = { {15,0,0}, {0,15,0}, {0,0,15}, {15,15,15}, {8,8,8}, {0,0,0} };
    const int pb[6] = { 0,0,0,0,0,0 };
    mode0(o, 5, ep, pb, IDX_UP4);
}
static void b_mode1_p12(unsigned char *o) {
    const int ep[4][3] = { {63,0,0}, {0,63,0}, {0,0,63}, {63,63,0} };
    mode1(o, 12, ep, 0, 1, IDX_ALT);
}
static void b_mode2_p33(unsigned char *o) {
    const int ep[6][3] = { {31,0,0}, {0,31,0}, {0,0,31}, {31,31,0}, {0,31,31}, {31,0,31} };
    mode2(o, 33, ep, IDX_CHECKER);
}
static void b_mode3_p60(unsigned char *o) {
    const int ep[4][3] = { {127,0,0}, {0,127,0}, {0,0,127}, {127,127,0} };
    const int pb[4] = { 1,0,0,1 };
    mode3(o, 60, ep, pb, IDX_CHECKER);
}
static void b_mode7_p63(unsigned char *o) {
    const int ep[4][3] = { {31,0,0}, {0,31,0}, {0,0,31}, {31,31,0} };
    const int aep[4] = { 31, 0, 0, 31 };
    const int pb[4] = { 1,0,0,1 };
    mode7(o, 63, ep, aep, pb, IDX_CHECKER);
}

static const Vec VECS[] = {
    VEC("mode0_part0",    0, b_mode0),
    VEC("mode1_part0",    1, b_mode1),
    VEC("mode2_part0",    2, b_mode2),
    VEC("mode3_part0",    3, b_mode3),
    VEC("mode4_r0_i0",    4, b_mode4_r0i0),
    VEC("mode4_r1_i0",    4, b_mode4_r1i0),
    VEC("mode4_r2_i1",    4, b_mode4_r2i1),
    VEC("mode4_r3_i1",    4, b_mode4_r3i1),
    VEC("mode5_r0",       5, b_mode5_r0),
    VEC("mode5_r1",       5, b_mode5_r1),
    VEC("mode5_r2",       5, b_mode5_r2),
    VEC("mode5_r3",       5, b_mode5_r3),
    VEC("mode6",          6, b_mode6),
    VEC("mode7_part0",    7, b_mode7),
    VEC("mode0_part5",    0, b_mode0_p5),
    VEC("mode1_part12",   1, b_mode1_p12),
    VEC("mode2_part33",   2, b_mode2_p33),
    VEC("mode3_part60",   3, b_mode3_p60),
    VEC("mode7_part63",   7, b_mode7_p63),
};

static void print_block_bytes(const unsigned char *b) {
    for (int i = 0; i < 16; i++) printf("(byte) %d,", b[i]);
}

int main(void) {
    int n = (int)(sizeof(VECS) / sizeof(VECS[0]));
    printf("static final byte[] MODE6_BLOCK = {");
    /* emit the mode-6 block first for byte-level comparison against the hand fixture */
    {
        unsigned char b[16];
        b_mode6(b);
        print_block_bytes(b);
        printf("};\n");
    }
    printf("static final ReferenceVector[] VECTORS = {\n");
    for (int v = 0; v < n; v++) {
        unsigned char block[16];
        unsigned char rgba[4 * 4 * 4];
        VECS[v].build(block);
        memset(rgba, 0, sizeof(rgba));
        bcdec_bc7(block, rgba, 4 * 4);

        printf("    new ReferenceVector(\"%s\", %d,\n        new byte[] {", VECS[v].name, VECS[v].mode);
        print_block_bytes(block);
        printf("},\n        new int[] {");
        for (int i = 0; i < 16; i++) {
            int r = rgba[i * 4 + 0], g = rgba[i * 4 + 1], b = rgba[i * 4 + 2], a = rgba[i * 4 + 3];
            printf("0x%02X%02X%02X%02X%s", (unsigned)a, (unsigned)r, (unsigned)g, (unsigned)b,
                   i < 15 ? "," : "");
        }
        printf("}),\n");
    }
    printf("};\n");
    return 0;
}
