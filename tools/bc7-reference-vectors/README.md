# BC7 reference vectors for `DdsDecoderTest`

This directory generates the independent BC7 cross-validation vectors that
`src/test/java/me/mzy/beamcraft/texture/Bc7ReferenceVectors.java` embeds. The
Java decoder under test is **not** used anywhere here -- the expected pixels in
`Bc7ReferenceVectors.java` are the output of the standalone bcdec reference
decoder, so the unit test is a true cross-validation rather than a self-check.

## Provenance

- **Reference decoder**: `bcdec.h` v0.97, by Sergii "iOrange" Kudlai
  (dual MIT / Unlicense), vendored verbatim from
  <https://github.com/iOrange/bcdec>. It is an independent, widely used
  implementation of the D3D11/Khronos BCn spec.
- **Partition tables**: `part2.inc` / `part3.inc` are extracted verbatim from
  bcdec's `partition_sets` table, which follows the
  `ARB_texture_compression_bptc` extension tables (the Khronos BPTC spec *text*
  is known to contain errors -- bcdec documents this).
- **Block construction**: `bc7-reference-vectors.c` builds one block per vector
  (modes 0-7; all four rotations and both index-selection values for modes 4/5)
  using an LSB-first 128-bit writer that matches how DDS stores a BC7 block,
  then decodes it with `bcdec_bc7`.

## Regeneration

```bash
gcc -O2 -o gen bc7-reference-vectors.c && ./gen
```

The program prints `MODE6_BLOCK` (the mode-6 block bytes, which are
byte-identical to the hand-computed `DdsFixtures.bc7Mode6Block` fixture) and
the `VECTORS` array. Paste the emitted `static final ...` lines into
`Bc7ReferenceVectors.java`, keeping the class's provenance header intact.

## Sanity cross-check

The mode-6 vector uses the same endpoint/P-bit/index values as the existing
hand-computed test `DdsDecoderTest.decodesBc7Mode6`. bcdec reproduces exactly
the hand-derived pixels there, which independently ties the reference toolchain
to spec-derived expectations before any of the other modes are relied on.
