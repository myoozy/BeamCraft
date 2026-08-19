package me.mzy.beamcraft.texture;

/**
 * Composes a base-colour RGBA texture with a separate single-channel opacity
 * texture, producing one RGBA image for upload.
 *
 * <p><b>Alpha rule (BeamNG-appropriate)</b>: the opacity texture's meaningful
 * single channel -- the <em>red</em> channel, see the BC4/8-bit decode contract
 * in {@link DdsDecoder} -- <em>multiplies</em> the base image's alpha:
 *
 * <pre>out.a = round(base.a * opacity.r / 255)</pre>
 *
 * <p>Multiplication is chosen because it degenerates to a pure replace when the
 * base has no alpha of its own: a DXT1/BC1 diffuse decodes with alpha 255, so
 * {@code out.a} becomes exactly the opacity value, which is the classic BeamNG
 * cutout behaviour for a BC4 opacity map. When the diffuse does carry baked
 * alpha (BC3/DXT5), it is preserved and scaled rather than discarded.
 *
 * <p>The opacity channel is a mask, not a colour: no color-space conversion is
 * applied to it, and its own {@code srgb} flag is ignored. The result inherits
 * the base image's {@code srgb} flag. Material-level alpha factors are applied
 * later, at render time, not here.
 *
 * <p><b>Dimension mismatch</b>: the two textures must have identical
 * dimensions; otherwise this throws {@link IllegalArgumentException}. BeamNG
 * emits matching diffuse/opacity pairs per stage, and silently scaling one of
 * the pair would misalign cutout geometry, so mismatch is rejected rather than
 * guessed.
 */
public final class TextureCompositor {

    private TextureCompositor() {
    }

    /**
     * Composes {@code base} with {@code opacity} into a new RGBA image. The
     * inputs are not modified and may be released by the caller afterwards.
     *
     * @param base    base-colour RGBA texture
     * @param opacity single-channel opacity texture
     * @return a new image with {@code rgb} from {@code base} and
     *         {@code a = round(base.a * opacity.r / 255)}
     * @throws NullPointerException     if either argument is null
     * @throws IllegalArgumentException if the dimensions differ
     */
    public static DecodedImage composeBaseWithOpacity(DecodedImage base, DecodedImage opacity) {
        if (base.width() != opacity.width() || base.height() != opacity.height()) {
            throw new IllegalArgumentException("diffuse/opacity dimension mismatch: " + base.width() + "x" + base.height()
                    + " vs " + opacity.width() + "x" + opacity.height());
        }
        byte[] basePixels = base.pixelData();
        byte[] opacityPixels = opacity.pixelData();
        byte[] out = new byte[basePixels.length];
        for (int i = 0; i < basePixels.length; i += 4) {
            out[i] = basePixels[i];
            out[i + 1] = basePixels[i + 1];
            out[i + 2] = basePixels[i + 2];
            int baseAlpha = basePixels[i + 3] & 0xFF;
            int opacityValue = opacityPixels[i] & 0xFF;
            out[i + 3] = (byte) ((baseAlpha * opacityValue + 127) / 255);
        }
        return DecodedImage.ofOwned(base.width(), base.height(), out, base.isSrgb());
    }
}
