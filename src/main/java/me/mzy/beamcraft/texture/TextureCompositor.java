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
        return composeBaseWithOpacity(base, opacity, false);
    }

    /**
     * As {@link #composeBaseWithOpacity(DecodedImage, DecodedImage)}, optionally
     * premultiplying {@code rgb} by the mask.
     *
     * <p>{@code premultiplyRgb} is for a material whose BeamNG {@code translucentBlendOp}
     * is {@code PreMulAlpha}: that mode blends with {@code (ONE, ONE_MINUS_SRC_ALPHA)},
     * which weights the source rgb by nothing, so the rgb has to arrive already scaled
     * by the effective alpha. Scaling only the alpha would leave the mask's soft parts
     * adding full-strength colour — the mask's antialiased edge would glow.
     *
     * @param premultiplyRgb scale {@code rgb} by the mask as well as the alpha
     */
    public static DecodedImage composeBaseWithOpacity(DecodedImage base, DecodedImage opacity,
                                                      boolean premultiplyRgb) {
        if (base.width() != opacity.width() || base.height() != opacity.height()) {
            throw new IllegalArgumentException("diffuse/opacity dimension mismatch: " + base.width() + "x" + base.height()
                    + " vs " + opacity.width() + "x" + opacity.height());
        }
        byte[] basePixels = base.pixelData();
        byte[] opacityPixels = opacity.pixelData();
        byte[] out = new byte[basePixels.length];
        for (int i = 0; i < basePixels.length; i += 4) {
            int opacityValue = opacityPixels[i] & 0xFF;
            int rgbScale = premultiplyRgb ? opacityValue : 255;
            out[i] = (byte) (((basePixels[i] & 0xFF) * rgbScale + 127) / 255);
            out[i + 1] = (byte) (((basePixels[i + 1] & 0xFF) * rgbScale + 127) / 255);
            out[i + 2] = (byte) (((basePixels[i + 2] & 0xFF) * rgbScale + 127) / 255);
            int baseAlpha = basePixels[i + 3] & 0xFF;
            out[i + 3] = (byte) ((baseAlpha * opacityValue + 127) / 255);
        }
        return DecodedImage.ofOwned(base.width(), base.height(), out, base.isSrgb());
    }

    /**
     * Composes the opacity mask over flat white, for a material that has no
     * base-colour texture at all. BeamNG's stock grille materials are exactly this
     * shape: a flat {@code baseColorFactor} for the colour plus an {@code opacityMap}
     * for the holes, and no {@code baseColorMap} anywhere in the material.
     *
     * <p>White is the identity for the colour, so the plan's factor tints the result
     * unchanged; what matters is that the pattern rides on the <em>alpha</em> the
     * cutout shader tests, exactly as it does for a textured material. Without this
     * there is nothing for that test to read and the part draws as an unbroken panel.
     *
     * @param opacity single-channel opacity texture
     * @return a new image with white {@code rgb} and {@code a = opacity.r}
     */
    public static DecodedImage composeWhiteWithOpacity(DecodedImage opacity) {
        return composeWhiteWithOpacity(opacity, false);
    }

    /**
     * As {@link #composeWhiteWithOpacity(DecodedImage)}, optionally premultiplying the
     * white by the mask — premultiplied white <em>is</em> the mask, so the rgb becomes
     * the mask value rather than 255. See the base-composite overload for why.
     */
    public static DecodedImage composeWhiteWithOpacity(DecodedImage opacity, boolean premultiplyRgb) {
        byte[] opacityPixels = opacity.pixelData();
        byte[] out = new byte[opacityPixels.length];
        for (int i = 0; i < out.length; i += 4) {
            int mask = opacityPixels[i] & 0xFF;
            byte channel = (byte) (premultiplyRgb ? mask : 0xFF);
            out[i] = channel;
            out[i + 1] = channel;
            out[i + 2] = channel;
            // The mask's own srgb flag is ignored, as it is for the base-composite path:
            // the channel is coverage, not colour.
            out[i + 3] = opacityPixels[i];
        }
        return DecodedImage.ofOwned(opacity.width(), opacity.height(), out, opacity.isSrgb());
    }
}
