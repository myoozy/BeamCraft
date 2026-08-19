package me.mzy.beamcraft.client.material;

/**
 * Immutable RGBA colour as four scalar float components. Small value type used
 * for material colour factors so that descriptors ({@link MaterialDefinition},
 * {@link MaterialStage}) never expose a mutable backing array.
 *
 * <p>Instances are interchangeable with equal components; there is deliberately
 * no public array form. {@code a} is the alpha channel, {@code 1} when a
 * three-component source colour was defaulted.
 */
public record RgbaColor(float r, float g, float b, float a) {

    /** White, opaque. Useful when a colour is required but the source omitted one. */
    public static final RgbaColor WHITE = new RgbaColor(1f, 1f, 1f, 1f);

    @Override
    public String toString() {
        return "RgbaColor(" + r + ", " + g + ", " + b + ", " + a + ')';
    }
}
