package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TextureCompositorTest {

    private static DecodedImage rgba(int w, int h, int[] pixels) {
        byte[] data = new byte[w * h * 4];
        for (int i = 0; i < pixels.length; i++) {
            int p = pixels[i];
            data[i * 4] = (byte) (p >>> 16);
            data[i * 4 + 1] = (byte) (p >>> 8);
            data[i * 4 + 2] = (byte) p;
            data[i * 4 + 3] = (byte) (p >>> 24);
        }
        return DecodedImage.of(w, h, data, true);
    }

    @Test
    void multipliesOpacityIntoBaseAlpha() {
        // Base: red, alpha 128 (0x80). Opacity: value 128 (0x80).
        // out.a = round(128*128/255) = 64.
        DecodedImage base = rgba(1, 1, new int[] {0x80FF0000});
        DecodedImage opacity = rgba(1, 1, new int[] {0xFF808080}); // replicated single channel
        DecodedImage out = TextureCompositor.composeBaseWithOpacity(base, opacity);
        assertEquals(0x40FF0000, out.getPixelRgba(0, 0));
        assertTrue(out.isSrgb(), "result inherits base srgb flag");
    }

    @Test
    void opaqueBaseDegeneratesToReplace() {
        // Base opaque (BC1/DXT1-style alpha 255): out.a must equal the opacity value exactly.
        DecodedImage base = rgba(1, 1, new int[] {0xFFFFFFFF});
        DecodedImage opacity = rgba(1, 1, new int[] {0xFF303030}); // opacity 48
        DecodedImage out = TextureCompositor.composeBaseWithOpacity(base, opacity);
        assertEquals(0x30FFFFFF, out.getPixelRgba(0, 0));
    }

    @Test
    void fullyTransparentOpacityZeroesAlpha() {
        DecodedImage base = rgba(1, 1, new int[] {0x80FF0000});
        DecodedImage opacity = rgba(1, 1, new int[] {0xFF000000}); // opacity 0
        DecodedImage out = TextureCompositor.composeBaseWithOpacity(base, opacity);
        assertEquals(0x00FF0000, out.getPixelRgba(0, 0));
    }

    @Test
    void keepsBaseColourUnchanged() {
        DecodedImage base = rgba(2, 1, new int[] {0x80FF0000, 0x8000FF00});
        DecodedImage opacity = rgba(2, 1, new int[] {0xFFFFFFFF, 0xFF808080});
        DecodedImage out = TextureCompositor.composeBaseWithOpacity(base, opacity);
        assertEquals(0xFF, (out.getPixelRgba(0, 0) >>> 16) & 0xFF); // red kept
        assertEquals(0xFF, (out.getPixelRgba(1, 0) >>> 8) & 0xFF);  // green kept
    }

    @Test
    void rejectsDimensionMismatch() {
        DecodedImage base = rgba(2, 2, new int[4]);
        DecodedImage opacity = rgba(4, 4, new int[16]);
        assertThrows(IllegalArgumentException.class, () -> TextureCompositor.composeBaseWithOpacity(base, opacity));
    }

    @Test
    void rejectsNullInput() {
        DecodedImage img = rgba(1, 1, new int[1]);
        assertThrows(NullPointerException.class, () -> TextureCompositor.composeBaseWithOpacity(null, img));
        assertThrows(NullPointerException.class, () -> TextureCompositor.composeBaseWithOpacity(img, null));
    }
}
