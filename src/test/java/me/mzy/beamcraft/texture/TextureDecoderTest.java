package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Test;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TextureDecoderTest {

    @Test
    void pngPreservesAlphaAndTopRowOrigin() throws Exception {
        BufferedImage source = new BufferedImage(1, 2, BufferedImage.TYPE_INT_ARGB);
        source.setRGB(0, 0, 0x280A141E);
        source.setRGB(0, 1, 0xFFC89664);
        byte[] encoded = encode(source, "png");

        assertEquals(TextureDecoder.Format.PNG, TextureDecoder.detectFormat(encoded));
        DecodedImage decoded = TextureDecoder.decode(encoded);
        assertEquals(0x280A141E, decoded.getPixelRgba(0, 0));
        assertEquals(0xFFC89664, decoded.getPixelRgba(0, 1));
        assertTrue(decoded.isSrgb());
    }

    @Test
    void jpegDecodesOpaqueSrgbPixels() throws Exception {
        BufferedImage source = new BufferedImage(8, 8, BufferedImage.TYPE_INT_RGB);
        for (int y = 0; y < source.getHeight(); y++) {
            for (int x = 0; x < source.getWidth(); x++) {
                source.setRGB(x, y, 0x336699);
            }
        }
        byte[] encoded = encode(source, "jpg");

        assertEquals(TextureDecoder.Format.JPEG, TextureDecoder.detectFormat(encoded));
        DecodedImage decoded = TextureDecoder.decode(encoded);
        int pixel = decoded.getPixelRgba(4, 4);
        assertEquals(0xFF, pixel >>> 24);
        assertEquals(0x33, (pixel >>> 16) & 0xFF, 4);
        assertEquals(0x66, (pixel >>> 8) & 0xFF, 4);
        assertEquals(0x99, pixel & 0xFF, 4);
        assertTrue(decoded.isSrgb());
    }

    @Test
    void ddsStillUsesTheExistingDecoder() throws Exception {
        byte[] dds = DdsFixtures.legacyDds(1, 1, "", 0x40 | 0x1, 32,
                0x000000FF, 0x0000FF00, 0x00FF0000, 0xFF000000,
                new byte[]{10, 20, 30, 40});

        assertEquals(TextureDecoder.Format.DDS, TextureDecoder.detectFormat(dds));
        assertEquals(0x280A141E, TextureDecoder.decode(dds).getPixelRgba(0, 0));
    }

    @Test
    void rejectsUnknownAndCorruptKnownFormats() {
        assertThrows(IOException.class, () -> TextureDecoder.decode(new byte[]{1, 2, 3, 4}));
        assertThrows(IOException.class, () -> TextureDecoder.decode(new byte[]{
                (byte) 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A
        }));
    }

    private static byte[] encode(BufferedImage image, String format) throws IOException {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        assertTrue(ImageIO.write(image, format, output));
        return output.toByteArray();
    }
}
