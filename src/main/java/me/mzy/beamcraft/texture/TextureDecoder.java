package me.mzy.beamcraft.texture;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;

/** Decodes the vehicle texture formats supported by BeamCraft into RGBA8. */
public final class TextureDecoder {

    enum Format {
        DDS,
        PNG,
        JPEG
    }

    private static final byte[] PNG_SIGNATURE = {
            (byte) 0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A
    };

    private TextureDecoder() {
    }

    public static DecodedImage decode(byte[] data) throws IOException {
        return switch (detectFormat(data)) {
            case DDS -> DdsDecoder.decode(data);
            case PNG, JPEG -> decodeStandardImage(data);
        };
    }

    static Format detectFormat(byte[] data) throws IOException {
        if (data == null) {
            throw new IOException("null texture data");
        }
        if (data.length >= 4
                && data[0] == 'D' && data[1] == 'D' && data[2] == 'S' && data[3] == ' ') {
            return Format.DDS;
        }
        if (startsWith(data, PNG_SIGNATURE)) {
            return Format.PNG;
        }
        if (data.length >= 3
                && (data[0] & 0xFF) == 0xFF
                && (data[1] & 0xFF) == 0xD8
                && (data[2] & 0xFF) == 0xFF) {
            return Format.JPEG;
        }
        throw new IOException("unsupported texture format (expected DDS, PNG, or JPEG signature)");
    }

    private static DecodedImage decodeStandardImage(byte[] data) throws IOException {
        BufferedImage image = ImageIO.read(new ByteArrayInputStream(data));
        if (image == null) {
            throw new IOException("ImageIO could not decode the PNG/JPEG texture");
        }

        int width = image.getWidth();
        int height = image.getHeight();
        long byteCount = (long) width * height * 4;
        if (width <= 0 || height <= 0 || byteCount > Integer.MAX_VALUE) {
            throw new IOException("decoded image is too large: " + width + "x" + height);
        }

        byte[] rgba = new byte[(int) byteCount];
        int[] row = new int[width];
        int output = 0;
        for (int y = 0; y < height; y++) {
            image.getRGB(0, y, width, 1, row, 0, width);
            for (int argb : row) {
                rgba[output++] = (byte) (argb >>> 16);
                rgba[output++] = (byte) (argb >>> 8);
                rgba[output++] = (byte) argb;
                rgba[output++] = (byte) (argb >>> 24);
            }
        }

        // BufferedImage.getRGB returns non-premultiplied default-sRGB values.
        return DecodedImage.ofOwned(width, height, rgba, true);
    }

    private static boolean startsWith(byte[] data, byte[] signature) {
        if (data.length < signature.length) {
            return false;
        }
        for (int i = 0; i < signature.length; i++) {
            if (data[i] != signature[i]) {
                return false;
            }
        }
        return true;
    }
}
