package me.mzy.beamcraft.texture;

import java.io.IOException;

/**
 * Thrown when a DDS file is well formed but uses a surface format this decoder
 * does not implement (for example BC5, BC6H, a 10:10:10:2 uncompressed layout,
 * or a paletted surface). The message names the format so callers can log or
 * fail soft per texture without losing the cause.
 *
 * <p>This is distinct from {@link DdsDecodeException}, which indicates a
 * structurally invalid or truncated file. A decoder that cannot handle a format
 * must throw this type rather than fabricate pixels.
 */
public final class UnsupportedDdsFormatException extends IOException {

    public UnsupportedDdsFormatException(String message) {
        super(message);
    }

    public UnsupportedDdsFormatException(String message, Throwable cause) {
        super(message, cause);
    }
}
