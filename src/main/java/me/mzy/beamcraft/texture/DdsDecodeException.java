package me.mzy.beamcraft.texture;

import java.io.IOException;

/**
 * Thrown when a DDS file is structurally invalid or truncated: bad magic,
 * impossible header fields, or a surface that runs past the end of the byte
 * array. Callers treat this as "this texture is corrupt" and fail soft for that
 * texture; it must never yield partially decoded pixels.
 */
public final class DdsDecodeException extends IOException {

    public DdsDecodeException(String message) {
        super(message);
    }

    public DdsDecodeException(String message, Throwable cause) {
        super(message, cause);
    }
}
