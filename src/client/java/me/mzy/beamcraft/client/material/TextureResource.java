package me.mzy.beamcraft.client.material;

import java.io.File;
import java.util.Objects;

/**
 * Opaque, immutable handle to one texture resolved from a registered source.
 * Instances can only be obtained from {@link TextureResourceLocator} (via
 * {@link MaterialLibrary#resolveTexture}) and handed back to
 * {@link TextureResourceLocator#readBytes} (via
 * {@link MaterialLibrary#readTexture}) to read the bytes.
 *
 * <p>The handle is deliberately opaque: the source archive/folder and the
 * entry path are private, there is no public constructor or factory, and no
 * public accessor exposes the backing {@link File}. Callers cannot forge a
 * handle to point at arbitrary filesystem paths; only the locator (same
 * package) can construct and unpack one, and it re-verifies path safety on
 * every read.
 *
 * <p>The public methods are limited to safe diagnostics and cache identity:
 * {@link #sourceId()} gives a stable identity for caching, {@link #describe()}
 * a human-readable form for logs, and {@code equals}/{@code hashCode} value
 * semantics for use as a map key. Instances are interchangeable when they
 * refer to the same source entry.
 */
public final class TextureResource {

    private final File source;
    private final String entryPath;
    private final boolean zip;

    /** Package-private: only {@link TextureResourceLocator} constructs handles. */
    TextureResource(File source, String entryPath, boolean zip) {
        this.source = Objects.requireNonNull(source, "source");
        this.entryPath = Objects.requireNonNull(entryPath, "entryPath");
        this.zip = zip;
    }

    /** Stable source identity for caching/diagnostics. */
    public String sourceId() {
        return source.getAbsolutePath();
    }

    /** Human-readable {@code <archive>!<entry>} diagnostic. */
    public String describe() {
        return sourceId() + "!" + entryPath;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof TextureResource that)) {
            return false;
        }
        return zip == that.zip
                && source.equals(that.source)
                && entryPath.equals(that.entryPath);
    }

    @Override
    public int hashCode() {
        int result = source.hashCode();
        result = 31 * result + entryPath.hashCode();
        result = 31 * result + (zip ? 1 : 0);
        return result;
    }

    @Override
    public String toString() {
        return describe();
    }

    // Package-private accessors, used by TextureResourceLocator only.
    File source() {
        return source;
    }

    String entryPath() {
        return entryPath;
    }

    boolean zip() {
        return zip;
    }
}
