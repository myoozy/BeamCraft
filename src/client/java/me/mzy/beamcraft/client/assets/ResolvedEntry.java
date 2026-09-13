package me.mzy.beamcraft.client.assets;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * One conflict-resolved asset file inside a {@link AssetSource}. Instances are
 * produced by {@link AssetScanner}; callers read bytes with {@link #open()} /
 * {@link #readBytes()}, or {@link #materializeForAssimp()} for files Assimp
 * needs as a real path (DAE meshes).
 *
 * <p>{@link #entryName()} preserves the archive's real casing for opening; the
 * lowercased {@link #logicalPath()} is the conflict/dedupe key.
 */
public final class ResolvedEntry {

    private final AssetSource source;
    private final String entryName;
    private final String logicalPath;
    private final long lastModified;
    private Path materializedTemp;

    ResolvedEntry(AssetSource source, String entryName, String logicalPath, long lastModified) {
        this.source = source;
        this.entryName = entryName;
        this.logicalPath = logicalPath;
        this.lastModified = lastModified;
    }

    public AssetSource source() {
        return source;
    }

    /** Real (case-preserving) entry name relative to the container. */
    public String entryName() {
        return entryName;
    }

    /** Lowercased, forward-slash, container-relative path (the dedupe key). */
    public String logicalPath() {
        return logicalPath;
    }

    /** Last-modified epoch millis; {@code -1} when unknown (zip entries without a timestamp). */
    public long lastModified() {
        return lastModified;
    }

    public boolean isZip() {
        return source.isZip();
    }

    /** Human-readable address for conflict messages: {@code <zip>!<entry>} or absolute file path. */
    public String sourceAddress() {
        if (source.isZip()) {
            return source.file().getAbsolutePath() + "!" + entryName;
        }
        return source.file().toPath().resolve(entryName).toAbsolutePath().normalize().toString();
    }

    /** Opens a fresh stream over the entry bytes. Closing it releases any backing {@link ZipFile}. */
    public InputStream open() throws IOException {
        if (source.isZip()) {
            ZipFile zipFile = new ZipFile(source.file());
            ZipEntry entry = zipFile.getEntry(entryName);
            if (entry == null) {
                zipFile.close();
                throw new IOException("Missing zip entry " + entryName + " in " + source.file());
            }
            return new ZipEntryInputStream(zipFile, zipFile.getInputStream(entry));
        }
        return Files.newInputStream(source.file().toPath().resolve(entryName));
    }

    /** Reads the full entry bytes. */
    public byte[] readBytes() throws IOException {
        try (InputStream in = open()) {
            return in.readAllBytes();
        }
    }

    /**
     * Materializes the entry as a real file path for Assimp. Folder entries
     * return their actual path (nothing to delete); zip entries are extracted to
     * a temporary {@code *.dae} file that the caller must release via
     * {@link #deleteTemp()}.
     */
    public Path materializeForAssimp() throws IOException {
        if (!source.isZip()) {
            return source.file().toPath().resolve(entryName);
        }
        materializedTemp = Files.createTempFile("beamcraft_dae_", ".dae");
        try (InputStream in = open()) {
            Files.copy(in, materializedTemp, StandardCopyOption.REPLACE_EXISTING);
        }
        return materializedTemp;
    }

    /** Deletes the temporary file created by {@link #materializeForAssimp()}, if any. */
    public void deleteTemp() {
        if (materializedTemp != null) {
            try {
                Files.deleteIfExists(materializedTemp);
            } catch (IOException ignored) {
            }
            materializedTemp = null;
        }
    }

    /** Stream wrapper that also closes the owning {@link ZipFile}. */
    private static final class ZipEntryInputStream extends InputStream {
        private final ZipFile zipFile;
        private final InputStream delegate;

        ZipEntryInputStream(ZipFile zipFile, InputStream delegate) {
            this.zipFile = zipFile;
            this.delegate = delegate;
        }

        @Override
        public int read() throws IOException {
            return delegate.read();
        }

        @Override
        public int read(byte[] b, int off, int len) throws IOException {
            return delegate.read(b, off, len);
        }

        @Override
        public void close() throws IOException {
            try {
                delegate.close();
            } finally {
                zipFile.close();
            }
        }
    }
}
