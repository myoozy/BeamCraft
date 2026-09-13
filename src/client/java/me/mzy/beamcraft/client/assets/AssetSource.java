package me.mzy.beamcraft.client.assets;

import java.io.File;

/**
 * One asset container: a folder or {@code .zip} that lives directly under a
 * configured asset root. Containers are discovered by {@link AssetScanner} and
 * their identity is their canonical path, so the same physical archive/folder
 * registered through two roots counts once.
 */
public final class AssetSource {

    private final File file;
    private final boolean zip;
    private final int rootIndex;
    private final String id;

    AssetSource(File file, boolean zip, int rootIndex, String id) {
        this.file = file;
        this.zip = zip;
        this.rootIndex = rootIndex;
        this.id = id;
    }

    /** The container folder or {@code .zip} file. */
    public File file() {
        return file;
    }

    public boolean isZip() {
        return zip;
    }

    /** Index of the owning root in the configured {@code assetRoots} list. */
    public int rootIndex() {
        return rootIndex;
    }

    /** Canonical path; the container-level dedupe key. */
    public String id() {
        return id;
    }

    public String containerName() {
        return file == null ? "" : file.getName();
    }
}
