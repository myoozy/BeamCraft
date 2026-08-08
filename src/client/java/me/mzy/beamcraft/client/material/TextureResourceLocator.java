package me.mzy.beamcraft.client.material;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * Resolves a logical BeamNG texture path (e.g. {@code /vehicles/foo/foo_d.png})
 * against the ZIP archives and loose folders that {@link MaterialLibrary}
 * indexes, mirroring {@code DaeMeshLoader}'s discovery scope.
 *
 * <p>Resolution rules:
 * <ul>
 *   <li>Leading slashes are stripped and backslashes normalised to forward
 *       slashes, so both {@code /vehicles/...} and {@code vehicles/...} work.</li>
 *   <li>Matching is case-insensitive while retaining the archive's real entry
 *       name for later reads.</li>
 *   <li>A logical {@code *.png} reference falls back to the same-stem
 *       {@code *.dds} entry, because BeamNG material JSON paths usually say
 *       {@code .png} while archives ship {@code .dds}.</li>
 * </ul>
 *
 * <p><b>Path safety</b>: logical paths are rejected when they contain an empty,
 * {@code .} or {@code ..} segment or a drive-letter {@code :}, so resolution
 * can never escape a registered source. Loose-folder matches are additionally
 * verified to stay inside the registered canonical root. The returned
 * {@link TextureResource} is an opaque handle that callers can only obtain
 * from this class and hand back to {@link #readBytes}; it cannot be forged to
 * point at arbitrary files.
 *
 * <p>No texture bytes are loaded eagerly, and no {@link ZipFile} handle is kept
 * open: ZIP entry names are indexed lazily on first use (the handle is closed
 * immediately after), and {@link #readBytes} re-opens the archive transiently
 * per call. A successful {@link #resolve} returns a stable
 * {@link TextureResource} that can be read later.
 *
 * <p>Sources are reference-counted: {@link #registerSource} may be called once
 * per owning namespace and {@link #unregisterSource} must be called the same
 * number of times. Only the final unregister drops the source and its lazily
 * built ZIP index, so releasing one namespace never disturbs another that
 * shares the same physical source.
 */
final class TextureResourceLocator {

    private static final class Source {
        final File file;
        final boolean zip;
        // Lazy, case-insensitive ZIP entry index: lowercase entry name -> real entry name.
        Map<String, String> zipIndex;

        Source(File file, boolean zip) {
            this.file = file;
            this.zip = zip;
        }
    }

    private final List<Source> sources = new ArrayList<>();
    private final Map<String, Source> byCanonicalPath = new HashMap<>();
    private final Map<String, Integer> refCounts = new HashMap<>();

    /**
     * Indexes an archive or folder for later lookups, for one owning namespace.
     * Idempotent by canonical path; registering the same path again only bumps
     * its reference count. A {@code .zip} file is treated as a ZIP archive,
     * anything else as a loose folder whose relative layout mirrors the logical
     * {@code vehicles/...} tree.
     */
    void registerSource(File file) {
        if (file == null || !file.exists()) return;
        File canonical;
        try {
            canonical = file.getCanonicalFile();
        } catch (IOException e) {
            // Not resolvable; fail soft and ignore this source.
            return;
        }
        String key = canonical.getPath();
        Integer existing = refCounts.get(key);
        if (existing != null) {
            refCounts.put(key, existing + 1);
            return;
        }
        boolean zip = file.isFile() && file.getName().toLowerCase(Locale.ROOT).endsWith(".zip");
        Source source = new Source(canonical, zip);
        sources.add(source);
        byCanonicalPath.put(key, source);
        refCounts.put(key, 1);
    }

    /**
     * Releases one owner of a source. Only when the reference count drops to
     * zero is the source dropped and its lazily built ZIP index reclaimed;
     * re-registering later rebuilds the index from the (possibly changed)
     * archive or folder.
     */
    void unregisterSource(File file) {
        if (file == null) return;
        File canonical;
        try {
            canonical = file.getCanonicalFile();
        } catch (IOException e) {
            return;
        }
        String key = canonical.getPath();
        Integer count = refCounts.get(key);
        if (count == null) {
            return;
        }
        if (count > 1) {
            refCounts.put(key, count - 1);
            return;
        }
        refCounts.remove(key);
        Source source = byCanonicalPath.remove(key);
        if (source != null) {
            sources.remove(source);
        }
    }

    /** Number of distinct registered sources (diagnostic). */
    int getSourceCount() {
        return sources.size();
    }

    /**
     * Resolves a logical texture path to a {@link TextureResource}, or returns
     * null when no indexed source contains it (exact or same-stem {@code .dds}).
     * Returns null for any path that is absolute, drive-styled, or contains a
     * traversal or empty segment.
     */
    TextureResource resolve(String logicalPath) {
        String normalized = normalize(logicalPath);
        if (normalized == null) {
            return null;
        }
        TextureResource found = lookup(normalized);
        if (found == null && normalized.toLowerCase(Locale.ROOT).endsWith(".png")) {
            found = lookup(normalized.substring(0, normalized.length() - 4) + ".dds");
        }
        return found;
    }

    /** Reads the full texture bytes for a resolved resource. */
    byte[] readBytes(TextureResource resource) throws IOException {
        if (resource == null) {
            throw new FileNotFoundException("Null texture resource");
        }
        requireSafeEntry(resource.entryPath());
        if (resource.zip()) {
            try (ZipFile zipFile = new ZipFile(resource.source())) {
                ZipEntry entry = zipFile.getEntry(resource.entryPath());
                if (entry == null) {
                    throw new FileNotFoundException("Missing ZIP entry " + resource.entryPath()
                            + " in " + resource.sourceId());
                }
                try (InputStream in = zipFile.getInputStream(entry)) {
                    return in.readAllBytes();
                }
            }
        }
        Path sourceRoot = resource.source().toPath().toAbsolutePath().normalize();
        Path target = sourceRoot.resolve(resource.entryPath()).toAbsolutePath().normalize();
        if (!target.startsWith(sourceRoot)) {
            throw new FileNotFoundException("Texture path escapes source root: " + resource.describe());
        }
        return Files.readAllBytes(target);
    }

    private static String normalize(String path) {
        if (path == null || path.isEmpty()) {
            return null;
        }
        String p = path.replace('\\', '/');
        while (p.startsWith("/")) {
            p = p.substring(1);
        }
        if (p.contains(":")) {
            return null; // Windows drive letter / URI scheme style absolute path
        }
        StringBuilder sb = new StringBuilder();
        for (String segment : p.split("/")) {
            if (segment.isEmpty() || segment.equals(".") || segment.equals("..")) {
                return null; // empty, current-directory or traversal segment
            }
            if (sb.length() > 0) {
                sb.append('/');
            }
            sb.append(segment);
        }
        return sb.length() == 0 ? null : sb.toString();
    }

    private static void requireSafeEntry(String entryPath) throws FileNotFoundException {
        for (String segment : entryPath.split("/")) {
            if (segment.isEmpty() || segment.equals(".") || segment.equals("..")) {
                throw new FileNotFoundException("Unsafe texture entry path: " + entryPath);
            }
        }
    }

    private TextureResource lookup(String normalized) {
        for (Source source : sources) {
            if (source.zip) {
                String realEntry = zipIndex(source).get(normalized.toLowerCase(Locale.ROOT));
                if (realEntry != null) {
                    return new TextureResource(source.file, realEntry, true);
                }
            } else {
                Path root = source.file.toPath();
                Path resolved = caseInsensitivePath(root, normalized.split("/"));
                if (resolved != null && isUnder(root, resolved)) {
                    String relative = root.relativize(resolved).toString().replace('\\', '/');
                    return new TextureResource(source.file, relative, false);
                }
            }
        }
        return null;
    }

    private Map<String, String> zipIndex(Source source) {
        if (source.zipIndex != null) {
            return source.zipIndex;
        }
        Map<String, String> index = new HashMap<>();
        try (ZipFile zipFile = new ZipFile(source.file)) {
            Enumeration<? extends ZipEntry> entries = zipFile.entries();
            while (entries.hasMoreElements()) {
                ZipEntry entry = entries.nextElement();
                if (entry.isDirectory()) {
                    continue;
                }
                String name = entry.getName();
                index.put(name.toLowerCase(Locale.ROOT), name);
            }
        } catch (IOException e) {
            System.err.println("⚠️ [Materials] Failed to index ZIP " + source.file.getAbsolutePath() + ": " + e.getMessage());
        }
        source.zipIndex = index;
        return index;
    }

    private static boolean isUnder(Path root, Path candidate) {
        return candidate.toAbsolutePath().normalize().startsWith(root.toAbsolutePath().normalize());
    }

    private static Path caseInsensitivePath(Path root, String[] segments) {
        Path current = root;
        for (int i = 0; i < segments.length; i++) {
            String segment = segments[i];
            if (segment.isEmpty()) {
                return null;
            }
            boolean leaf = (i == segments.length - 1);
            Path exact = current.resolve(segment);
            if (leaf ? Files.isRegularFile(exact) : Files.isDirectory(exact)) {
                current = exact;
                continue;
            }
            Path matched = matchChildIgnoreCase(current, segment, leaf);
            if (matched == null) {
                return null;
            }
            current = matched;
        }
        return Files.isRegularFile(current) ? current : null;
    }

    private static Path matchChildIgnoreCase(Path parent, String segment, boolean leaf) {
        try (DirectoryStream<Path> children = Files.newDirectoryStream(parent)) {
            for (Path child : children) {
                String name = child.getFileName().toString();
                if (name.equalsIgnoreCase(segment)) {
                    if (leaf ? Files.isRegularFile(child) : Files.isDirectory(child)) {
                        return child;
                    }
                }
            }
        } catch (IOException e) {
            // Fall through to null.
        }
        return null;
    }
}
