package me.mzy.beamcraft.client.assets;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * Unified asset discovery for JBeam, DAE mesh and material/texture loaders.
 *
 * <p>A configured asset root is a directory whose <em>direct children</em> are
 * containers: folders or {@code .zip} archives. The outer container name is
 * arbitrary; the authoritative vehicle name is the inner {@code vehicles/&lt;ns&gt;/}
 * path segment (BeamNG mod convention), matched segment-boundary-aware so
 * {@code vehicles/sunburst2/} never matches namespace {@code sunburst}. The
 * {@code common} namespace is resolved by the same rule, plus a legacy fallback
 * that accepts every entry in containers literally named {@code common} /
 * {@code common.zip} (their internal layout mirrors the vehicle folders).
 *
 * <p>Entries are grouped by their lowercased container-relative path; when one
 * logical path exists in several sources the configured
 * {@link ConflictPolicy} picks the winner and {@link ConflictReporter} logs (and
 * optionally notifies in-game). Containers are deduplicated by canonical path so
 * the same physical archive under two roots counts once.
 */
public final class AssetScanner {

    public static final AssetScanner INSTANCE = new AssetScanner();

    private volatile ConflictPolicy policy = ConflictPolicy.DEFAULT;

    private AssetScanner() {
    }

    /** Called once at client init with the loaded config policy. */
    public void configure(ConflictPolicy policy) {
        if (policy != null) {
            this.policy = policy;
        }
    }

    public NamespaceScan scan(List<File> roots, String namespace) {
        return scan(roots, namespace, policy);
    }

    /** Policy-aware variant, so tests can exercise every strategy without touching the singleton. */
    public NamespaceScan scan(List<File> roots, String namespace, ConflictPolicy policy) {
        String ns = namespace == null ? "common" : namespace.toLowerCase(Locale.ROOT);
        ConflictPolicy pol = policy == null ? ConflictPolicy.DEFAULT : policy;
        boolean isCommon = ns.equals("common");

        Map<String, Integer> seenContainers = new HashMap<>();
        Map<String, List<ResolvedEntry>> groups = new LinkedHashMap<>();

        if (roots != null) {
            for (int rootIndex = 0; rootIndex < roots.size(); rootIndex++) {
                File root = roots.get(rootIndex);
                if (root == null || !root.isDirectory()) {
                    continue;
                }
                File[] children = root.listFiles();
                if (children == null) {
                    continue;
                }
                List<File> containers = new ArrayList<>();
                for (File child : children) {
                    if (child.isDirectory() || (child.isFile()
                            && child.getName().toLowerCase(Locale.ROOT).endsWith(".zip"))) {
                        containers.add(child);
                    }
                }
                containers.sort(Comparator.comparing(File::getName, String.CASE_INSENSITIVE_ORDER));
                for (File child : containers) {
                    String id = canonicalPath(child);
                    if (id == null || seenContainers.containsKey(id)) {
                        continue;
                    }
                    seenContainers.put(id, rootIndex);
                    // Legacy common containers only feed the common namespace.
                    if (!isCommon && isLegacyCommonContainer(child.getName())) {
                        continue;
                    }
                    boolean legacyCommon = isCommon && isLegacyCommonContainer(child.getName());
                    AssetSource source = new AssetSource(child, child.isFile(), rootIndex, id);
                    collectContainer(source, ns, legacyCommon, groups);
                }
            }
        }

        List<ResolvedEntry> winners = new ArrayList<>();
        for (Map.Entry<String, List<ResolvedEntry>> group : groups.entrySet()) {
            List<ResolvedEntry> candidates = group.getValue();
            ResolvedEntry winner = selectWinner(candidates, pol);
            if (candidates.size() > 1) {
                List<String> losers = new ArrayList<>();
                for (ResolvedEntry c : candidates) {
                    if (c != winner) {
                        losers.add(c.sourceAddress());
                    }
                }
                ConflictReporter.INSTANCE.report(group.getKey(), winner.sourceAddress(), losers, pol.notifyChat());
            }
            winners.add(winner);
        }
        winners.sort(Comparator.comparing(ResolvedEntry::logicalPath));

        List<AssetSource> sources = new ArrayList<>();
        Map<String, Boolean> seenSources = new HashMap<>();
        for (ResolvedEntry w : winners) {
            if (!seenSources.containsKey(w.source().id())) {
                seenSources.put(w.source().id(), Boolean.TRUE);
                sources.add(w.source());
            }
        }
        sources.sort(Comparator.comparingInt(AssetSource::rootIndex)
                .thenComparing(AssetSource::containerName, String.CASE_INSENSITIVE_ORDER));

        return new NamespaceScan(ns, winners, sources);
    }

    /**
     * True when {@code path} (a zip entry name or container-relative file path)
     * sits under a {@code vehicles/&lt;namespace&gt;/} directory, as a leading
     * segment or an infix (wrapped mods). Segment-boundary-aware and
     * case-insensitive, so {@code vehicles/sunburst2/…} never matches {@code sunburst}.
     */
    public static boolean underVehiclesNamespace(String path, String namespace) {
        String ns = namespace == null ? "common" : namespace.toLowerCase(Locale.ROOT);
        String p = path.replace('\\', '/').toLowerCase(Locale.ROOT);
        return p.startsWith("vehicles/" + ns + "/") || p.contains("/vehicles/" + ns + "/");
    }

    /** True for a container literally named {@code common} or {@code common.zip}, case-insensitive. */
    public static boolean isLegacyCommonContainer(String name) {
        if (name == null) {
            return false;
        }
        String lower = name.toLowerCase(Locale.ROOT);
        return lower.equals("common") || lower.equals("common.zip");
    }

    public static String canonicalPath(File f) {
        try {
            return f.getCanonicalPath();
        } catch (IOException e) {
            return f.getAbsolutePath();
        }
    }

    private void collectContainer(AssetSource source, String ns, boolean legacyCommon,
                                  Map<String, List<ResolvedEntry>> groups) {
        try {
            if (source.isZip()) {
                try (ZipFile zf = new ZipFile(source.file())) {
                    List<ZipEntry> entries = new ArrayList<>();
                    var it = zf.entries();
                    while (it.hasMoreElements()) {
                        ZipEntry ze = it.nextElement();
                        if (!ze.isDirectory() && !ze.getName().contains("__MACOSX")) {
                            entries.add(ze);
                        }
                    }
                    entries.sort(Comparator.comparing(ZipEntry::getName));
                    for (ZipEntry ze : entries) {
                        String name = ze.getName();
                        if (!legacyCommon && !underVehiclesNamespace(name, ns)) {
                            continue;
                        }
                        String key = normalize(name);
                        if (key == null) {
                            continue;
                        }
                        groups.computeIfAbsent(key, k -> new ArrayList<>())
                                .add(new ResolvedEntry(source, ze.getName(), key, ze.getTime()));
                    }
                }
            } else {
                try (Stream<Path> paths = Files.walk(source.file().toPath())) {
                    List<Path> files = new ArrayList<>();
                    paths.filter(Files::isRegularFile).forEach(files::add);
                    files.sort(Comparator.comparing(p -> p.getFileName().toString(), String.CASE_INSENSITIVE_ORDER));
                    Path root = source.file().toPath().toAbsolutePath().normalize();
                    for (Path p : files) {
                        String rel = root.relativize(p.toAbsolutePath().normalize()).toString().replace('\\', '/');
                        if (rel.contains("__MACOSX")) {
                            continue;
                        }
                        if (!legacyCommon && !underVehiclesNamespace(rel, ns)) {
                            continue;
                        }
                        String key = normalize(rel);
                        if (key == null) {
                            continue;
                        }
                        long mtime;
                        try {
                            mtime = Files.getLastModifiedTime(p).toMillis();
                        } catch (IOException e) {
                            mtime = -1L;
                        }
                        groups.computeIfAbsent(key, k -> new ArrayList<>())
                                .add(new ResolvedEntry(source, rel, key, mtime));
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("🚨 [Assets] Failed to scan container " + source.file().getAbsolutePath() + ": " + e.getMessage());
        }
    }

    /** Lowercased, forward-slash path used as the conflict group key. */
    private static String normalize(String path) {
        String p = path.replace('\\', '/');
        while (p.startsWith("/")) {
            p = p.substring(1);
        }
        return p.toLowerCase(Locale.ROOT);
    }

    private static ResolvedEntry selectWinner(List<ResolvedEntry> candidates, ConflictPolicy policy) {
        if (candidates.size() <= 1) {
            return candidates.get(0);
        }
        List<ResolvedEntry> ordered = new ArrayList<>(candidates);
        ordered.sort(Comparator.comparingInt((ResolvedEntry r) -> r.source().rootIndex())
                .thenComparing(r -> r.source().containerName(), String.CASE_INSENSITIVE_ORDER));
        switch (policy.strategy()) {
            case EARLIER_ROOT:
                return ordered.get(0);
            case NEWER:
                ResolvedEntry best = ordered.get(0);
                for (ResolvedEntry r : ordered) {
                    if (newerThan(r, best)) {
                        best = r;
                    }
                }
                return best;
            case LATER_ROOT:
            default:
                return ordered.get(ordered.size() - 1);
        }
    }

    /** Compares by effective mtime (unknown treated as oldest); ties break to later root, then later container. */
    private static boolean newerThan(ResolvedEntry a, ResolvedEntry b) {
        long mtA = a.lastModified() < 0 ? Long.MIN_VALUE : a.lastModified();
        long mtB = b.lastModified() < 0 ? Long.MIN_VALUE : b.lastModified();
        if (mtA != mtB) {
            return mtA > mtB;
        }
        if (a.source().rootIndex() != b.source().rootIndex()) {
            return a.source().rootIndex() > b.source().rootIndex();
        }
        return a.source().containerName().compareToIgnoreCase(b.source().containerName()) > 0;
    }
}
