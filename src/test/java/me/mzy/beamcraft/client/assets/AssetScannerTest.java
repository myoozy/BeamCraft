package me.mzy.beamcraft.client.assets;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.FileTime;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for the unified {@link AssetScanner}: segment-boundary namespace
 * matching (a {@code vehicles/sunburst2/} path never matches {@code sunburst}),
 * outer container names being irrelevant, all three conflict strategies, the
 * legacy {@code common}/{@code common.zip} all-entries fallback, canonical-path
 * container dedupe, and the one-time chat notification gate.
 */
class AssetScannerTest {

    @TempDir
    Path root;

    @AfterEach
    void resetReporter() {
        ConflictReporter.INSTANCE.setChatSink(null);
    }

    private static void write(Path file, String content) throws IOException {
        Files.createDirectories(file.getParent());
        Files.writeString(file, content, StandardCharsets.UTF_8);
    }

    private static void writeZip(Path zip, Map<String, String> entries) throws IOException {
        try (ZipOutputStream out = new ZipOutputStream(Files.newOutputStream(zip))) {
            for (Map.Entry<String, String> e : entries.entrySet()) {
                out.putNextEntry(new ZipEntry(e.getKey()));
                out.write(e.getValue().getBytes(StandardCharsets.UTF_8));
                out.closeEntry();
            }
        }
    }

    private static String content(NamespaceScan scan) throws IOException {
        assertEquals(1, scan.entries().size());
        return new String(scan.entries().get(0).readBytes(), StandardCharsets.UTF_8);
    }

    @Test
    void segmentBoundaryDoesNotMatchSiblingNamespace() throws IOException {
        write(root.resolve("sunburst2/vehicles/sunburst2/body.jbeam"), "{}");

        NamespaceScan forSunburst = scan("sunburst", ConflictStrategy.LATER_ROOT);
        assertTrue(forSunburst.entries().isEmpty(),
                "vehicles/sunburst2/ must not satisfy namespace sunburst");

        NamespaceScan forSunburst2 = scan("sunburst2", ConflictStrategy.LATER_ROOT);
        assertEquals(1, forSunburst2.entries().size());
    }

    @Test
    void outerContainerNameIsIrrelevant() throws IOException {
        write(root.resolve("arbitrary_container/vehicles/pickup/body.jbeam"), "{}");
        assertEquals(1, scan("pickup", ConflictStrategy.LATER_ROOT).entries().size());
    }

    @Test
    void laterRootWins() throws IOException {
        Path r1 = root.resolve("r1");
        Path r2 = root.resolve("r2");
        write(r1.resolve("a/vehicles/pickup/body.jbeam"), "A");
        write(r2.resolve("b/vehicles/pickup/body.jbeam"), "B");
        assertEquals("B", content(scanAcross(List.of(r1, r2), ConflictStrategy.LATER_ROOT)));
    }

    @Test
    void earlierRootWins() throws IOException {
        Path r1 = root.resolve("r1");
        Path r2 = root.resolve("r2");
        write(r1.resolve("a/vehicles/pickup/body.jbeam"), "A");
        write(r2.resolve("b/vehicles/pickup/body.jbeam"), "B");
        assertEquals("A", content(scanAcross(List.of(r1, r2), ConflictStrategy.EARLIER_ROOT)));
    }

    @Test
    void newerFileWins() throws IOException {
        Path r1 = root.resolve("r1");
        Path r2 = root.resolve("r2");
        Path oldFile = r1.resolve("a/vehicles/pickup/body.jbeam");
        Path newFile = r2.resolve("b/vehicles/pickup/body.jbeam");
        write(oldFile, "old");
        write(newFile, "new");
        Files.setLastModifiedTime(oldFile, FileTime.fromMillis(1_000));
        Files.setLastModifiedTime(newFile, FileTime.fromMillis(2_000));
        assertEquals("new", content(scanAcross(List.of(r1, r2), ConflictStrategy.NEWER)));
    }

    @Test
    void zipEntryConflictResolvesByStrategy() throws IOException {
        Path r1 = root.resolve("r1");
        Path r2 = root.resolve("r2");
        Files.createDirectories(r1);
        Files.createDirectories(r2);
        writeZip(r1.resolve("a.zip"), Map.of("vehicles/pickup/body.jbeam", "A"));
        writeZip(r2.resolve("b.zip"), Map.of("vehicles/pickup/body.jbeam", "B"));
        assertEquals("B", content(scanAcross(List.of(r1, r2), ConflictStrategy.LATER_ROOT)));
    }

    @Test
    void legacyCommonContainerAcceptsAllEntries() throws IOException {
        write(root.resolve("common/vehicles/common/shared.jbeam"), "{}");
        write(root.resolve("common/loose.jbeam"), "{}"); // legacy: everything under common/ is common
        assertEquals(2, scan("common", ConflictStrategy.LATER_ROOT).entries().size());
    }

    @Test
    void sameCanonicalContainerViaTwoRootsCountsOnce() throws IOException {
        Path veh = root.resolve("veh");
        write(veh.resolve("pickup/vehicles/pickup/body.jbeam"), "{}");
        List<File> roots = List.of(veh.toFile(), veh.toFile());
        NamespaceScan scan = AssetScanner.INSTANCE.scan(roots, "pickup",
                new ConflictPolicy(ConflictStrategy.LATER_ROOT, false));
        assertEquals(1, scan.entries().size());
        assertEquals(1, scan.sources().size());
    }

    @Test
    void conflictChatNotificationIsDeduplicatedPerLogicalPath() throws IOException {
        Path r1 = root.resolve("r1");
        Path r2 = root.resolve("r2");
        write(r1.resolve("a/vehicles/pickup/body.jbeam"), "A");
        write(r2.resolve("b/vehicles/pickup/body.jbeam"), "B");
        List<String> chats = new CopyOnWriteArrayList<>();
        ConflictReporter.INSTANCE.setChatSink(chats::add);
        ConflictPolicy notifyPolicy = new ConflictPolicy(ConflictStrategy.LATER_ROOT, true);

        AssetScanner.INSTANCE.scan(List.of(r1.toFile(), r2.toFile()), "pickup", notifyPolicy);
        AssetScanner.INSTANCE.scan(List.of(r1.toFile(), r2.toFile()), "pickup", notifyPolicy);

        assertEquals(1, chats.size(), "the same conflict path must be notified once");
    }

    private NamespaceScan scan(String ns, ConflictStrategy strategy) {
        return AssetScanner.INSTANCE.scan(List.of(root.toFile()), ns,
                new ConflictPolicy(strategy, false));
    }

    private NamespaceScan scanAcross(List<Path> roots, ConflictStrategy strategy) {
        return AssetScanner.INSTANCE.scan(roots.stream().map(p -> p.toFile()).toList(), "pickup",
                new ConflictPolicy(strategy, false));
    }
}
