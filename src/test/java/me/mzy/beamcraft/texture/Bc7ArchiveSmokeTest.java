package me.mzy.beamcraft.texture;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import java.util.Enumeration;
import java.util.Locale;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Local-only integration smoke test: decodes a bounded sample of <em>real</em>
 * BeamNG BC7 DDS entries straight from the developer's vehicle archives.
 *
 * <p>This is deliberately <b>not</b> part of normal CI: it is skipped unless the
 * archives exist at the configured vehicle directory (system property
 * {@code beamcraft.vehicle.dir}, default {@code run/mods/beamcraft/vehicles} --
 * the same directory the game reads them from), and the vehicle assets
 * themselves are never committed to git. When present, a bounded sample of
 * plain 2D BC7 (DX10 formats 98/99) entries from {@code pickup.zip} and
 * {@code bx.zip} is decoded and every one is asserted to yield valid
 * dimensions, a full RGBA buffer, and no exception.
 */
class Bc7ArchiveSmokeTest {

    private static final int DXGI_BC7_UNORM = 98;
    private static final int DXGI_BC7_UNORM_SRGB = 99;
    private static final int MAX_ENTRIES_PER_ARCHIVE = 30;
    private static final long MAX_ENTRY_BYTES = 8L * 1024 * 1024;

    private static final int DDSCAPS2_CUBEMAP = 0x200;
    private static final int DDSCAPS2_VOLUME = 0x200000;

    @Test
    void decodesBoundedSampleOfRealBc7Entries() throws Exception {
        Path vehicles = vehicleDir();
        File[] archives = {vehicles.resolve("pickup.zip").toFile(), vehicles.resolve("bx.zip").toFile()};
        int present = 0;
        int decoded = 0;
        for (File archive : archives) {
            if (!archive.isFile()) {
                continue;
            }
            present++;
            try (ZipFile zip = new ZipFile(archive)) {
                int sampled = 0;
                Enumeration<? extends ZipEntry> entries = zip.entries();
                while (entries.hasMoreElements() && sampled < MAX_ENTRIES_PER_ARCHIVE) {
                    ZipEntry entry = entries.nextElement();
                    if (entry.isDirectory() || !entry.getName().toLowerCase(Locale.ROOT).endsWith(".dds")) {
                        continue;
                    }
                    if (entry.getSize() > MAX_ENTRY_BYTES) {
                        continue; // unbounded/pathological entry; not representative
                    }
                    byte[] header = readHeader(zip, entry);
                    if (!isPlain2dBc7(header)) {
                        continue; // not a 2D BC7 surface this decoder targets
                    }
                    DecodedImage img = DdsDecoder.decode(readAll(zip, entry));
                    assertNotNull(img, "decode returned null for " + archive.getName() + "!" + entry.getName());
                    assertTrue(img.width() > 0 && img.height() > 0,
                            "non-positive dimensions in " + archive.getName() + "!" + entry.getName());
                    assertEquals((long) img.width() * img.height() * 4, img.copyPixelData().length,
                            "decoded buffer length mismatch in " + archive.getName() + "!" + entry.getName());
                    decoded++;
                    sampled++;
                }
            }
        }
        Assumptions.assumeTrue(present > 0,
                "no BeamNG vehicle archives present; skipping real-data BC7 smoke test");
        assertTrue(decoded > 0, "no decodable plain 2D BC7 DDS entries found in the local archives");
    }

    private static Path vehicleDir() {
        String prop = System.getProperty("beamcraft.vehicle.dir");
        if (prop != null && !prop.isBlank()) {
            return Path.of(prop);
        }
        return Path.of("run", "mods", "beamcraft", "vehicles");
    }

    /** First 148 bytes of a DDS entry (the header), or null if shorter. */
    private static byte[] readHeader(ZipFile zip, ZipEntry entry) throws IOException {
        try (InputStream in = zip.getInputStream(entry)) {
            byte[] header = new byte[148];
            int read = 0;
            while (read < 148) {
                int n = in.read(header, read, 148 - read);
                if (n < 0) {
                    break;
                }
                read += n;
            }
            return read == 148 ? header : null;
        }
    }

    private static byte[] readAll(ZipFile zip, ZipEntry entry) throws IOException {
        try (InputStream in = zip.getInputStream(entry)) {
            return in.readAllBytes();
        }
    }

    /** True for a legacy magic + DX10 header that is a plain 2D BC7 surface. */
    private static boolean isPlain2dBc7(byte[] h) {
        if (h == null || h.length < 148) {
            return false;
        }
        if (readInt(h, 0) != 0x20534444) { // "DDS "
            return false;
        }
        if (readInt(h, 84) != 0x30315844) { // "DX10"
            return false;
        }
        int dxgi = readInt(h, 128);
        if (dxgi != DXGI_BC7_UNORM && dxgi != DXGI_BC7_UNORM_SRGB) {
            return false;
        }
        if (readInt(h, 132) != 3) { // D3D10_RESOURCE_DIMENSION_TEXTURE2D
            return false;
        }
        if (readInt(h, 136) != 0) { // no cubemap misc flag
            return false;
        }
        if (readInt(h, 140) != 1) { // arraySize == 1
            return false;
        }
        if (readInt(h, 24) != 1) { // depth == 1
            return false;
        }
        int caps2 = readInt(h, 112);
        return (caps2 & (DDSCAPS2_CUBEMAP | DDSCAPS2_VOLUME)) == 0;
    }

    private static int readInt(byte[] b, int off) {
        return (b[off] & 0xFF) | ((b[off + 1] & 0xFF) << 8)
                | ((b[off + 2] & 0xFF) << 16) | ((b[off + 3] & 0xFF) << 24);
    }
}
