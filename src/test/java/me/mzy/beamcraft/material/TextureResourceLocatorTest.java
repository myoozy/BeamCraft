package me.mzy.beamcraft.client.material;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Direct tests for {@link TextureResourceLocator} — the class every material texture
 * path resolves through, which was only ever covered indirectly.
 *
 * <p>Focused on BeamNG's {@code @name} shorthand: a material stage saying
 * {@code "@licenseplate-default"} names the tail of a file name, not a path, and the
 * stock assets ship it as {@code premade@licenseplate-default.dds}. Next to it, the
 * plain path and {@code .png} rules it has to keep working.
 */
class TextureResourceLocatorTest {

    @TempDir
    Path root;

    private static TextureResourceLocator locatorFor(Path container) {
        TextureResourceLocator locator = new TextureResourceLocator();
        locator.registerSource(container.toFile());
        return locator;
    }

    private static void write(Path container, String relative, String content) throws IOException {
        Path file = container.resolve(relative);
        Files.createDirectories(file.getParent());
        Files.writeString(file, content, StandardCharsets.UTF_8);
    }

    private static void writeZip(Path zip, Map<String, String> entries) throws IOException {
        try (ZipOutputStream out = new ZipOutputStream(Files.newOutputStream(zip))) {
            for (Map.Entry<String, String> entry : entries.entrySet()) {
                out.putNextEntry(new ZipEntry(entry.getKey()));
                out.write(entry.getValue().getBytes(StandardCharsets.UTF_8));
                out.closeEntry();
            }
        }
    }

    @Test
    void resolvesAPlainPathToTheRealFile() throws IOException {
        write(root, "vehicles/x/body.dds", "body");
        TextureResourceLocator locator = locatorFor(root);

        TextureResource resource = locator.resolve("/vehicles/x/body.dds");

        assertNotNull(resource);
        assertEquals("vehicles/x/body.dds", resource.entryPath());
        assertArrayEquals("body".getBytes(StandardCharsets.UTF_8), locator.readBytes(resource));
    }

    @Test
    void resolvesAnAtReferenceThroughTheFileNameTail() throws IOException {
        write(root, "vehicles/common/licenseplates/premade@licenseplate-default.dds", "plate");
        TextureResourceLocator locator = locatorFor(root);

        TextureResource resource = locator.resolve("@licenseplate-default");

        assertNotNull(resource, "@name must resolve through the file-name tail");
        assertEquals("vehicles/common/licenseplates/premade@licenseplate-default.dds", resource.entryPath());
        assertArrayEquals("plate".getBytes(StandardCharsets.UTF_8), locator.readBytes(resource));
    }

    @Test
    void prefersTheBuiltDdsOverTheSourceImage() throws IOException {
        write(root, "vehicles/x/premade@screen.png", "png");
        write(root, "vehicles/x/premade@screen.dds", "dds");

        TextureResource resource = locatorFor(root).resolve("@screen");

        assertNotNull(resource);
        assertEquals("vehicles/x/premade@screen.dds", resource.entryPath());
    }

    @Test
    void acceptsAnAtReferenceThatCarriesAnExtension() throws IOException {
        write(root, "vehicles/x/premade@screen.dds", "dds");

        TextureResource resource = locatorFor(root).resolve("@screen.png");

        assertNotNull(resource, "a suffix on the reference must not defeat the tail match");
        assertEquals("vehicles/x/premade@screen.dds", resource.entryPath());
    }

    @Test
    void doesNotMatchALongerNameThatMerelySharesThePrefix() throws IOException {
        write(root, "vehicles/x/premade@licenseplate-default-specular.dds", "spec");

        assertNull(locatorFor(root).resolve("@licenseplate-default"),
                "-specular is a different texture, not the one referenced");
    }

    @Test
    void picksTheLexicographicallyFirstMatchRegardlessOfIndexOrder() throws IOException {
        // Same tail in two containers built in opposite orders: the pick must come from
        // the rule, not from hash iteration order.
        Path first = root.resolve("first");
        write(first, "vehicles/x/a@wheel.dds", "a");
        write(first, "vehicles/x/b@wheel.dds", "b");

        Path second = root.resolve("second");
        write(second, "vehicles/x/b@wheel.dds", "b");
        write(second, "vehicles/x/a@wheel.dds", "a");

        assertEquals("vehicles/x/a@wheel.dds", locatorFor(first).resolve("@wheel").entryPath());
        assertEquals("vehicles/x/a@wheel.dds", locatorFor(second).resolve("@wheel").entryPath());
    }

    @Test
    void resolvesAnAtReferenceInsideAZip() throws IOException {
        Path zip = root.resolve("common.zip");
        writeZip(zip, Map.of("vehicles/common/premade@plate.dds", "zipped"));

        TextureResource resource = locatorFor(zip).resolve("@plate");

        assertNotNull(resource);
        assertEquals("vehicles/common/premade@plate.dds", resource.entryPath());
        assertTrue(resource.zip());
    }

    @Test
    void unknownAtReferenceResolvesToNull() throws IOException {
        write(root, "vehicles/x/other.dds", "x");

        assertNull(locatorFor(root).resolve("@missing"));
    }

    @Test
    void atReferenceInAContainerThatDoesNotUseItStaysUnresolved() throws IOException {
        // The folder index is only built when the shorthand is used; a container with
        // ordinary files must not accidentally satisfy an @name.
        write(root, "vehicles/x/licenseplate-default.dds", "plain");

        assertNull(locatorFor(root).resolve("@licenseplate-default"),
                "the tail must include the @ marker");
    }
}
