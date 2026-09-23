package me.mzy.beamcraft.client.assets;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VehicleCatalogTest {
    @TempDir
    Path tempDir;

    @Test
    void indexesFolderAndZipContainersWithoutParsingTheirContents() throws IOException {
        Path folderContainer = tempDir.resolve("folder-mod");
        write(folderContainer.resolve("vehicles/Pickup/pickup.jbeam"));
        write(folderContainer.resolve("vehicles/Pickup/D15.pc"));
        write(folderContainer.resolve("vehicles/Pickup/offroad.PC"));
        write(folderContainer.resolve("vehicles/common/shared.jbeam"));
        write(folderContainer.resolve("vehicles/mesh_only/model.dae"));

        Path zipContainer = tempDir.resolve("wrapped-mod.zip");
        try (ZipOutputStream zip = new ZipOutputStream(Files.newOutputStream(zipContainer))) {
            add(zip, "wrapped/vehicles/covet/covet.jbeam");
            add(zip, "wrapped/vehicles/covet/DX.pc");
        }

        VehicleCatalog catalog = VehicleCatalog.scan(List.of(tempDir.toFile()));

        assertEquals(List.of("covet", "pickup"), catalog.vehicleNames());
        assertEquals(List.of("DX.pc"), catalog.pcFiles("COVET"));
        assertEquals(List.of("D15.pc", "offroad.PC"), catalog.pcFiles("pickup"));
        assertEquals(List.of(), catalog.pcFiles("common"));
        assertEquals(List.of(), catalog.pcFiles("mesh_only"));
    }

    private static void write(Path path) throws IOException {
        Files.createDirectories(path.getParent());
        Files.writeString(path, "not parsed");
    }

    private static void add(ZipOutputStream zip, String path) throws IOException {
        zip.putNextEntry(new ZipEntry(path));
        zip.write("not parsed".getBytes());
        zip.closeEntry();
    }
}
