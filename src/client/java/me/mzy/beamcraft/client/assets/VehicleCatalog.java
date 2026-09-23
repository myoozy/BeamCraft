package me.mzy.beamcraft.client.assets;

import me.mzy.beamcraft.BeamCraft;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * Lightweight client-side index used by command completion.
 *
 * <p>Only path names are inspected; JBeam and PC contents are not parsed. The
 * result is immutable and intended to be built once after asset roots have been
 * configured, rather than rescanning folders and archives for every Tab press.
 */
public final class VehicleCatalog {
    private final Map<String, List<String>> pcFilesByVehicle;

    private VehicleCatalog(Map<String, List<String>> pcFilesByVehicle) {
        this.pcFilesByVehicle = pcFilesByVehicle;
    }

    public static VehicleCatalog scan(List<File> roots) {
        Map<String, MutableVehicle> discovered = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
        Set<String> seenContainers = new HashSet<>();

        if (roots != null) {
            for (File root : roots) {
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
                for (File container : containers) {
                    String id = AssetScanner.canonicalPath(container);
                    if (id != null && seenContainers.add(id)) {
                        scanContainer(container, discovered);
                    }
                }
            }
        }

        Map<String, List<String>> result = new LinkedHashMap<>();
        for (Map.Entry<String, MutableVehicle> entry : discovered.entrySet()) {
            MutableVehicle vehicle = entry.getValue();
            if (vehicle.hasJbeam) {
                result.put(entry.getKey(), List.copyOf(vehicle.pcFiles.values()));
            }
        }
        return new VehicleCatalog(Collections.unmodifiableMap(new LinkedHashMap<>(result)));
    }

    public List<String> vehicleNames() {
        return List.copyOf(pcFilesByVehicle.keySet());
    }

    public List<String> pcFiles(String vehicleName) {
        if (vehicleName == null) {
            return List.of();
        }
        for (Map.Entry<String, List<String>> entry : pcFilesByVehicle.entrySet()) {
            if (entry.getKey().equalsIgnoreCase(vehicleName)) {
                return entry.getValue();
            }
        }
        return List.of();
    }

    private static void scanContainer(File container, Map<String, MutableVehicle> discovered) {
        try {
            if (container.isFile()) {
                try (ZipFile zip = new ZipFile(container)) {
                    var entries = zip.entries();
                    while (entries.hasMoreElements()) {
                        ZipEntry entry = entries.nextElement();
                        if (!entry.isDirectory() && !entry.getName().contains("__MACOSX")) {
                            collect(entry.getName(), discovered);
                        }
                    }
                }
            } else {
                Path root = container.toPath().toAbsolutePath().normalize();
                try (Stream<Path> paths = Files.walk(root)) {
                    paths.filter(Files::isRegularFile).forEach(path ->
                            collect(root.relativize(path.toAbsolutePath().normalize()).toString(), discovered));
                }
            }
        } catch (IOException e) {
            BeamCraft.LOGGER.warn("Failed to index vehicle assets in {}", container.getAbsolutePath(), e);
        }
    }

    private static void collect(String rawPath, Map<String, MutableVehicle> discovered) {
        String path = rawPath.replace('\\', '/');
        String[] segments = path.split("/");
        for (int i = 0; i + 2 < segments.length; i++) {
            if (!segments[i].equalsIgnoreCase("vehicles")) {
                continue;
            }
            String namespace = segments[i + 1].toLowerCase(Locale.ROOT);
            if (namespace.isBlank() || namespace.equals("common")) {
                return;
            }
            String fileName = segments[segments.length - 1];
            String lowerFileName = fileName.toLowerCase(Locale.ROOT);
            MutableVehicle vehicle = discovered.computeIfAbsent(namespace, ignored -> new MutableVehicle());
            if (lowerFileName.endsWith(".jbeam")) {
                vehicle.hasJbeam = true;
            } else if (lowerFileName.endsWith(".pc")) {
                vehicle.pcFiles.putIfAbsent(lowerFileName, fileName);
            }
            return;
        }
    }

    private static final class MutableVehicle {
        private boolean hasJbeam;
        private final Map<String, String> pcFiles = new TreeMap<>(String.CASE_INSENSITIVE_ORDER);
    }
}
