package me.mzy.beamcraft.client.material;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.texture.DecodedImage;
import me.mzy.beamcraft.texture.DecodedTextureCache;
import me.mzy.beamcraft.texture.DdsDecoder;
import me.mzy.beamcraft.texture.TextureCompositor;
import me.mzy.beamcraft.texture.TextureOwnership;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * Client-side material library for BeamNG {@code *.materials.json}.
 *
 * <p><b>Discovery scope</b> mirrors {@code DaeMeshLoader} exactly: the
 * {@code common.zip}/{@code common/} pair is scanned for the shared library,
 * then every file under the vehicles root whose name contains the requested
 * vehicle name (case-insensitive) is scanned as a ZIP archive or loose folder.
 * Only entries below {@code vehicles/&lt;namespace&gt;/} are considered for a
 * vehicle; {@code __MACOSX} junk and directories are skipped.
 *
 * <p><b>Indexing rules</b>: each JSON material is indexed by its {@code mapTo}
 * (falling back to the material name when {@code mapTo} is absent). Lookups are
 * case-insensitive while original names are retained in
 * {@link MaterialDefinition} for diagnostics. Vehicle definitions override
 * common definitions for the same {@code mapTo}.
 *
 * <p><b>Lifecycle</b>: {@link #requireMaterials} / {@link #releaseMaterials}
 * follow the same reference-counting scheme as
 * {@code DaeMeshLoader.requireVehicleModels}. Concurrent instances of the same
 * vehicle share one index; when the last instance is released, the vehicle-only
 * index is reclaimed and the vehicle's texture sources are unregistered from the
 * locator (dropping their lazily built ZIP indexes), so requiring the vehicle
 * again rescans and reindexes the current archives. Common stays loaded once the
 * first vehicle asks for it, matching the current DAE behaviour.
 *
 * <p>Malformed individual material files are logged with their archive/path
 * location and skipped; they never prevent other material files or meshes from
 * loading.
 */
public final class MaterialLibrary {

    private static final String COMMON_NS = "common";
    private static final int MAX_STAGES = 4;

    /** mapTo (lowercase) -> definition for the shared common library. */
    private static final Map<String, MaterialDefinition> COMMON_INDEX = new HashMap<>();

    /** namespace (lowercase) -> mapTo (lowercase) -> definition, per loaded vehicle. */
    private static final Map<String, Map<String, MaterialDefinition>> VEHICLE_INDEXES = new LinkedHashMap<>();

    /** namespace -> live instance count. */
    private static final Map<String, Integer> REF_COUNTS = new HashMap<>();

    /** namespace (lowercase) -> texture sources owned by that vehicle, to unregister on release. */
    private static final Map<String, List<File>> NAMESPACE_SOURCES = new HashMap<>();

    /** canonical source ids of the shared common sources (never namespace-released). */
    private static final Set<String> COMMON_SOURCE_IDS = new HashSet<>();

    /** namespace (lowercase) -> canonical source ids owned by that vehicle. */
    private static final Map<String, Set<String>> NAMESPACE_SOURCE_IDS = new HashMap<>();

    /**
     * Decoded-texture cache keyed by {@link TextureResource}. Acquired images
     * are pinned until released; vehicle-only entries are evicted when their
     * namespace's last reference releases, while common/shared entries stay.
     * See {@link DecodedTextureCache} for the exact contract.
     */
    private static final DecodedTextureCache<TextureResource> DECODED_TEXTURES = new DecodedTextureCache<>(1024);

    private static final TextureResourceLocator LOCATOR = new TextureResourceLocator();

    private static boolean isCommonLoaded = false;

    private MaterialLibrary() {
    }

    /**
     * Loads (once) the material indexes for {@code targetVehicleName} and the
     * common library, registering their archives/folders with the texture
     * locator. Safe to call repeatedly for the same vehicle.
     */
    public static void requireMaterials(File vehiclesRootDir, String targetVehicleName) {
        if (vehiclesRootDir == null || !vehiclesRootDir.isDirectory()) {
            return;
        }
        String ns = targetVehicleName.toLowerCase(Locale.ROOT);
        int count = REF_COUNTS.getOrDefault(ns, 0);
        REF_COUNTS.put(ns, count + 1);
        if (count > 0) {
            return; // Already indexed for a live instance.
        }

        if (!ns.equals(COMMON_NS)) {
            DECODED_TEXTURES.retainNamespace(ns);
        }
        if (!isCommonLoaded) {
            scanCommon(vehiclesRootDir);
            isCommonLoaded = true;
        }
        if (!ns.equals(COMMON_NS)) {
            scanVehicle(vehiclesRootDir, targetVehicleName);
        }
    }

    /**
     * Releases one instance of a vehicle's materials. When the reference count
     * reaches zero the vehicle-only index and the vehicle's texture sources are
     * reclaimed; common stays shared.
     */
    public static void releaseMaterials(String targetVehicleName) {
        String ns = targetVehicleName.toLowerCase(Locale.ROOT);
        int count = REF_COUNTS.getOrDefault(ns, 0) - 1;
        if (count <= 0) {
            REF_COUNTS.remove(ns);
            VEHICLE_INDEXES.remove(ns);
            NAMESPACE_SOURCE_IDS.remove(ns);
            if (!ns.equals(COMMON_NS)) {
                DECODED_TEXTURES.releaseNamespace(ns);
            }
            List<File> sources = NAMESPACE_SOURCES.remove(ns);
            if (sources != null) {
                for (File source : sources) {
                    LOCATOR.unregisterSource(source);
                }
            }
        } else {
            REF_COUNTS.put(ns, count);
        }
    }

    /**
     * Scoped lookup: the vehicle's own definitions first, then common.
     * Case-insensitive on {@code mapTo}.
     *
     * @param namespace vehicle namespace, or null to skip the vehicle tier
     */
    public static MaterialDefinition getMaterial(String namespace, String mapTo) {
        if (mapTo == null || mapTo.isEmpty()) {
            return null;
        }
        String key = mapTo.toLowerCase(Locale.ROOT);
        if (namespace != null) {
            Map<String, MaterialDefinition> vehicleIndex =
                    VEHICLE_INDEXES.get(namespace.toLowerCase(Locale.ROOT));
            if (vehicleIndex != null) {
                MaterialDefinition def = vehicleIndex.get(key);
                if (def != null) {
                    return def;
                }
            }
        }
        return COMMON_INDEX.get(key);
    }

    /**
     * Unqualified lookup across all loaded vehicles in require order, then
     * common. With several vehicles loaded at once a {@code mapTo} may collide
     * across namespaces; prefer the scoped
     * {@link #getMaterial(String, String)} when the vehicle is known.
     */
    public static MaterialDefinition getMaterial(String mapTo) {
        if (mapTo == null || mapTo.isEmpty()) {
            return null;
        }
        String key = mapTo.toLowerCase(Locale.ROOT);
        for (Map<String, MaterialDefinition> index : VEHICLE_INDEXES.values()) {
            MaterialDefinition def = index.get(key);
            if (def != null) {
                return def;
            }
        }
        return COMMON_INDEX.get(key);
    }

    /** Total indexed material definitions (diagnostic). */
    public static int getMaterialCount() {
        int total = COMMON_INDEX.size();
        for (Map<String, MaterialDefinition> index : VEHICLE_INDEXES.values()) {
            total += index.size();
        }
        return total;
    }

    /** Namespaces currently holding a live vehicle index (unmodifiable). */
    public static Set<String> getLoadedNamespaces() {
        return Collections.unmodifiableSet(VEHICLE_INDEXES.keySet());
    }

    /** Indexed texture sources (diagnostic). */
    public static int getTextureSourceCount() {
        return LOCATOR.getSourceCount();
    }

    /**
     * Resolves a logical texture path (e.g. {@code /vehicles/foo/foo_d.png}) to
     * a stable, library-owned {@link TextureResource} handle for later reading,
     * or returns null when no indexed source contains it (or the path is
     * unsafe). The returned handle is opaque and cannot be constructed by
     * callers; pass it back to {@link #readTexture(TextureResource)} to obtain
     * the bytes.
     */
    public static TextureResource resolveTexture(String logicalPath) {
        return LOCATOR.resolve(logicalPath);
    }

    /**
     * Reads the full bytes of a texture previously obtained from
     * {@link #resolveTexture}. Throws when the resource cannot be read.
     */
    public static byte[] readTexture(TextureResource resource) throws IOException {
        return LOCATOR.readBytes(resource);
    }

    /**
     * Convenience: resolves then reads in one call. Returns null when the path
     * cannot be resolved (including unsafe paths); throws when resolution
     * succeeded but reading failed.
     */
    public static byte[] readTexture(String logicalPath) throws IOException {
        TextureResource resource = LOCATOR.resolve(logicalPath);
        return resource == null ? null : LOCATOR.readBytes(resource);
    }

    // ------------------------------------------------------------------
    // Decoded textures (cache + composition)
    // ------------------------------------------------------------------

    /**
     * Decodes a resolved texture to an RGBA8 image via the backend-neutral DDS
     * decoder and retains it in the decoded-texture cache. The returned image
     * is pinned; call {@link #releaseDecodedTexture} when done. The image is
     * owned by the cache (or by a later renderer uploader) and must not be
     * mutated.
     *
     * @param resource  an opaque handle from {@link #resolveTexture}
     * @param namespace the vehicle namespace acquiring the texture (used to
     *                  decide lifecycle ownership); may be null
     * @return the decoded image, never null
     * @throws IOException if the texture cannot be read or decoded; nothing is
     *                     cached on failure
     */
    public static DecodedImage acquireDecodedTexture(TextureResource resource, String namespace) throws IOException {
        if (resource == null) {
            throw new IOException("cannot decode a null texture resource");
        }
        String ns = namespace == null ? null : namespace.toLowerCase(Locale.ROOT);
        String ownership = TextureOwnership.resolve(resource.sourceId(), COMMON_SOURCE_IDS, ns,
                ns == null ? null : NAMESPACE_SOURCE_IDS.get(ns));
        return DECODED_TEXTURES.acquire(resource, ownership, key -> DdsDecoder.decode(LOCATOR.readBytes(key)));
    }

    /**
     * Releases one prior {@link #acquireDecodedTexture} of {@code resource}.
     * Idempotent; safe to call for textures never acquired.
     */
    public static void releaseDecodedTexture(TextureResource resource) {
        DECODED_TEXTURES.release(resource);
    }

    /**
     * Convenience: acquires the diffuse and opacity textures, composes them
     * (opacity multiplies the diffuse alpha, see {@link TextureCompositor}),
     * releases both acquires and returns the composed image. The result is a
     * fresh image that is <em>not</em> cached and is owned by the caller.
     *
     * @param diffuse   base-colour texture handle
     * @param opacity   single-channel opacity texture handle
     * @param namespace vehicle namespace for lifecycle ownership
     * @return the composed RGBA image (caller-owned, not cached)
     * @throws IOException if either texture cannot be decoded, or the
     *                     dimensions mismatch (as {@link IllegalArgumentException})
     */
    public static DecodedImage composeDiffuseAndOpacity(TextureResource diffuse, TextureResource opacity,
                                                        String namespace) throws IOException {
        DecodedImage base = acquireDecodedTexture(diffuse, namespace);
        DecodedImage opacityImage;
        try {
            opacityImage = acquireDecodedTexture(opacity, namespace);
        } catch (IOException | RuntimeException e) {
            releaseDecodedTexture(diffuse);
            throw e;
        }
        try {
            return TextureCompositor.composeBaseWithOpacity(base, opacityImage);
        } finally {
            releaseDecodedTexture(opacity);
            releaseDecodedTexture(diffuse);
        }
    }

    /** Decoded textures currently retained by the cache (diagnostic). */
    public static int getDecodedTextureCount() {
        return DECODED_TEXTURES.size();
    }

    /** Decoded textures still pinned by an acquire (diagnostic). */
    public static int getDecodedTexturePinnedCount() {
        return DECODED_TEXTURES.pinnedCount();
    }

    // ------------------------------------------------------------------
    // Scanning
    // ------------------------------------------------------------------

    private static void scanCommon(File vehiclesRootDir) {
        File commonZip = new File(vehiclesRootDir, COMMON_NS + ".zip");
        File commonDir = new File(vehiclesRootDir, COMMON_NS);
        if (commonZip.exists()) {
            LOCATOR.registerSource(commonZip);
            COMMON_SOURCE_IDS.add(canonicalPath(commonZip));
            scanZipForMaterials(commonZip, COMMON_NS, true);
        }
        if (commonDir.isDirectory()) {
            LOCATOR.registerSource(commonDir);
            COMMON_SOURCE_IDS.add(canonicalPath(commonDir));
            scanFolderForMaterials(commonDir, COMMON_NS, true);
        }
    }

    private static void scanVehicle(File vehiclesRootDir, String targetVehicleName) {
        File[] files = vehiclesRootDir.listFiles();
        if (files == null) {
            return;
        }
        List<File> ownedSources = new ArrayList<>();
        Set<String> ownedSourceIds = new HashSet<>();
        for (File file : files) {
            String name = file.getName();
            if (name.equals(COMMON_NS + ".zip") || name.equals(COMMON_NS)) {
                continue;
            }
            if (!name.toLowerCase(Locale.ROOT).contains(targetVehicleName.toLowerCase(Locale.ROOT))) {
                continue;
            }
            if (file.isDirectory()) {
                LOCATOR.registerSource(file);
                ownedSources.add(file);
                ownedSourceIds.add(canonicalPath(file));
                scanFolderForMaterials(file, targetVehicleName, false);
            } else if (name.toLowerCase(Locale.ROOT).endsWith(".zip")) {
                LOCATOR.registerSource(file);
                ownedSources.add(file);
                ownedSourceIds.add(canonicalPath(file));
                scanZipForMaterials(file, targetVehicleName, false);
            }
        }
        NAMESPACE_SOURCES.put(targetVehicleName.toLowerCase(Locale.ROOT), ownedSources);
        NAMESPACE_SOURCE_IDS.put(targetVehicleName.toLowerCase(Locale.ROOT), ownedSourceIds);
    }

    /** Canonical absolute path, or null when not resolvable. */
    private static String canonicalPath(File file) {
        try {
            return file.getCanonicalPath();
        } catch (IOException e) {
            return file.getAbsolutePath();
        }
    }

    /**
     * True when {@code path} (a ZIP entry name or a filesystem path) sits below
     * a {@code vehicles/<namespace>/} directory. Matching is case-insensitive
     * and backslashes are treated as path separators, mirroring how the texture
     * locator resolves logical paths. The {@code vehicles/<namespace>/} segment
     * requirement is unchanged from a literal match; only the segment boundary
     * is enforced, so a {@code <something>vehicles/<namespace>/} entry does not
     * qualify.
     */
    private static boolean underVehiclesNamespace(String path, String namespace) {
        String normalized = path.replace('\\', '/').toLowerCase(Locale.ROOT);
        String ns = namespace.toLowerCase(Locale.ROOT);
        return normalized.contains("/vehicles/" + ns + "/")
                || normalized.startsWith("vehicles/" + ns + "/");
    }

    private static void scanZipForMaterials(File zipFile, String namespace, boolean isCommon) {
        try (ZipFile zf = new ZipFile(zipFile)) {
            Enumeration<? extends ZipEntry> entries = zf.entries();
            while (entries.hasMoreElements()) {
                ZipEntry entry = entries.nextElement();
                String entryName = entry.getName();
                if (entry.isDirectory() || entryName.contains("__MACOSX")) {
                    continue;
                }
                boolean isTarget = isCommon || underVehiclesNamespace(entryName, namespace);
                if (!isTarget || !entryName.toLowerCase(Locale.ROOT).endsWith(".materials.json")) {
                    continue;
                }
                String source = zipFile.getAbsolutePath() + "!" + entryName;
                try (InputStream in = zf.getInputStream(entry)) {
                    parseMaterialsFile(in, source, namespace, isCommon);
                } catch (Exception e) {
                    System.err.println("⚠️ [Materials] Failed to read " + source + ": " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.err.println("⚠️ [Materials] Failed to scan ZIP " + zipFile.getName() + ": " + e.getMessage());
        }
    }

    private static void scanFolderForMaterials(File folder, String namespace, boolean isCommon) {
        try (Stream<Path> paths = Files.walk(folder.toPath())) {
            paths.filter(Files::isRegularFile).forEach(path -> {
                String filePath = path.toString().replace('\\', '/');
                boolean isTarget = isCommon || underVehiclesNamespace(filePath, namespace);
                if (!isTarget || !filePath.toLowerCase(Locale.ROOT).endsWith(".materials.json")) {
                    return;
                }
                String source = path.toAbsolutePath().toString();
                try (InputStream in = Files.newInputStream(path)) {
                    parseMaterialsFile(in, source, namespace, isCommon);
                } catch (Exception e) {
                    System.err.println("⚠️ [Materials] Failed to read " + source + ": " + e.getMessage());
                }
            });
        } catch (Exception e) {
            System.err.println("⚠️ [Materials] Failed to walk folder " + folder.getName() + ": " + e.getMessage());
        }
    }

    private static void parseMaterialsFile(InputStream in, String source, String namespace, boolean isCommon)
            throws IOException {
        String content = new String(in.readAllBytes(), StandardCharsets.UTF_8);
        JsonObject root = RelaxedJson.parse(content);
        Map<String, MaterialDefinition> index = indexFor(namespace, isCommon);
        for (Map.Entry<String, JsonElement> entry : root.entrySet()) {
            String materialName = entry.getKey();
            JsonElement value = entry.getValue();
            if (value == null || !value.isJsonObject()) {
                System.err.println("⚠️ [Materials] " + source + ": material '" + materialName
                        + "' is not an object, skipping");
                continue;
            }
            try {
                MaterialDefinition definition = MaterialDefinition.fromJson(materialName, value.getAsJsonObject(), source);
                if (definition == null) {
                    continue;
                }
                index.put(definition.mapTo.toLowerCase(Locale.ROOT), definition);
            } catch (Exception e) {
                System.err.println("⚠️ [Materials] " + source + ": failed to parse material '"
                        + materialName + "': " + e.getMessage());
            }
        }
    }

    private static Map<String, MaterialDefinition> indexFor(String namespace, boolean isCommon) {
        if (isCommon) {
            return COMMON_INDEX;
        }
        return VEHICLE_INDEXES.computeIfAbsent(namespace.toLowerCase(Locale.ROOT), k -> new HashMap<>());
    }
}
