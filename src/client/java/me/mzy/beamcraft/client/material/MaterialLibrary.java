package me.mzy.beamcraft.client.material;

import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.AssetSource;
import me.mzy.beamcraft.client.assets.NamespaceScan;
import me.mzy.beamcraft.client.assets.ResolvedEntry;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import me.mzy.beamcraft.texture.DecodedImage;
import me.mzy.beamcraft.texture.DecodedTextureCache;
import me.mzy.beamcraft.texture.TextureCompositor;
import me.mzy.beamcraft.texture.TextureDecoder;
import me.mzy.beamcraft.texture.TextureOwnership;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Client-side material library for BeamNG {@code *.materials.json}.
 *
 * <p><b>Discovery scope</b> is delegated to {@link AssetScanner}: every
 * configured asset root is scanned for containers (folders and {@code .zip}
 * archives) that hold entries below {@code vehicles/&lt;namespace&gt;/}; the
 * shared common library is resolved the same way, with the legacy
 * {@code common}/{@code common.zip} all-entries fallback. {@code __MACOSX} junk
 * and directories are skipped, and path conflicts are resolved per the
 * configured {@link me.mzy.beamcraft.client.assets.ConflictPolicy}.
 *
 * <p><b>Indexing rules</b>: each JSON material is indexed by its {@code mapTo}
 * (falling back to the material name when {@code mapTo} is absent). Lookups are
 * case-insensitive while original names are retained in
 * {@link MaterialDefinition} for diagnostics. Vehicle definitions override
 * common definitions for the same {@code mapTo}.
 *
 * <p><b>Static material aliases</b>: alongside the {@code *.materials.json}
 * files, every {@code *.jbeam} file in the same scan is parsed (via the shared
 * {@link RelaxedJson} cleaner) for {@code glowMap} sections and each entry's
 * {@code off} material is indexed as a namespace-scoped static alias
 * {@code glowMap-key -> off-target} (see {@link GlowMapAliasExtractor}). The
 * DAE only knows raw mesh material names (e.g. {@code pickup_lowbeamglass}),
 * which usually have no {@code mapTo} of their own; the alias redirects them to
 * the real lights-off material ({@code pickup_lightglass}). Alias resolution is
 * scoped to the requesting namespace and can never leak across vehicles. This
 * is static only: live emissive switching ({@code on}/{@code on_intense}) and
 * deformation switching ({@code deformMaterialBase}/{@code deformMaterialDamaged})
 * are out of scope — the latter would need a second JBeam field and is the
 * documented gap that keeps the parser from being fully generic.
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

    /**
     * namespace (lowercase) -> aliasKey (lowercase) -> glowMap {@code off}
     * target (lowercase), per loaded vehicle (and for the common namespace).
     * Static lights-off material redirection only; see {@link GlowMapAliasExtractor}.
     */
    private static final Map<String, Map<String, String>> ALIAS_INDEXES = new LinkedHashMap<>();

    /** namespace -> live instance count. */
    private static final Map<String, Integer> REF_COUNTS = new HashMap<>();

    /** namespace (lowercase) -> texture sources owned by that vehicle, to unregister on release. */
    private static final Map<String, List<File>> NAMESPACE_SOURCES = new HashMap<>();

    /** canonical source ids of the shared common sources (never namespace-released). */
    private static final Set<String> COMMON_SOURCE_IDS = new HashSet<>();

    /** namespace (lowercase) -> canonical source ids owned by that vehicle. */
    private static final Map<String, Set<String>> NAMESPACE_SOURCE_IDS = new HashMap<>();

    /**
     * Backend-neutral lifecycle observers notified when a vehicle namespace is
     * actually released (reference count reached zero), after its vehicle-only
     * index, decoded textures and sources are reclaimed. A renderer backend
     * (e.g. the GL texture uploader) registers here to free its own
     * vehicle-only resources; the hook stays free of any GL/Vulkan type so the
     * material model remains backend-neutral.
     */
    private static final List<java.util.function.Consumer<String>> NAMESPACE_RELEASE_LISTENERS = new ArrayList<>();

    /** Registers a namespace-release observer; never called with a null namespace. */
    public static void addNamespaceReleaseListener(java.util.function.Consumer<String> listener) {
        if (listener != null) {
            NAMESPACE_RELEASE_LISTENERS.add(listener);
        }
    }

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
        requireMaterials(List.of(vehiclesRootDir), targetVehicleName);
    }

    /**
     * Multi-root variant: scans every configured asset root for the vehicle's
     * {@code *.materials.json} and {@code *.jbeam} files (plus the shared common
     * library) via {@link AssetScanner}, and registers the contributing
     * containers with the texture locator.
     */
    public static void requireMaterials(List<File> assetRoots, String targetVehicleName) {
        if (assetRoots == null) {
            return;
        }
        boolean anyDirectory = false;
        for (File root : assetRoots) {
            if (root != null && root.isDirectory()) {
                anyDirectory = true;
                break;
            }
        }
        if (!anyDirectory) {
            return; // No usable root; nothing to index.
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
            scanCommon(assetRoots);
            isCommonLoaded = true;
        }
        if (!ns.equals(COMMON_NS)) {
            scanVehicle(assetRoots, ns);
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
            ALIAS_INDEXES.remove(ns);
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
            for (java.util.function.Consumer<String> listener : NAMESPACE_RELEASE_LISTENERS) {
                try {
                    listener.accept(ns);
                } catch (RuntimeException e) {
                    System.err.println("⚠️ [Materials] Namespace release listener failed for " + ns + ": " + e.getMessage());
                }
            }
        } else {
            REF_COUNTS.put(ns, count);
        }
    }

    /**
     * Scoped lookup with static-alias resolution. Resolution order is: the
     * vehicle's own {@code mapTo} first, then the namespace's static glowMap
     * alias target (itself resolved in the vehicle index, then common), then the
     * existing common fallback for the original key. Case-insensitive
     * throughout. Alias resolution is scoped to {@code namespace} and can never
     * leak across vehicle namespaces.
     *
     * @param namespace vehicle namespace, or null to skip the vehicle and alias tiers
     */
    public static MaterialDefinition getMaterial(String namespace, String mapTo) {
        if (mapTo == null || mapTo.isEmpty()) {
            return null;
        }
        String key = mapTo.toLowerCase(Locale.ROOT);
        String ns = namespace == null ? null : namespace.toLowerCase(Locale.ROOT);
        if (ns == null) {
            return COMMON_INDEX.get(key);
        }
        return resolveMaterial(VEHICLE_INDEXES.get(ns), ALIAS_INDEXES.get(ns), COMMON_INDEX, key);
    }

    /**
     * Pure, unit-tested material resolution. Order: the vehicle's own
     * {@code mapTo} first, then the namespace's static glowMap alias target
     * (resolved in the vehicle index, then common), then the existing common
     * fallback for the original key. {@code aliasIndex} is the requesting
     * namespace's alias map only, so aliases can never leak across vehicle
     * namespaces. Any of the maps may be null.
     */
    static MaterialDefinition resolveMaterial(Map<String, MaterialDefinition> vehicleIndex,
                                              Map<String, String> aliasIndex,
                                              Map<String, MaterialDefinition> commonIndex,
                                              String key) {
        MaterialDefinition def = vehicleIndex == null ? null : vehicleIndex.get(key);
        if (def != null) {
            return def; // 1. direct mapTo in the vehicle namespace
        }
        String aliasTarget = aliasIndex == null ? null : aliasIndex.get(key);
        if (aliasTarget != null) {
            def = vehicleIndex == null ? null : vehicleIndex.get(aliasTarget);
            if (def != null) {
                return def; // 2a. alias target in the vehicle namespace
            }
            def = commonIndex == null ? null : commonIndex.get(aliasTarget);
            if (def != null) {
                return def; // 2b. alias target in common
            }
        }
        return commonIndex == null ? null : commonIndex.get(key); // 3. common fallback for the direct key
    }

    /**
     * Unqualified lookup across all loaded vehicles in require order, then
     * common. With several vehicles loaded at once a {@code mapTo} may collide
     * across namespaces; prefer the scoped
     * {@link #getMaterial(String, String)} when the vehicle is known. Static
     * glowMap aliases are deliberately <em>not</em> resolved here: the caller
     * has no namespace context, so applying aliases could leak one vehicle's
     * aliases into another. Use the scoped lookup for alias resolution.
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

    /** Total registered static glowMap aliases across all namespaces (diagnostic). */
    public static int getAliasCount() {
        int total = 0;
        for (Map<String, String> index : ALIAS_INDEXES.values()) {
            total += index.size();
        }
        return total;
    }

    /** Namespaces currently holding a static alias index (unmodifiable). */
    public static Set<String> getAliasNamespaces() {
        return Collections.unmodifiableSet(ALIAS_INDEXES.keySet());
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
     * Decodes a resolved DDS, PNG, or JPEG texture to a backend-neutral RGBA8
     * image and retains it in the decoded-texture cache. The returned image
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
        return DECODED_TEXTURES.acquire(resource, ownership, key -> TextureDecoder.decode(LOCATOR.readBytes(key)));
    }

    /**
     * Releases one prior {@link #acquireDecodedTexture} of {@code resource}.
     * Idempotent; safe to call for textures never acquired.
     */
    public static void releaseDecodedTexture(TextureResource resource) {
        DECODED_TEXTURES.release(resource);
    }

    /**
     * Resolves the lifecycle ownership of {@code resource} for the given
     * requesting namespace, mirroring the ownership used internally by
     * {@link #acquireDecodedTexture}: the requesting namespace for that
     * namespace's own sources, otherwise {@code null} (shared/common, durable).
     * A renderer backend uses this to know whether its cached upload of a
     * texture should be reclaimed when the namespace is released.
     *
     * @param resource  an opaque handle from {@link #resolveTexture}
     * @param namespace the namespace acquiring the texture (may be null)
     * @return the owning namespace, or null for shared/common/unknown sources
     */
    public static String resolveTextureOwnership(TextureResource resource, String namespace) {
        if (resource == null) {
            return null;
        }
        String ns = namespace == null ? null : namespace.toLowerCase(Locale.ROOT);
        return TextureOwnership.resolve(resource.sourceId(), COMMON_SOURCE_IDS, ns,
                ns == null ? null : NAMESPACE_SOURCE_IDS.get(ns));
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

    private static void scanCommon(List<File> roots) {
        NamespaceScan scan = AssetScanner.INSTANCE.scan(roots, COMMON_NS);
        for (ResolvedEntry entry : scan.entries()) {
            processScanEntry(entry, COMMON_NS, true);
        }
        for (AssetSource source : scan.sources()) {
            LOCATOR.registerSource(source.file());
            COMMON_SOURCE_IDS.add(canonicalPath(source.file()));
        }
    }

    private static void scanVehicle(List<File> roots, String ns) {
        // TODO(texture-vs-conflict-strategy): Texture paths are resolved by
        // TextureResourceLocator's first-registered-source order, independently
        // of the AssetScanner conflict strategy. This only diverges when two
        // registered containers share a texture path with different content and
        // BOTH hold a winning entry (partial vehicle overlap across roots); a
        // full override is consistent because the shadowed root is never
        // registered. Fix: expose the per-logical-path winner from AssetScanner
        // and have the locator prefer that source (needs mtime for "newer").
        NamespaceScan scan = AssetScanner.INSTANCE.scan(roots, ns);
        List<File> ownedSources = new ArrayList<>();
        Set<String> ownedSourceIds = new HashSet<>();
        for (ResolvedEntry entry : scan.entries()) {
            processScanEntry(entry, ns, false);
        }
        for (AssetSource source : scan.sources()) {
            LOCATOR.registerSource(source.file());
            ownedSources.add(source.file());
            ownedSourceIds.add(canonicalPath(source.file()));
        }
        NAMESPACE_SOURCES.put(ns, ownedSources);
        NAMESPACE_SOURCE_IDS.put(ns, ownedSourceIds);
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
     * Consumes one conflict-resolved entry into the material/alias index. The
     * caller registers the entry's container with the locator separately, so a
     * container that contributed only shadowed entries is never registered.
     */
    private static void processScanEntry(ResolvedEntry entry, String namespace, boolean isCommon) {
        String lower = entry.logicalPath();
        if (!isIndexedFile(lower)) {
            return;
        }
        String source = entry.sourceAddress();
        try (InputStream in = entry.open()) {
            processIndexedFile(in, source, lower, namespace, isCommon);
        } catch (Exception e) {
            System.err.println("⚠️ [Materials] Failed to read " + source + ": " + e.getMessage());
        }
    }

    /** A file that feeds the material index: materials JSON or JBeam (for glowMap aliases). */
    private static boolean isIndexedFile(String lowerName) {
        return lowerName.endsWith(".materials.json") || lowerName.endsWith(".jbeam");
    }

    private static void processIndexedFile(InputStream in, String source, String lowerName,
                                           String namespace, boolean isCommon) throws IOException {
        if (lowerName.endsWith(".jbeam")) {
            registerJBeamAliases(in, namespace);
        } else {
            parseMaterialsFile(in, source, namespace, isCommon);
        }
    }

    /**
     * Reads one JBeam file and registers its {@code glowMap} {@code off} aliases
     * into the namespace's static alias index. Malformed content contributes
     * nothing and never aborts the scan (see {@link GlowMapAliasExtractor}).
     */
    private static void registerJBeamAliases(InputStream in, String namespace) throws IOException {
        String content = new String(in.readAllBytes(), StandardCharsets.UTF_8);
        Map<String, String> index = ALIAS_INDEXES.computeIfAbsent(
                namespace.toLowerCase(Locale.ROOT), k -> new HashMap<>());
        GlowMapAliasExtractor.collectFromJBeam(content, index);
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
