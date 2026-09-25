package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.ResolvedEntry;
import me.mzy.beamcraft.client.debug.LoadTiming;

import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.assimp.*;
import org.lwjgl.PointerBuffer;

import java.io.File;
import java.nio.IntBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

public class DaeMeshLoader {

    /** The shared library namespace every vehicle resolves against. */
    private static final String COMMON = "common";

    public static class SubMesh {
        public String materialName;
        public int startIndex;
        public int indexCount;

        public SubMesh(String materialName, int startIndex, int indexCount) {
            this.materialName = materialName;
            this.startIndex = startIndex;
            this.indexCount = indexCount;
        }
    }

    public static class RawGeometry {
        public float[] positions;
        public float[] normals;
        public float[] uvs;
        public int[] indices;
        public int vertexCount;
        public int indexCount;
        public List<SubMesh> subMeshes;
        /** DAE object origin after the scene-node transform, used as a rigid-prop pivot. */
        public float originX;
        public float originY;
        public float originZ;
        /** Unit object-local axes after the DAE scene-node transform. */
        public float axisXX = 1.0f, axisXY, axisXZ;
        public float axisYX, axisYY = 1.0f, axisYZ;
        public float axisZX, axisZY, axisZZ = 1.0f;

        // 引用计数器
        public int refCount = 0;
    }

    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    // 车型存活实例计数器 (例如: "pickup" -> 2 辆)
    private static final Map<String, Integer> VEHICLE_REF_COUNT = new HashMap<>();

    // ---- On-demand mesh loading ---------------------------------------------
    //
    // A vehicle needs a small slice of the shared library. The stock common.zip
    // holds 167 .dae files totalling 615 MB, while an etk800 touches a handful (one
    // wheel, one tire, the brakes). Handing all of them to Assimp costs ~34 s, so
    // only the shared-library files that back a requested mesh are imported, and the
    // provider of a mesh is found by scanning candidate DAE text for node names.
    //
    // JBeam cannot say which file a mesh lives in (flexbody and prop tables carry
    // the mesh name) and the file names are not a usable index either —
    // tire_super_modern_sport lives in tires.dae, disc_brake in disc_brakes.dae.
    //
    // Every visible mesh is registered in flex.meshName[], so requireMeshes()
    // primes exactly the set that can ever be requested; resolveMesh() also
    // resolves lazily on a miss, so nothing outside that set can silently vanish.

    /** Cached candidate list and scan state for one namespace. */
    private static final class NamespaceState {
        List<ResolvedEntry> candidates;
        /** entryName of every candidate already handed to Assimp. */
        final Set<String> imported = new HashSet<>();
        /**
         * Node names seen in a candidate's text; kept across releases (files do not
         * change). Written from the parallel harvest, so it must be concurrent.
         */
        final Map<String, Set<String>> namesByEntry = new ConcurrentHashMap<>();
    }

    /**
     * How many candidates are read and indexed before names are attributed. Each
     * slice is scanned in parallel; the slice boundary is what keeps a vehicle from
     * indexing a library it never touches.
     */
    private static final int HARVEST_SLICE = 32;

    private static final Map<String, NamespaceState> NAMESPACE_STATE = new HashMap<>();
    /** Mesh names of the current require call that no candidate declares. */
    private static final Set<String> unattributed = new LinkedHashSet<>();
    /** Cap on how many unattributed names the report prints, to keep it readable. */
    private static final int MAX_REPORTED_NAMES = 12;
    /** The ATTRIBUTE patterns a COLLADA node or geometry declares its name with. */
    private static final String[] NAME_ATTRIBUTES = {"name=\"", "id=\""};
    /** Roots of the most recent require call, for late resolveMesh misses. */
    private static List<File> activeRoots = List.of();

    public static String cleanIdentifier(String name) {
        if (name == null || name.isEmpty()) return "";
        int dotIdx = name.lastIndexOf('.');
        if (dotIdx > 0 && name.length() - dotIdx <= 5) {
            boolean isNumeric = true;
            for (int i = dotIdx + 1; i < name.length(); i++) {
                if (!Character.isDigit(name.charAt(i))) { isNumeric = false; break; }
            }
            if (isNumeric) name = name.substring(0, dotIdx);
        }
        if (name.endsWith("-mesh")) name = name.substring(0, name.length() - 5);
        if (name.endsWith("_mesh")) name = name.substring(0, name.length() - 5);
        return name;
    }

    /**
     * Loads the meshes behind {@code meshNames} for {@code namespace}. The vehicle's
     * own namespace is imported whole; each name is then resolved against the shared
     * library on demand, importing only the {@code .dae} files that declare it.
     *
     * <p>Names that no file declares are fine: they cost only the provider search,
     * and such a mesh never rendered anyway. They are reported rather than chased,
     * because importing the whole shared library to find out costs ~40 s per miss.
     */
    public static void requireMeshes(List<File> assetRoots, String namespace, Collection<String> meshNames) {
        activeRoots = List.copyOf(assetRoots);
        VEHICLE_REF_COUNT.merge(namespace, 1, Integer::sum);

        long started = LoadTiming.start();

        // The vehicle's own namespace is imported whole. It is small (two .dae for an
        // etk800) and nearly all of it is needed, so a provider search would cost more
        // than it saves — and a name the index cannot see would silently drop the body
        // mesh. The waste is all in the shared library, so only that side is on demand.
        if (!COMMON.equals(namespace)) {
            importAllRemaining(namespace);
        }

        // Every wanted name is attributed in one batched pass: it stops as soon as the
        // last name has a provider, instead of walking the candidate list per name.
        Set<String> pending = new LinkedHashSet<>();
        for (String meshName : meshNames) {
            if (meshName != null && !meshName.isEmpty() && !isCached(namespace, meshName)) {
                pending.add(meshName);
            }
        }
        attributeAll(COMMON, pending);

        unattributed.clear();
        unattributed.addAll(pending);
        LoadTiming.log("  [dae] " + namespace + ": " + importedSummary(namespace), started);
        reportUnattributed();
    }

    /**
     * Names no {@code .dae} declares. Not necessarily a problem: a visible-mesh table
     * keeps rows for parts whose mesh is simply absent, and those meshes never
     * rendered. It is reported because the other explanation is that the name index
     * reads a name differently than Assimp does, which would drop a real mesh.
     */
    private static void reportUnattributed() {
        if (unattributed.isEmpty()) {
            return;
        }
        StringBuilder shown = new StringBuilder();
        int printed = 0;
        for (String name : unattributed) {
            if (printed == MAX_REPORTED_NAMES) {
                shown.append(", …");
                break;
            }
            if (printed > 0) {
                shown.append(", ");
            }
            shown.append(name);
            printed++;
        }
        System.err.println("  [dae] " + unattributed.size() + " mesh name(s) declared no provider, "
                + "so they will not render: " + shown);
    }

    /** e.g. {@code etk800 imported 1/2, common imported 4/167}. */
    private static String importedSummary(String namespace) {
        StringBuilder summary = new StringBuilder();
        for (String ns : COMMON.equals(namespace) ? List.of(COMMON) : List.of(namespace, COMMON)) {
            NamespaceState state = NAMESPACE_STATE.get(ns);
            if (state == null) {
                continue;
            }
            if (summary.length() > 0) {
                summary.append(", ");
            }
            summary.append(ns).append(" imported ").append(state.imported.size())
                    .append('/').append(state.candidates.size());
        }
        return summary.length() == 0 ? "no candidates scanned" : summary.toString();
    }

    /**
     * Maps the shared-library files that declare any of {@code pending}, removing a
     * name from the set once it resolves. The vehicle's own namespace is already
     * imported by the caller, so only the shared library is searched.
     *
     * <p>Candidates are indexed a slice at a time and attributed in path order, so a
     * name provided by several files still goes to the same winner it did when the
     * search ran one name at a time. Inside a slice the scan runs in parallel: the
     * needed files sit across the whole 615 MB of the shared library, so every
     * candidate gets read either way, and the text scan is what costs — reading all
     * 178 of them is 0.5 s against 1.2 s of scanning.
     */
    private static void attributeAll(String namespace, Set<String> pending) {
        NamespaceState state = stateFor(namespace);
        for (int from = 0; from < state.candidates.size() && !pending.isEmpty(); from += HARVEST_SLICE) {
            int to = Math.min(from + HARVEST_SLICE, state.candidates.size());
            List<ResolvedEntry> slice = state.candidates.subList(from, to);

            slice.parallelStream()
                    .filter(entry -> !state.imported.contains(entry.entryName()))
                    .forEach(entry -> harvest(state, entry));

            for (ResolvedEntry entry : slice) {
                if (pending.isEmpty()) {
                    break;
                }
                if (state.imported.contains(entry.entryName())) {
                    continue;
                }
                Set<String> names = state.namesByEntry.get(entry.entryName());
                if (names == null || Collections.disjoint(names, pending)) {
                    continue;
                }
                importEntry(entry, namespace, state);
                // An import only counts if the geometry really landed: a file Assimp
                // rejects must not retire a name it "declared".
                pending.removeIf(name -> MESH_CACHE.containsKey(namespace + ":" + name));
            }
        }
    }

    /**
     * Reads a candidate's text into the name index, once. Each entry appears in one
     * slice, so no two threads ever harvest the same file.
     */
    private static void harvest(NamespaceState state, ResolvedEntry entry) {
        state.namesByEntry.put(entry.entryName(), harvestNames(entry));
    }

    /** Resolves one mesh name eagerly, for a caller that is not {@link #requireMeshes}. */
    private static void ensureMesh(String namespace, String meshName) {
        if (isCached(namespace, meshName)) {
            return;
        }
        Set<String> pending = new LinkedHashSet<>();
        pending.add(meshName);
        attributeAll(COMMON, pending);
        if (!pending.isEmpty()) {
            unattributed.add(meshName);
        }
    }

    private static boolean isCached(String namespace, String meshName) {
        return MESH_CACHE.containsKey(namespace + ":" + meshName)
                || MESH_CACHE.containsKey(COMMON + ":" + meshName);
    }

    /**
     * Collects the node and geometry names a {@code .dae} declares, without going
     * through Assimp. Deliberately over-inclusive: both {@code name=} and {@code id=}
     * are taken, so a name Assimp sanitises differently costs at most one wasted
     * import before the next candidate is tried.
     */
    private static Set<String> harvestNames(ResolvedEntry entry) {
        try {
            return extractMeshNames(entry.readBytes());
        } catch (Exception e) {
            System.err.println("Failed to index mesh names in " + entry.sourceAddress());
            return Set.of();
        }
    }

    /**
     * The names a COLLADA document declares on its nodes and geometries. Extracted
     * without Assimp because the whole point is to avoid importing a file just to
     * learn whether it holds a name. Package-private so the extraction is testable
     * without a real mesh or the native importer.
     *
     * <p>Casing is preserved: cache keys are Assimp node names, and lookups are
     * case-sensitive.
     */
    static Set<String> extractMeshNames(byte[] bytes) {
        Set<String> names = new HashSet<>();
        for (String attribute : NAME_ATTRIBUTES) {
            int from = 0;
            while (true) {
                int at = indexOf(bytes, attribute, from);
                if (at < 0) {
                    break;
                }
                int start = at + attribute.length();
                int end = start;
                while (end < bytes.length && bytes[end] != '"') {
                    end++;
                }
                if (end < bytes.length) {
                    String cleaned = cleanIdentifier(
                            new String(bytes, start, end - start, StandardCharsets.US_ASCII));
                    if (!cleaned.isEmpty()) {
                        names.add(cleaned);
                    }
                }
                from = end + 1;
            }
        }
        return names;
    }

    /** Plain byte search; the needles are ASCII, so no decoding is involved. */
    private static int indexOf(byte[] haystack, String needle, int from) {
        int last = haystack.length - needle.length();
        byte first = (byte) needle.charAt(0);
        outer:
        for (int i = Math.max(0, from); i <= last; i++) {
            if (haystack[i] != first) {
                continue;
            }
            for (int j = 1; j < needle.length(); j++) {
                if (haystack[i + j] != (byte) needle.charAt(j)) {
                    continue outer;
                }
            }
            return i;
        }
        return -1;
    }

    /** Imports every candidate of a namespace that has not been imported yet. */
    private static void importAllRemaining(String namespace) {
        NamespaceState state = stateFor(namespace);
        for (ResolvedEntry entry : state.candidates) {
            if (!state.imported.contains(entry.entryName())) {
                importEntry(entry, namespace, state);
            }
        }
    }

    /** Imports one candidate; marked first so a failure is not retried forever. */
    private static void importEntry(ResolvedEntry entry, String namespace, NamespaceState state) {
        state.imported.add(entry.entryName());
        long fileStart = LoadTiming.start();
        try {
            Path path = entry.materializeForAssimp();
            try {
                loadMeshUsingAssimp(path.toString(), namespace);
            } finally {
                entry.deleteTemp();
            }
        } catch (Exception e) {
            System.err.println("Failed to load DAE asset: " + entry.sourceAddress());
        }
        LoadTiming.log("  [dae] " + entry.logicalPath() + " total", fileStart);
    }

    private static NamespaceState stateFor(String namespace) {
        NamespaceState state = NAMESPACE_STATE.get(namespace);
        if (state == null) {
            state = new NamespaceState();
            state.candidates = candidateMeshes(namespace);
            NAMESPACE_STATE.put(namespace, state);
        }
        return state;
    }

    private static List<ResolvedEntry> candidateMeshes(String namespace) {
        List<ResolvedEntry> candidates = new ArrayList<>();
        for (ResolvedEntry entry : AssetScanner.INSTANCE.scan(activeRoots, namespace).entries()) {
            // logicalPath() is the lowercased dedupe key, so this is case-insensitive.
            if (entry.logicalPath().endsWith(".dae")) {
                candidates.add(entry);
            }
        }
        return candidates;
    }

    /**
     * 当一辆车被从世界移除时调用（触发垃圾回收）
     */
    public static void releaseVehicleModels(String targetVehicleName) {
        int count = VEHICLE_REF_COUNT.getOrDefault(targetVehicleName, 0) - 1;
        if (count <= 0) {
            VEHICLE_REF_COUNT.remove(targetVehicleName);
            System.out.println("Releasing DAE assets for: " + targetVehicleName);

            // 从缓存中安全移除属于该车系的所有网格数据，释放堆内存
            String prefix = targetVehicleName + ":";
            MESH_CACHE.entrySet().removeIf(entry -> entry.getKey().startsWith(prefix));

            // The geometries are gone, so a later spawn of this namespace has to
            // import its providers again. The name index stays valid — files do not
            // change — so only the import bookkeeping is dropped.
            NamespaceState state = NAMESPACE_STATE.get(targetVehicleName);
            if (state != null) {
                state.imported.clear();
            }
        } else {
            VEHICLE_REF_COUNT.put(targetVehicleName, count);
        }
    }

    /**
     * Shared scoped lookup: the namespace's own mesh first, then the common library.
     * A miss is resolved on demand so a caller that did not go through
     * {@link #requireMeshes} cannot silently lose a mesh that does exist.
     */
    public static RawGeometry resolveMesh(String namespace, String meshName) {
        RawGeometry geometry = MESH_CACHE.get(namespace + ":" + meshName);
        if (geometry != null) {
            return geometry;
        }
        geometry = MESH_CACHE.get(COMMON + ":" + meshName);
        if (geometry != null || activeRoots.isEmpty()) {
            return geometry;
        }
        // Not primed, or genuinely absent. Search the shared library's providers so a
        // lookup that did not go through requireMeshes still finds its mesh.
        ensureMesh(namespace, meshName);
        geometry = MESH_CACHE.get(namespace + ":" + meshName);
        return geometry != null ? geometry : MESH_CACHE.get(COMMON + ":" + meshName);
    }

    private static void loadMeshUsingAssimp(String filePath, String namespace) {
        int postProcessingFlags =
                Assimp.aiProcess_Triangulate |              // 切分为三角形
                        Assimp.aiProcess_GenSmoothNormals |         // 生成平滑着色法线
                        Assimp.aiProcess_JoinIdenticalVertices |    // 优化合并
                        Assimp.aiProcess_ImproveCacheLocality;

        // Disable Assimp's automatic Z-up to Y-up conversion so the imported
        // vertices stay in BeamNG's native frame, matching the JBeam slot transforms.
        AIPropertyStore store = Assimp.aiCreatePropertyStore();
        if (store != null) {
            Assimp.aiSetImportPropertyInteger(store, Assimp.AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
        }

        // Split the native import from our own scene walk: the two have very
        // different fixes (Assimp post-processing flags vs the bake loop).
        long importStart = LoadTiming.start();
        AIScene scene;
        if (store != null) {
            // 携带属性强制加载
            scene = Assimp.aiImportFileExWithProperties(filePath, postProcessingFlags, null, store);
            Assimp.aiReleasePropertyStore(store);
        } else {
            // Fallback (通常不会走到这里)
            scene = Assimp.aiImportFile(filePath, postProcessingFlags);
        }
        LoadTiming.log("    assimp import", importStart);

        if (scene == null || scene.mRootNode() == null) return;

        // 继续使用上一版的矩阵级联传递，此时的根矩阵是纯净的 Identity
        long bakeStart = LoadTiming.start();
        processSceneNodesRecursively(scene.mRootNode(), scene, namespace, new Matrix4f().identity());
        LoadTiming.log("    scene bake", bakeStart);
        Assimp.aiReleaseImport(scene);
    }

    private static void processSceneNodesRecursively(AINode node, AIScene scene, String namespace, Matrix4f parentTransform) {
        if (node == null) return;

        // 1. 提取当前 Collada 节点的局部变换矩阵 (Assimp 矩阵为行主序)
        AIMatrix4x4 m = node.mTransformation();
        // 映射到 JOML Matrix4f 的列主序构造函数中 (Col 0, Col 1, Col 2, Col 3)
        Matrix4f localTransform = new Matrix4f(
                m.a1(), m.b1(), m.c1(), m.d1(),
                m.a2(), m.b2(), m.c2(), m.d2(),
                m.a3(), m.b3(), m.c3(), m.d3(),
                m.a4(), m.b4(), m.c4(), m.d4()
        );

        // 2. 累乘计算出当前节点的全局绝对变换矩阵：Global = Parent * Local
        Matrix4f globalTransform = new Matrix4f(parentTransform).mul(localTransform);
        Matrix3f normalTransform = new Matrix3f(globalTransform);
        if (Math.abs(normalTransform.determinant()) > 1.0e-8f) {
            normalTransform.invert().transpose();
        } else {
            // Keep malformed singular DAE transforms from injecting NaNs into
            // every subsequent GPU skinning update.
            normalTransform.identity();
        }

        String rawNodeName = node.mName().dataString();
        String cleanNodeName = cleanIdentifier(rawNodeName);

        int numMeshes = node.mNumMeshes();
        IntBuffer meshIndices = node.mMeshes();

        // 3. 如果当前节点挂载了实体网格，则对其进行烘焙提取
        if (numMeshes > 0 && meshIndices != null && !cleanNodeName.isEmpty()) {
            PointerBuffer sceneMeshesBuffer = scene.mMeshes();
            List<AIMesh> attachedMeshSlices = new ArrayList<>(numMeshes);
            int totalTriangleFaces = 0;

            for (int i = 0; i < numMeshes; i++) {
                int meshIdx = meshIndices.get(i);
                AIMesh aiMesh = AIMesh.create(sceneMeshesBuffer.get(meshIdx));
                attachedMeshSlices.add(aiMesh);

                int faceCount = aiMesh.mNumFaces();
                AIFace.Buffer facesBuffer = aiMesh.mFaces();
                for (int f = 0; f < faceCount; f++) {
                    if (facesBuffer.get(f).mNumIndices() == 3) {
                        totalTriangleFaces++;
                    }
                }
            }

            if (totalTriangleFaces > 0) {
                int totalRenderVertices = 0;
                for (AIMesh mesh : attachedMeshSlices) {
                    totalRenderVertices += mesh.mNumVertices();
                }
                int totalRenderIndices = totalTriangleFaces * 3;
                float[] mergedPositions = new float[totalRenderVertices * 3];
                float[] mergedNormals   = new float[totalRenderVertices * 3];
                float[] mergedUvs       = new float[totalRenderVertices * 2];
                int[] mergedIndices     = new int[totalRenderIndices];
                List<SubMesh> subMeshes = new ArrayList<>();

                int currentMergedVertPtr = 0;
                int currentMergedIndexPtr = 0;
                // Reuse temporary vectors instead of allocating per vertex.
                Vector3f tempPos = new Vector3f();
                Vector3f tempNorm = new Vector3f();

                for (AIMesh aiMesh : attachedMeshSlices) {
                    int subMeshStartIndex = currentMergedIndexPtr;

                    String materialNameStr = "default";
                    int matIdx = aiMesh.mMaterialIndex();
                    if (scene.mMaterials() != null && matIdx < scene.mNumMaterials()) {
                        AIMaterial aiMaterial = AIMaterial.create(scene.mMaterials().get(matIdx));
                        AIString matName = AIString.calloc();
                        if (Assimp.aiGetMaterialString(aiMaterial, Assimp.AI_MATKEY_NAME, 0, 0, matName) == Assimp.aiReturn_SUCCESS) {
                            materialNameStr = matName.dataString();
                        }
                        matName.free();
                    }

                    AIFace.Buffer facesBuffer = aiMesh.mFaces();
                    AIVector3D.Buffer posBuffer = aiMesh.mVertices();
                    AIVector3D.Buffer normBuffer = aiMesh.mNormals();
                    AIVector3D.Buffer uvBuffer = aiMesh.mTextureCoords(0);
                    int faceCount = aiMesh.mNumFaces();
                    boolean[] usedVertices = new boolean[aiMesh.mNumVertices()];
                    for (int faceIndex = 0; faceIndex < faceCount; faceIndex++) {
                        AIFace face = facesBuffer.get(faceIndex);
                        if (face.mNumIndices() != 3) {
                            continue;
                        }
                        IntBuffer faceIndices = face.mIndices();
                        usedVertices[faceIndices.get(0)] = true;
                        usedVertices[faceIndices.get(1)] = true;
                        usedVertices[faceIndices.get(2)] = true;
                    }

                    int[] localToMergedVertex = new int[aiMesh.mNumVertices()];
                    Arrays.fill(localToMergedVertex, -1);
                    for (int vertex = 0; vertex < aiMesh.mNumVertices(); vertex++) {
                        if (!usedVertices[vertex]) {
                            continue;
                        }
                        localToMergedVertex[vertex] = currentMergedVertPtr;
                        AIVector3D pos = posBuffer.get(vertex);
                        tempPos.set(pos.x(), pos.y(), pos.z());
                        globalTransform.transformPosition(tempPos);

                        mergedPositions[currentMergedVertPtr * 3] = tempPos.x;
                        mergedPositions[currentMergedVertPtr * 3 + 1] = tempPos.y;
                        mergedPositions[currentMergedVertPtr * 3 + 2] = tempPos.z;

                        if (normBuffer != null) {
                            AIVector3D norm = normBuffer.get(vertex);
                            tempNorm.set(norm.x(), norm.y(), norm.z());
                            normalTransform.transform(tempNorm);
                            if (tempNorm.lengthSquared() > 1.0e-12f) {
                                tempNorm.normalize();
                            } else {
                                tempNorm.set(0.0f, 0.0f, 1.0f);
                            }
                            mergedNormals[currentMergedVertPtr * 3] = tempNorm.x;
                            mergedNormals[currentMergedVertPtr * 3 + 1] = tempNorm.y;
                            mergedNormals[currentMergedVertPtr * 3 + 2] = tempNorm.z;
                        } else {
                            mergedNormals[currentMergedVertPtr * 3] = 0.0f;
                            mergedNormals[currentMergedVertPtr * 3 + 1] = 0.0f;
                            mergedNormals[currentMergedVertPtr * 3 + 2] = 1.0f;
                        }

                        if (uvBuffer != null) {
                            AIVector3D uv = uvBuffer.get(vertex);
                            mergedUvs[currentMergedVertPtr * 2] = uv.x();
                            mergedUvs[currentMergedVertPtr * 2 + 1] = 1.0f - uv.y();
                        }

                        currentMergedVertPtr++;
                    }

                    for (int faceIndex = 0; faceIndex < faceCount; faceIndex++) {
                        AIFace face = facesBuffer.get(faceIndex);
                        if (face.mNumIndices() != 3) {
                            continue;
                        }
                        IntBuffer faceIndices = face.mIndices();
                        mergedIndices[currentMergedIndexPtr++] = localToMergedVertex[faceIndices.get(0)];
                        mergedIndices[currentMergedIndexPtr++] = localToMergedVertex[faceIndices.get(1)];
                        mergedIndices[currentMergedIndexPtr++] = localToMergedVertex[faceIndices.get(2)];
                    }

                    int subMeshIndexCount = currentMergedIndexPtr - subMeshStartIndex;
                    if (subMeshIndexCount > 0) {
                        subMeshes.add(new SubMesh(materialNameStr, subMeshStartIndex, subMeshIndexCount));
                    }
                }

                RawGeometry unifiedGeometry = new RawGeometry();
                unifiedGeometry.positions   = mergedPositions;
                unifiedGeometry.normals     = mergedNormals;
                unifiedGeometry.uvs         = mergedUvs;
                unifiedGeometry.indices     = mergedIndices;
                unifiedGeometry.vertexCount = currentMergedVertPtr;
                unifiedGeometry.indexCount  = currentMergedIndexPtr;
                unifiedGeometry.subMeshes   = subMeshes;
                Vector3f meshOrigin = new Vector3f();
                globalTransform.transformPosition(meshOrigin);
                unifiedGeometry.originX = meshOrigin.x;
                unifiedGeometry.originY = meshOrigin.y;
                unifiedGeometry.originZ = meshOrigin.z;
                Vector3f meshAxisX = new Vector3f(1.0f, 0.0f, 0.0f);
                Vector3f meshAxisY = new Vector3f(0.0f, 1.0f, 0.0f);
                Vector3f meshAxisZ = new Vector3f(0.0f, 0.0f, 1.0f);
                globalTransform.transformDirection(meshAxisX);
                globalTransform.transformDirection(meshAxisY);
                globalTransform.transformDirection(meshAxisZ);
                orthonormalizeObjectAxes(meshAxisX, meshAxisY, meshAxisZ);
                unifiedGeometry.axisXX = meshAxisX.x;
                unifiedGeometry.axisXY = meshAxisX.y;
                unifiedGeometry.axisXZ = meshAxisX.z;
                unifiedGeometry.axisYX = meshAxisY.x;
                unifiedGeometry.axisYY = meshAxisY.y;
                unifiedGeometry.axisYZ = meshAxisY.z;
                unifiedGeometry.axisZX = meshAxisZ.x;
                unifiedGeometry.axisZY = meshAxisZ.y;
                unifiedGeometry.axisZZ = meshAxisZ.z;

                // Register under both the cleaned and the original node name so either resolves.
                MESH_CACHE.put(namespace + ":" + cleanNodeName, unifiedGeometry);
                if (!cleanNodeName.equals(rawNodeName)) {
                    MESH_CACHE.put(namespace + ":" + rawNodeName, unifiedGeometry);
                }
            }
        }

        // 4. 携带当前计算完毕的绝对矩阵，继续向下层级递归传递
        int numChildren = node.mNumChildren();
        PointerBuffer childrenBuffer = node.mChildren();
        if (numChildren > 0 && childrenBuffer != null) {
            for (int i = 0; i < numChildren; i++) {
                processSceneNodesRecursively(AINode.create(childrenBuffer.get(i)), scene, namespace, globalTransform);
            }
        }
    }

    private static void orthonormalizeObjectAxes(Vector3f x, Vector3f y, Vector3f authoredZ) {
        if (x.lengthSquared() <= 1.0e-12f || y.lengthSquared() <= 1.0e-12f) {
            x.set(1.0f, 0.0f, 0.0f);
            y.set(0.0f, 1.0f, 0.0f);
            authoredZ.set(0.0f, 0.0f, 1.0f);
            return;
        }
        x.normalize();
        y.fma(-x.dot(y), x);
        if (y.lengthSquared() <= 1.0e-12f) {
            y.set(0.0f, 1.0f, 0.0f);
            if (Math.abs(x.dot(y)) > 0.9f) y.set(0.0f, 0.0f, 1.0f);
            y.fma(-x.dot(y), x);
        }
        y.normalize();
        Vector3f cross = new Vector3f(x).cross(y).normalize();
        if (authoredZ.lengthSquared() > 1.0e-12f && cross.dot(authoredZ) < 0.0f) {
            y.negate();
            cross.negate();
        }
        authoredZ.set(cross);
    }

}
