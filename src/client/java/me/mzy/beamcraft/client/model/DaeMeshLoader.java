package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.NamespaceScan;
import me.mzy.beamcraft.client.assets.ResolvedEntry;

import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.assimp.*;
import org.lwjgl.PointerBuffer;

import java.io.File;
import java.nio.IntBuffer;
import java.nio.file.Path;
import java.util.*;

public class DaeMeshLoader {

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

        // 引用计数器
        public int refCount = 0;
    }

    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    // 车型存活实例计数器 (例如: "pickup" -> 2 辆)
    private static final Map<String, Integer> VEHICLE_REF_COUNT = new HashMap<>();
    // 标记 common 库是否已挂载
    private static boolean isCommonLoaded = false;

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
     * 当一辆车准备生成时调用（按需加载该车系及 Common 的所有模型）
     */
    public static void requireVehicleModels(File vehiclesRootDir, String targetVehicleName) {
        requireVehicleModels(List.of(vehiclesRootDir), targetVehicleName);
    }

    /**
     * Multi-root variant: scans every configured asset root for the vehicle's
     * {@code .dae} meshes (and the shared common library), via
     * {@link AssetScanner}. Zip-sourced meshes are materialised to a temporary
     * file for Assimp and deleted afterwards.
     */
    public static void requireVehicleModels(List<File> assetRoots, String targetVehicleName) {
        int count = VEHICLE_REF_COUNT.getOrDefault(targetVehicleName, 0);
        VEHICLE_REF_COUNT.put(targetVehicleName, count + 1);

        if (count == 0) {
            System.out.println("====== 🚀 按需加载 DAE 资产: " + targetVehicleName + " ======");

            // 1. 确保基础 common 资产已加载
            if (!isCommonLoaded) {
                loadSpecificVehicleDae(assetRoots, "common");
                isCommonLoaded = true;
            }

            // 2. 加载目标车系的所有 DAE 资产
            if (!targetVehicleName.equals("common")) {
                loadSpecificVehicleDae(assetRoots, targetVehicleName);
            }
        }
    }

    /**
     * 当一辆车被从世界移除时调用（触发垃圾回收）
     */
    public static void releaseVehicleModels(String targetVehicleName) {
        int count = VEHICLE_REF_COUNT.getOrDefault(targetVehicleName, 0) - 1;
        if (count <= 0) {
            VEHICLE_REF_COUNT.remove(targetVehicleName);
            System.out.println("====== 🗑️ 回收 DAE 资产: " + targetVehicleName + " ======");

            // 从缓存中安全移除属于该车系的所有网格数据，释放堆内存
            String prefix = targetVehicleName + ":";
            MESH_CACHE.entrySet().removeIf(entry -> entry.getKey().startsWith(prefix));

        } else {
            VEHICLE_REF_COUNT.put(targetVehicleName, count);
        }
    }

    /** Shared scoped lookup: the namespace's own mesh first, then the common library. */
    public static RawGeometry resolveMesh(String namespace, String meshName) {
        RawGeometry geometry = MESH_CACHE.get(namespace + ":" + meshName);
        return geometry != null ? geometry : MESH_CACHE.get("common:" + meshName);
    }

    private static void loadSpecificVehicleDae(List<File> assetRoots, String namespace) {
        NamespaceScan scan = AssetScanner.INSTANCE.scan(assetRoots, namespace);
        for (ResolvedEntry entry : scan.entries()) {
            if (!entry.logicalPath().endsWith(".dae")) {
                continue;
            }
            try {
                Path path = entry.materializeForAssimp();
                try {
                    loadMeshUsingAssimp(path.toString(), namespace);
                } finally {
                    entry.deleteTemp();
                }
            } catch (Exception e) {
                System.err.println("🚨 加载 DAE 资产失败: " + entry.sourceAddress());
            }
        }
    }

    private static void loadMeshUsingAssimp(String filePath, String namespace) {
        int postProcessingFlags =
                Assimp.aiProcess_Triangulate |              // 切分为三角形
                        Assimp.aiProcess_GenSmoothNormals |         // 生成平滑着色法线
                        Assimp.aiProcess_JoinIdenticalVertices |    // 优化合并
                        Assimp.aiProcess_ImproveCacheLocality;

        // 创建属性存储器，强制禁止 Assimp 自动将 Z-up 转换为 Y-up！
        // 这样读取进来的顶点就是纯正的 BeamNG 原始数据，完美对接 JBeam 插槽旋转。
        AIPropertyStore store = Assimp.aiCreatePropertyStore();
        if (store != null) {
            Assimp.aiSetImportPropertyInteger(store, Assimp.AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
        }

        AIScene scene;
        if (store != null) {
            // 携带属性强制加载
            scene = Assimp.aiImportFileExWithProperties(filePath, postProcessingFlags, null, store);
            Assimp.aiReleasePropertyStore(store);
        } else {
            // Fallback (通常不会走到这里)
            scene = Assimp.aiImportFile(filePath, postProcessingFlags);
        }

        if (scene == null || scene.mRootNode() == null) return;

        // 继续使用上一版的矩阵级联传递，此时的根矩阵是纯净的 Identity
        processSceneNodesRecursively(scene.mRootNode(), scene, namespace, new Matrix4f().identity());
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
                // 复用临时向量对象，避免高频创建销毁产生内存垃圾
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

                // 完美映射：基于原生节点名与切片名进行双重全域覆盖
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
}
