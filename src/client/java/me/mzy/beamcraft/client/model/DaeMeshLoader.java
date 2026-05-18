package me.mzy.beamcraft.client.model;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.assimp.*;
import org.lwjgl.PointerBuffer;

import java.io.*;
import java.nio.IntBuffer;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.*;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

public class DaeMeshLoader {

    public static class SubMesh {
        public String materialName;
        public int startVertex;
        public int vertexCount;

        public SubMesh(String materialName, int startVertex, int vertexCount) {
            this.materialName = materialName;
            this.startVertex = startVertex;
            this.vertexCount = vertexCount;
        }
    }

    public static class RawGeometry {
        public float[] positions;       // 烘焙绝对变换后的 BeamNG Z-up 原生坐标流
        public float[] normals;         // 修正姿态后的原生平滑着色法线流
        public float[] uvs;             // 修正 V 轴后的贴图 UV 流
        public int vertexCount;         // 填充后的渲染顶点总数 (严格为 4 的倍数)
        public List<SubMesh> subMeshes;
    }

    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    /**
     * 🚀 强健的标识符清洗器：全方位剥离 DCC 软件导出的各类几何体尾缀
     */
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

    public static void scanAndLoadAllVehicles(File vehiclesRootDir) {
        if (!vehiclesRootDir.exists()) {
            vehiclesRootDir.mkdirs();
            return;
        }

        System.out.println("====== 🚀 启动 Assimp 手动矩阵级联提取与拓扑深度清洗 ======");
        MESH_CACHE.clear();

        File[] files = vehiclesRootDir.listFiles();
        if (files == null) return;

        for (File file : files) {
            if (file.isDirectory()) {
                scanFolderForDae(file, vehiclesRootDir);
            } else if (file.getName().toLowerCase().endsWith(".zip")) {
                scanZipForDae(file);
            }
        }
        System.out.println("📦 载具底模全域挂载完毕！高保真聚合网格数: " + MESH_CACHE.size());
    }

    private static void scanZipForDae(File zipFile) {
        try (ZipFile zf = new ZipFile(zipFile)) {
            Enumeration<? extends ZipEntry> entries = zf.entries();
            while (entries.hasMoreElements()) {
                ZipEntry entry = entries.nextElement();
                String entryName = entry.getName();
                if (!entry.isDirectory() && !entryName.contains("__MACOSX") && entryName.toLowerCase().endsWith(".dae")) {
                    String namespace = extractTrueVehicleName(entryName);
                    if (namespace == null) continue;

                    File tempFile = File.createTempFile("assimp_assets_", ".dae");
                    try {
                        try (InputStream is = zf.getInputStream(entry)) {
                            Files.copy(is, tempFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
                        }
                        loadMeshUsingAssimp(tempFile.getAbsolutePath(), namespace);
                    } finally {
                        tempFile.delete();
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("🚨 穿透读取 ZIP 资产失败: " + zipFile.getName());
        }
    }

    private static void scanFolderForDae(File folder, File rootDir) {
        List<File> daeFiles = new ArrayList<>();
        collectFilesEndingWith(folder, ".dae", daeFiles);
        for (File daeFile : daeFiles) {
            String relativePath = rootDir.toURI().relativize(daeFile.toURI()).getPath();
            String namespace = extractTrueVehicleName(relativePath);
            if (namespace == null) namespace = folder.getName();
            loadMeshUsingAssimp(daeFile.getAbsolutePath(), namespace);
        }
    }

    private static void collectFilesEndingWith(File dir, String suffix, List<File> list) {
        File[] files = dir.listFiles();
        if (files == null) return;
        for (File f : files) {
            if (f.isDirectory()) collectFilesEndingWith(f, suffix, list);
            else if (f.getName().toLowerCase().endsWith(suffix)) list.add(f);
        }
    }

    private static String extractTrueVehicleName(String internalPath) {
        String normalizedPath = internalPath.replace("\\", "/");
        String[] parts = normalizedPath.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            if (parts[i].equalsIgnoreCase("vehicles")) return parts[i + 1];
        }
        return null;
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
                int totalRenderVertices = totalTriangleFaces * 3;
                float[] mergedPositions = new float[totalRenderVertices * 3];
                float[] mergedNormals   = new float[totalRenderVertices * 3];
                float[] mergedUvs       = new float[totalRenderVertices * 2];
                List<SubMesh> subMeshes = new ArrayList<>();

                int currentMergedVertPtr = 0;
                // 复用临时向量对象，避免高频创建销毁产生内存垃圾
                Vector3f tempPos = new Vector3f();
                Vector3f tempNorm = new Vector3f();

                for (AIMesh aiMesh : attachedMeshSlices) {
                    int subMeshStartVertex = currentMergedVertPtr;
                    int subMeshAddedVertices = 0;

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
                    for (int f = 0; f < faceCount; f++) {
                        AIFace face = facesBuffer.get(f);
                        if (face.mNumIndices() != 3) continue;

                        IntBuffer indices = face.mIndices();
                        int[] targetIndices = new int[] { indices.get(0), indices.get(1), indices.get(2) };

                        for (int vIdx : targetIndices) {
                            // 读取原生顶点并注入绝对变换矩阵
                            AIVector3D pos = posBuffer.get(vIdx);
                            tempPos.set(pos.x(), pos.y(), pos.z());
                            globalTransform.transformPosition(tempPos);

                            mergedPositions[currentMergedVertPtr * 3]     = tempPos.x;
                            mergedPositions[currentMergedVertPtr * 3 + 1] = tempPos.y;
                            mergedPositions[currentMergedVertPtr * 3 + 2] = tempPos.z;

                            // 同步变换法线向量方向 (仅受旋转影响)
                            if (normBuffer != null) {
                                AIVector3D norm = normBuffer.get(vIdx);
                                tempNorm.set(norm.x(), norm.y(), norm.z());
                                globalTransform.transformDirection(tempNorm);
                                tempNorm.normalize(); // 确保法线单位化

                                mergedNormals[currentMergedVertPtr * 3]     = tempNorm.x;
                                mergedNormals[currentMergedVertPtr * 3 + 1] = tempNorm.y;
                                mergedNormals[currentMergedVertPtr * 3 + 2] = tempNorm.z;
                            } else {
                                mergedNormals[currentMergedVertPtr * 3]     = 0f;
                                mergedNormals[currentMergedVertPtr * 3 + 1] = 0f;
                                mergedNormals[currentMergedVertPtr * 3 + 2] = 1f;
                            }

                            if (uvBuffer != null) {
                                AIVector3D uv = uvBuffer.get(vIdx);
                                mergedUvs[currentMergedVertPtr * 2]     = uv.x();
                                mergedUvs[currentMergedVertPtr * 2 + 1] = 1.0f - uv.y(); // 对齐 MC 纹理坐标
                            } else {
                                mergedUvs[currentMergedVertPtr * 2]     = 0f;
                                mergedUvs[currentMergedVertPtr * 2 + 1] = 0f;
                            }

                            currentMergedVertPtr++;
                            subMeshAddedVertices++;
                        }
                    }

                    if (subMeshAddedVertices > 0) {
                        subMeshes.add(new SubMesh(materialNameStr, subMeshStartVertex, subMeshAddedVertices));
                    }
                }

                RawGeometry unifiedGeometry = new RawGeometry();
                unifiedGeometry.positions   = mergedPositions;
                unifiedGeometry.normals     = mergedNormals;
                unifiedGeometry.uvs         = mergedUvs;
                unifiedGeometry.vertexCount = totalRenderVertices;
                unifiedGeometry.subMeshes   = subMeshes;

                // 完美映射：基于原生节点名与切片名进行双重全域覆盖
                MESH_CACHE.put(namespace + ":" + cleanNodeName, unifiedGeometry);
                // BeamCraft.LOGGER.info("cleanNodeName: " + namespace + ":" + cleanNodeName);
                if (!cleanNodeName.equals(rawNodeName)) {
                    MESH_CACHE.put(namespace + ":" + rawNodeName, unifiedGeometry);
                    // BeamCraft.LOGGER.info("rawNodeName: " + namespace + ":" + rawNodeName);
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