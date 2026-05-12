package me.mzy.beamcraft.client.model;

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
        public float[] positions;       // 紧凑且对齐 MC QUADS 规范的纯正 BeamNG Z-up 顶点流
        public float[] normals;         // 原生法线流
        public float[] uvs;             // 修正倒置后的 UV 流
        public int vertexCount;         // 填充后的渲染顶点总数 (严格为 4 的倍数)
        public List<SubMesh> subMeshes;
    }

    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    public static void scanAndLoadAllVehicles(File vehiclesRootDir) {
        if (!vehiclesRootDir.exists()) {
            vehiclesRootDir.mkdirs();
            return;
        }

        System.out.println("====== 🚀 启动 Assimp 高保真资产载入 (双轨字典映射与 QUADS 对齐) ======");
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
        System.out.println("📦 底模网格全数重组完毕！安全聚合模型总数: " + MESH_CACHE.size());
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

                    File tempFile = File.createTempFile("assimp_geom_", ".dae");
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
            System.err.println("🚨 穿透读取 ZIP 载具包失败: " + zipFile.getName());
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
                Assimp.aiProcess_Triangulate |              // 基础切分为实心三角形
                        Assimp.aiProcess_GenSmoothNormals |         // 自动补充平滑着色表面法线
                        Assimp.aiProcess_JoinIdenticalVertices |    // 合并重复顶点优化存储
                        Assimp.aiProcess_ImproveCacheLocality;

        AIScene scene = Assimp.aiImportFile(filePath, postProcessingFlags);
        if (scene == null || scene.mRootNode() == null) return;

        float[] identityMatrix = new float[] { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
        processSceneNodesRecursively(scene.mRootNode(), scene, namespace, identityMatrix);
        Assimp.aiReleaseImport(scene);
    }

    private static void processSceneNodesRecursively(AINode node, AIScene scene, String namespace, float[] parentMatrix) {
        if (node == null) return;

        AIMatrix4x4 localMat = node.mTransformation();
        float[] nodeMatrix = new float[] {
                localMat.a1(), localMat.a2(), localMat.a3(), localMat.a4(),
                localMat.b1(), localMat.b2(), localMat.b3(), localMat.b4(),
                localMat.c1(), localMat.c2(), localMat.c3(), localMat.c4(),
                localMat.d1(), localMat.d2(), localMat.d3(), localMat.d4()
        };
        float[] currentGlobalMatrix = multiplyMatrix4x4(parentMatrix, nodeMatrix);

        String rawNodeName = node.mName().dataString();
        String cleanNodeName = rawNodeName.endsWith("-mesh") ? rawNodeName.substring(0, rawNodeName.length() - 5) : rawNodeName;

        int numMeshes = node.mNumMeshes();
        IntBuffer meshIndices = node.mMeshes();

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
                // 🚀 QUADS 对齐填充：每个实心三角形填充展开为 4 个顶点
                int totalRenderVertices = totalTriangleFaces * 4;
                float[] mergedPositions = new float[totalRenderVertices * 3];
                float[] mergedNormals   = new float[totalRenderVertices * 3];
                float[] mergedUvs       = new float[totalRenderVertices * 2];
                List<SubMesh> subMeshes = new ArrayList<>();

                int currentMergedVertPtr = 0;

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
                        int idxA = indices.get(0);
                        int idxB = indices.get(1);
                        int idxC = indices.get(2);

                        // 补齐构建退化四边形 (A, B, C, C) 完美迎合 MC 渲染器
                        int[] targetIndices = new int[] { idxA, idxB, idxC, idxC };

                        for (int vIdx : targetIndices) {
                            // 保持缓存在原汁原味的 BeamNG Z-up 参考系中
                            AIVector3D pos = posBuffer.get(vIdx);
                            float rawX = pos.x(), rawY = pos.y(), rawZ = pos.z();
                            float tx = rawX * currentGlobalMatrix[0] + rawY * currentGlobalMatrix[1] + rawZ * currentGlobalMatrix[2] + currentGlobalMatrix[3];
                            float ty = rawX * currentGlobalMatrix[4] + rawY * currentGlobalMatrix[5] + rawZ * currentGlobalMatrix[6] + currentGlobalMatrix[7];
                            float tz = rawX * currentGlobalMatrix[8] + rawY * currentGlobalMatrix[9] + rawZ * currentGlobalMatrix[10] + currentGlobalMatrix[11];

                            mergedPositions[currentMergedVertPtr * 3]     = tx;
                            mergedPositions[currentMergedVertPtr * 3 + 1] = ty;
                            mergedPositions[currentMergedVertPtr * 3 + 2] = tz;

                            if (normBuffer != null) {
                                AIVector3D norm = normBuffer.get(vIdx);
                                float nx = norm.x(), ny = norm.y(), nz = norm.z();
                                float ntx = nx * currentGlobalMatrix[0] + ny * currentGlobalMatrix[1] + nz * currentGlobalMatrix[2];
                                float nty = nx * currentGlobalMatrix[4] + ny * currentGlobalMatrix[5] + nz * currentGlobalMatrix[6];
                                float ntz = nx * currentGlobalMatrix[8] + ny * currentGlobalMatrix[9] + nz * currentGlobalMatrix[10];

                                mergedNormals[currentMergedVertPtr * 3]     = ntx;
                                mergedNormals[currentMergedVertPtr * 3 + 1] = nty;
                                mergedNormals[currentMergedVertPtr * 3 + 2] = ntz;
                            } else {
                                mergedNormals[currentMergedVertPtr * 3]     = 0f;
                                mergedNormals[currentMergedVertPtr * 3 + 1] = 0f;
                                mergedNormals[currentMergedVertPtr * 3 + 2] = 1f;
                            }

                            if (uvBuffer != null) {
                                AIVector3D uv = uvBuffer.get(vIdx);
                                mergedUvs[currentMergedVertPtr * 2]     = uv.x();
                                mergedUvs[currentMergedVertPtr * 2 + 1] = 1.0f - uv.y();
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

                // 🌟 双轨字典快照注册：确保无论是层级呼叫还是切片直接呼叫都能精准命中！
                MESH_CACHE.put(namespace + ":" + cleanNodeName, unifiedGeometry);
                if (!cleanNodeName.equals(rawNodeName)) {
                    MESH_CACHE.put(namespace + ":" + rawNodeName, unifiedGeometry);
                }
                for (AIMesh slice : attachedMeshSlices) {
                    String baseSliceName = slice.mName().dataString();
                    if (!baseSliceName.isEmpty()) {
                        String cleanSlice = baseSliceName.endsWith("-mesh") ? baseSliceName.substring(0, baseSliceName.length() - 5) : baseSliceName;
                        MESH_CACHE.putIfAbsent(namespace + ":" + cleanSlice, unifiedGeometry);
                    }
                }
            }
        }

        int numChildren = node.mNumChildren();
        PointerBuffer childrenBuffer = node.mChildren();
        if (numChildren > 0 && childrenBuffer != null) {
            for (int i = 0; i < numChildren; i++) {
                processSceneNodesRecursively(AINode.create(childrenBuffer.get(i)), scene, namespace, currentGlobalMatrix);
            }
        }
    }

    private static float[] multiplyMatrix4x4(float[] a, float[] b) {
        float[] result = new float[16];
        for (int r=0; r<4; r++) {
            for (int c=0; c<4; c++) {
                float sum=0;
                for (int k=0; k<4; k++) sum += a[r*4+k] * b[k*4+c];
                result[r*4+c] = sum;
            }
        }
        return result;
    }
}