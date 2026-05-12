package me.mzy.beamcraft.client.model;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.*;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

public class DaeMeshLoader {

    public static class RawGeometry {
        public float[] positions;
        public float[] normals;
        public float[] uvs;
        public int vertexCount;
    }

    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    // =========================================================================
    // 内部结构定义 - 彻底遵循 COLLADA 规范层级
    // =========================================================================

    private static class AccessorData {
        float[] rawFloats;
        int stride = 1;
        int offset = 0;
        int count = 0;
    }

    private static class InputBinding {
        String semantic;
        String sourceId;
        int offset;
    }

    private static class AbstractPolygon {
        int[] posIndices;
        int[] normIndices;
        int[] uvIndices;
    }

    private static class PrimitiveGroup {
        List<InputBinding> bindings = new ArrayList<>();
        List<AbstractPolygon> polygons = new ArrayList<>();
        int maxOffset = -1;
    }

    private static class GeometryTemplate {
        String id;
        String name;
        Map<String, AccessorData> accessors = new HashMap<>();
        String posSourceId = "";
        List<PrimitiveGroup> primitiveGroups = new ArrayList<>();
    }

    private static class BakedVertex {
        final Vector3f pos = new Vector3f();
        final Vector3f norm = new Vector3f(0, 1, 0);
        final float[] uv = new float[2];
    }

    // =========================================================================

    public static void scanAndLoadAllVehicles(File vehiclesRootDir) {
        if (!vehiclesRootDir.exists()) {
            vehiclesRootDir.mkdirs();
            return;
        }

        MESH_CACHE.clear();
        File[] files = vehiclesRootDir.listFiles();
        if (files == null) return;

        for (File file : files) {
            String name = file.getName();
            if (file.isDirectory()) {
                scanFolderForDae(file);
            } else if (name.toLowerCase().endsWith(".zip")) {
                scanZipForDae(file);
            }
        }
    }

    private static void scanZipForDae(File zipFile) {
        try (ZipFile zf = new ZipFile(zipFile)) {
            Enumeration<? extends ZipEntry> entries = zf.entries();
            while (entries.hasMoreElements()) {
                ZipEntry entry = entries.nextElement();
                String entryName = entry.getName();
                if (!entry.isDirectory() && !entryName.contains("__MACOSX") && entryName.toLowerCase().endsWith(".dae")) {
                    String trueNamespace = extractTrueVehicleName(entryName);
                    if (trueNamespace == null) continue;
                    try (InputStream is = zf.getInputStream(entry)) {
                        parseDaePipeline(is, trueNamespace);
                    }
                }
            }
        } catch (Exception ignored) {}
    }

    private static void scanFolderForDae(File rootFolder) {
        File innerVehiclesDir = new File(rootFolder, "vehicles");
        if (!innerVehiclesDir.exists() || !innerVehiclesDir.isDirectory()) innerVehiclesDir = rootFolder;
        File[] vehicleFolders = innerVehiclesDir.listFiles();
        if (vehicleFolders == null) return;
        for (File vFolder : vehicleFolders) {
            if (vFolder.isDirectory()) processDaeFilesRecursively(vFolder, vFolder.getName());
        }
    }

    private static void processDaeFilesRecursively(File currentDir, String namespace) {
        File[] files = currentDir.listFiles();
        if (files == null) return;
        for (File f : files) {
            if (f.isDirectory()) {
                processDaeFilesRecursively(f, namespace);
            } else if (f.getName().toLowerCase().endsWith(".dae")) {
                try (InputStream is = new FileInputStream(f)) {
                    parseDaePipeline(is, namespace);
                } catch (Exception ignored) {}
            }
        }
    }

    private static String extractTrueVehicleName(String zipEntryPath) {
        String[] parts = zipEntryPath.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            if (parts[i].equalsIgnoreCase("vehicles")) return parts[i + 1];
        }
        return null;
    }

    // =========================================================================
    // 核心管线解析流
    // =========================================================================

    private static void parseDaePipeline(InputStream is, String namespace) {
        try {
            DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
            dbf.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(is);
            doc.getDocumentElement().normalize();

            // 1. 读取资产全局单位和轴向定义
            float unitMeter = 1.0f;
            String upAxis = "Z_UP";
            NodeList assetList = doc.getElementsByTagName("asset");
            if (assetList.getLength() > 0) {
                Element assetEl = (Element) assetList.item(0);
                NodeList unitList = assetEl.getElementsByTagName("unit");
                if (unitList.getLength() > 0) {
                    String meterStr = ((Element) unitList.item(0)).getAttribute("meter");
                    if (!meterStr.isEmpty()) try { unitMeter = Float.parseFloat(meterStr); } catch (Exception ignored) {}
                }
                NodeList axisList = assetEl.getElementsByTagName("up_axis");
                if (axisList.getLength() > 0) {
                    upAxis = axisList.item(0).getTextContent().trim().toUpperCase();
                }
            }

            // 2. 提取底层独立几何体 AST 模板
            Map<String, GeometryTemplate> templatePool = new HashMap<>();
            NodeList geomList = doc.getElementsByTagName("geometry");
            for (int i = 0; i < geomList.getLength(); i++) {
                Element geomNode = (Element) geomList.item(i);
                GeometryTemplate template = extractGeometryTemplate(geomNode);
                if (template != null) {
                    templatePool.put(template.id, template);
                }
            }

            // 3. 场景树遍历与同名实体收集
            // Key: 实例名称 -> Value: 三个点组成的连续顶点流
            Map<String, List<BakedVertex>> outputAccumulator = new HashMap<>();
            Map<String, Set<String>> aliasMap = new HashMap<>();

            NodeList sceneList = doc.getElementsByTagName("visual_scene");
            if (sceneList.getLength() > 0) {
                Element sceneRoot = (Element) sceneList.item(0);
                NodeList topNodes = sceneRoot.getChildNodes();
                for (int i = 0; i < topNodes.getLength(); i++) {
                    Node child = topNodes.item(i);
                    if (child instanceof Element && "node".equals(child.getNodeName())) {
                        traverseAndInstantiate((Element) child, new Matrix4f(), templatePool, outputAccumulator, aliasMap, unitMeter, upAxis);
                    }
                }
            } else {
                // 退化保底：当场景图缺失时，默认输出基础模板
                for (GeometryTemplate tmpl : templatePool.values()) {
                    String primaryName = tmpl.id;
                    Set<String> aliases = aliasMap.computeIfAbsent(primaryName, k -> new HashSet<>());
                    aliases.add(tmpl.id);
                    if (tmpl.name != null && !tmpl.name.isEmpty()) aliases.add(tmpl.name);
                    if (tmpl.id.endsWith("-mesh")) aliases.add(tmpl.id.substring(0, tmpl.id.length() - 5));

                    instantiateGeometry(tmpl, new Matrix4f(), primaryName, outputAccumulator, unitMeter, upAxis);
                }
            }

            // 4. 连续数组平铺与快照广播
            for (Map.Entry<String, List<BakedVertex>> entry : outputAccumulator.entrySet()) {
                String primaryName = entry.getKey();
                List<BakedVertex> vertices = entry.getValue();
                if (vertices.isEmpty() || vertices.size() % 3 != 0) continue;

                int vCount = vertices.size();
                float[] flatPos = new float[vCount * 3];
                float[] flatNorm = new float[vCount * 3];
                float[] flatUv = new float[vCount * 2];

                int pIdx = 0, nIdx = 0, uIdx = 0;
                for (BakedVertex bv : vertices) {
                    flatPos[pIdx++] = bv.pos.x; flatPos[pIdx++] = bv.pos.y; flatPos[pIdx++] = bv.pos.z;
                    flatNorm[nIdx++] = bv.norm.x; flatNorm[nIdx++] = bv.norm.y; flatNorm[nIdx++] = bv.norm.z;
                    flatUv[uIdx++] = bv.uv[0]; flatUv[uIdx++] = bv.uv[1];
                }

                RawGeometry finalGeom = new RawGeometry();
                finalGeom.positions = flatPos;
                finalGeom.normals = flatNorm;
                finalGeom.uvs = flatUv;
                finalGeom.vertexCount = vCount;

                Set<String> aliases = aliasMap.getOrDefault(primaryName, Collections.singleton(primaryName));
                for (String alias : aliases) {
                    if (alias != null && !alias.isEmpty()) {
                        MESH_CACHE.put(namespace + ":" + alias, finalGeom);
                    }
                }
            }

        } catch (Exception ignored) {}
    }

    // =========================================================================
    // 阶段一：遵循规范解析 Accessor 与间接语义寻址
    // =========================================================================

    private static GeometryTemplate extractGeometryTemplate(Element geomNode) {
        String id = geomNode.getAttribute("id");
        if (id.isEmpty()) return null;

        NodeList meshList = geomNode.getElementsByTagName("mesh");
        if (meshList.getLength() == 0) return null;
        Element meshNode = (Element) meshList.item(0);

        GeometryTemplate template = new GeometryTemplate();
        template.id = id;
        template.name = geomNode.getAttribute("name");

        // 1. 抓取浮点源并根据 accessor 包装
        Map<String, float[]> rawArrays = new HashMap<>();
        NodeList sourceList = meshNode.getElementsByTagName("source");
        for (int si = 0; si < sourceList.getLength(); si++) {
            Element srcEl = (Element) sourceList.item(si);
            String srcId = srcEl.getAttribute("id");

            NodeList fList = srcEl.getElementsByTagName("float_array");
            if (fList.getLength() > 0) {
                Element fEl = (Element) fList.item(0);
                float[] arr = parseStringToFloatArray(fEl.getTextContent());
                String arrayId = fEl.getAttribute("id");
                if (!arrayId.isEmpty()) rawArrays.put(arrayId, arr);
                rawArrays.put(srcId, arr);
            }

            NodeList tCommonList = srcEl.getElementsByTagName("technique_common");
            if (tCommonList.getLength() > 0) {
                NodeList accList = ((Element) tCommonList.item(0)).getElementsByTagName("accessor");
                if (accList.getLength() > 0) {
                    Element accEl = (Element) accList.item(0);
                    AccessorData accData = new AccessorData();

                    String strVal = accEl.getAttribute("stride");
                    if (!strVal.isEmpty()) try { accData.stride = Integer.parseInt(strVal); } catch (Exception ignored) {}
                    String offVal = accEl.getAttribute("offset");
                    if (!offVal.isEmpty()) try { accData.offset = Integer.parseInt(offVal); } catch (Exception ignored) {}
                    String cntVal = accEl.getAttribute("count");
                    if (!cntVal.isEmpty()) try { accData.count = Integer.parseInt(cntVal); } catch (Exception ignored) {}

                    // 延迟到建立映射时注入数组
                    String targetSource = accEl.getAttribute("source").replace("#", "");
                    accData.rawFloats = rawArrays.get(targetSource.isEmpty() ? srcId : targetSource);
                    if (accData.rawFloats == null) accData.rawFloats = rawArrays.get(srcId);

                    if (accData.rawFloats != null) {
                        template.accessors.put(srcId, accData);
                    }
                }
            }
        }

        // 2. 建立 vertices 间接解析寻址表
        Map<String, String> vertexSemanticMap = new HashMap<>();
        NodeList vertList = meshNode.getElementsByTagName("vertices");
        if (vertList.getLength() > 0) {
            Element vertEl = (Element) vertList.item(0);
            String vertId = vertEl.getAttribute("id");
            NodeList vInputs = vertEl.getElementsByTagName("input");
            for (int vi = 0; vi < vInputs.getLength(); vi++) {
                Element viEl = (Element) vInputs.item(vi);
                String sem = viEl.getAttribute("semantic");
                String src = viEl.getAttribute("source").replace("#", "");
                vertexSemanticMap.put(sem, src);
                if ("POSITION".equals(sem)) template.posSourceId = src;
            }
            // 兼容将 id 注册为桥接
            if (!template.posSourceId.isEmpty()) {
                template.accessors.put(vertId, template.accessors.get(template.posSourceId));
            }
        }

        // 3. 提取全图元拓扑块
        String[] primTags = {"triangles", "polylist", "polygons"};
        for (String tag : primTags) {
            NodeList pNodes = meshNode.getElementsByTagName(tag);
            for (int p = 0; p < pNodes.getLength(); p++) {
                extractPrimitiveBlock((Element) pNodes.item(p), template, vertexSemanticMap);
            }
        }

        if (template.primitiveGroups.isEmpty() || !template.accessors.containsKey(template.posSourceId)) return null;
        return template;
    }

    private static void extractPrimitiveBlock(Element primNode, GeometryTemplate template, Map<String, String> vertMap) {
        PrimitiveGroup group = new PrimitiveGroup();
        int maxOffset = -1;

        // 解析图元输入映射绑定
        NodeList inputs = primNode.getElementsByTagName("input");
        for (int i = 0; i < inputs.getLength(); i++) {
            Element inpEl = (Element) inputs.item(i);
            String offStr = inpEl.getAttribute("offset");
            if (offStr.isEmpty()) continue;
            int offset = Integer.parseInt(offStr);
            if (offset > maxOffset) maxOffset = offset;

            String sem = inpEl.getAttribute("semantic");
            String src = inpEl.getAttribute("source").replace("#", "");

            // 间接展开 VERTEX 语义
            if ("VERTEX".equals(sem)) {
                // 绑定到真实的 POSITION 数据源
                String realPosSrc = vertMap.getOrDefault("POSITION", src);
                InputBinding pb = new InputBinding(); pb.semantic = "POSITION"; pb.sourceId = realPosSrc; pb.offset = offset;
                group.bindings.add(pb);

                // 若 vertices 中携带了 NORMAL，一并展开
                if (vertMap.containsKey("NORMAL")) {
                    InputBinding nb = new InputBinding(); nb.semantic = "NORMAL"; nb.sourceId = vertMap.get("NORMAL"); nb.offset = offset;
                    group.bindings.add(nb);
                }
            } else {
                InputBinding b = new InputBinding(); b.semantic = sem; b.sourceId = src; b.offset = offset;
                group.bindings.add(b);
            }
        }

        int stride = maxOffset + 1;
        if (stride <= 0) return;
        group.maxOffset = maxOffset;

        // 检索特定的通道偏移
        int pOff = -1, nOff = -1, tOff = -1;
        for (InputBinding b : group.bindings) {
            if ("POSITION".equals(b.semantic)) pOff = b.offset;
            else if ("NORMAL".equals(b.semantic)) nOff = b.offset;
            else if ("TEXCOORD".equals(b.semantic) && tOff == -1) tOff = b.offset; // 优先获取首个 UV set
        }
        if (pOff == -1) return;

        boolean isPolygons = "polygons".equals(primNode.getNodeName());
        NodeList pList = primNode.getElementsByTagName("p");
        if (pList.getLength() == 0) return;

        if (isPolygons) {
            // <polygons> 多 <p> 独立成面结构
            for (int f = 0; f < pList.getLength(); f++) {
                int[] rawIndices = parseStringToIntArray(pList.item(f).getTextContent());
                int vCount = rawIndices.length / stride;
                if (vCount < 3) continue;

                AbstractPolygon poly = new AbstractPolygon();
                poly.posIndices = new int[vCount];
                poly.normIndices = new int[vCount];
                poly.uvIndices = new int[vCount];

                int ptr = 0;
                for (int v = 0; v < vCount; v++) {
                    poly.posIndices[v] = rawIndices[ptr + pOff];
                    poly.normIndices[v] = nOff != -1 ? rawIndices[ptr + nOff] : -1;
                    poly.uvIndices[v] = tOff != -1 ? rawIndices[ptr + tOff] : -1;
                    ptr += stride;
                }
                group.polygons.add(poly);
            }
        } else {
            // <triangles> 或 <polylist> 连续数据流
            int[] rawIndices = parseStringToIntArray(pList.item(0).getTextContent());
            int[] vcounts = null;

            if ("polylist".equals(primNode.getNodeName())) {
                NodeList vcList = primNode.getElementsByTagName("vcount");
                if (vcList.getLength() > 0) {
                    vcounts = parseStringToIntArray(vcList.item(0).getTextContent());
                }
            }

            int ptr = 0;
            if (vcounts != null) {
                for (int vc : vcounts) {
                    if (vc < 3) { ptr += vc * stride; continue; }
                    if (ptr + vc * stride > rawIndices.length) break;

                    AbstractPolygon poly = new AbstractPolygon();
                    poly.posIndices = new int[vc];
                    poly.normIndices = new int[vc];
                    poly.uvIndices = new int[vc];
                    for (int v = 0; v < vc; v++) {
                        poly.posIndices[v] = rawIndices[ptr + pOff];
                        poly.normIndices[v] = nOff != -1 ? rawIndices[ptr + nOff] : -1;
                        poly.uvIndices[v] = tOff != -1 ? rawIndices[ptr + tOff] : -1;
                        ptr += stride;
                    }
                    group.polygons.add(poly);
                }
            } else {
                while (ptr + 3 * stride <= rawIndices.length) {
                    AbstractPolygon poly = new AbstractPolygon();
                    poly.posIndices = new int[3];
                    poly.normIndices = new int[3];
                    poly.uvIndices = new int[3];
                    for (int v = 0; v < 3; v++) {
                        poly.posIndices[v] = rawIndices[ptr + pOff];
                        poly.normIndices[v] = nOff != -1 ? rawIndices[ptr + nOff] : -1;
                        poly.uvIndices[v] = tOff != -1 ? rawIndices[ptr + tOff] : -1;
                        ptr += stride;
                    }
                    group.polygons.add(poly);
                }
            }
        }

        if (!group.polygons.isEmpty()) {
            template.primitiveGroups.add(group);
        }
    }

    private static float[] parseStringToFloatArray(String str) {
        String[] parts = str.trim().split("\\s+");
        float[] arr = new float[parts.length];
        int count = 0;
        for (String s : parts) {
            if (!s.isEmpty()) try { arr[count++] = Float.parseFloat(s); } catch (Exception ignored) {}
        }
        return count == arr.length ? arr : Arrays.copyOf(arr, count);
    }

    private static int[] parseStringToIntArray(String str) {
        String[] parts = str.trim().split("\\s+");
        int[] arr = new int[parts.length];
        int count = 0;
        for (String s : parts) {
            if (!s.isEmpty()) try { arr[count++] = Integer.parseInt(s); } catch (Exception ignored) {}
        }
        return count == arr.length ? arr : Arrays.copyOf(arr, count);
    }

    // =========================================================================
    // 阶段二：安全遍历场景树（只查直接子元素，根治重复堆叠）
    // =========================================================================

    private static void traverseAndInstantiate(Element nodeEl, Matrix4f parentMatrix,
                                               Map<String, GeometryTemplate> templates,
                                               Map<String, List<BakedVertex>> accumulator,
                                               Map<String, Set<String>> aliasMap,
                                               float unitMeter, String upAxis) {
        Matrix4f localMatrix = new Matrix4f(parentMatrix);

        // 仅遍历当前节点的直接子元素，绝不越级深搜
        NodeList children = nodeEl.getChildNodes();
        for (int i = 0; i < children.getLength(); i++) {
            Node childNode = children.item(i);
            if (!(childNode instanceof Element)) continue;
            Element child = (Element) childNode;
            String name = child.getNodeName();
            String[] vals = child.getTextContent().trim().split("\\s+");

            try {
                if ("translate".equals(name) && vals.length >= 3) {
                    localMatrix.translate(Float.parseFloat(vals[0]) * unitMeter,
                            Float.parseFloat(vals[1]) * unitMeter,
                            Float.parseFloat(vals[2]) * unitMeter);
                } else if ("rotate".equals(name) && vals.length >= 4) {
                    localMatrix.rotate((float) Math.toRadians(Float.parseFloat(vals[3])),
                            Float.parseFloat(vals[0]), Float.parseFloat(vals[1]), Float.parseFloat(vals[2]));
                } else if ("scale".equals(name) && vals.length >= 3) {
                    localMatrix.scale(Float.parseFloat(vals[0]), Float.parseFloat(vals[1]), Float.parseFloat(vals[2]));
                } else if ("matrix".equals(name) && vals.length >= 16) {
                    float[] f = new float[16];
                    for (int v = 0; v < 16; v++) f[v] = Float.parseFloat(vals[v]);
                    Matrix4f mat = new Matrix4f(
                            f[0], f[4], f[8],  f[12],
                            f[1], f[5], f[9],  f[13],
                            f[2], f[6], f[10], f[14],
                            f[3], f[7], f[11], f[15]
                    );
                    mat.m30(mat.m30() * unitMeter);
                    mat.m31(mat.m31() * unitMeter);
                    mat.m32(mat.m32() * unitMeter);
                    localMatrix.mul(mat);
                }
            } catch (Exception ignored) {}
        }

        String nodeName = nodeEl.getAttribute("name");
        String nodeId = nodeEl.getAttribute("id");

        // 仅匹配挂载在当前节点第一层的几何体实例
        for (int i = 0; i < children.getLength(); i++) {
            Node childNode = children.item(i);
            if (childNode instanceof Element && "instance_geometry".equals(childNode.getNodeName())) {
                Element instEl = (Element) childNode;
                String url = instEl.getAttribute("url").replace("#", "");
                GeometryTemplate baseGeom = templates.get(url);

                if (baseGeom != null) {
                    String primaryName = nodeName;
                    if (primaryName == null || primaryName.isEmpty()) primaryName = nodeId;
                    if (primaryName == null || primaryName.isEmpty()) primaryName = url;

                    Set<String> aliases = aliasMap.computeIfAbsent(primaryName, k -> new HashSet<>());
                    aliases.add(primaryName);
                    if (nodeName != null && !nodeName.isEmpty()) aliases.add(nodeName);
                    if (nodeId != null && !nodeId.isEmpty()) aliases.add(nodeId);
                    aliases.add(url);
                    if (url.endsWith("-mesh")) aliases.add(url.substring(0, url.length() - 5));
                    if (baseGeom.name != null && !baseGeom.name.isEmpty()) aliases.add(baseGeom.name);

                    instantiateGeometry(baseGeom, localMatrix, primaryName, accumulator, unitMeter, upAxis);
                }
            }
        }

        // 继续递归向直接子级节点传递矩阵
        for (int i = 0; i < children.getLength(); i++) {
            Node childNode = children.item(i);
            if (childNode instanceof Element && "node".equals(childNode.getNodeName())) {
                traverseAndInstantiate((Element) childNode, localMatrix, templates, accumulator, aliasMap, unitMeter, upAxis);
            }
        }
    }

    // =========================================================================
    // 阶段三：绝对空间矩阵烘焙、法线逆转置与 MC 终点映射
    // =========================================================================

    private static void instantiateGeometry(GeometryTemplate template, Matrix4f matrix,
                                            String primaryName,
                                            Map<String, List<BakedVertex>> accumulator,
                                            float unitMeter, String upAxis) {
        AccessorData posAcc = template.accessors.get(template.posSourceId);
        if (posAcc == null || posAcc.rawFloats == null) return;

        List<BakedVertex> targetPool = accumulator.computeIfAbsent(primaryName, k -> new ArrayList<>());

        // 计算用于法线变换的逆转置矩阵 (消除缩放对法向的影响)
        Matrix4f normalMatrix = new Matrix4f();
        matrix.invert(normalMatrix).transpose();

        boolean swapWindingOrder = matrix.determinant() < 0;
        Vector4f vPos = new Vector4f();
        Vector3f vNorm = new Vector3f();

        for (PrimitiveGroup group : template.primitiveGroups) {
            // 追踪当前图元组绑定的法线和 UV 源
            AccessorData normAcc = null;
            AccessorData uvAcc = null;
            for (InputBinding b : group.bindings) {
                if ("NORMAL".equals(b.semantic)) normAcc = template.accessors.get(b.sourceId);
                else if ("TEXCOORD".equals(b.semantic) && uvAcc == null) uvAcc = template.accessors.get(b.sourceId);
            }

            for (AbstractPolygon poly : group.polygons) {
                int vc = poly.posIndices.length;
                if (vc < 3) continue;

                // 扇形展开建立基础面片
                int p0 = poly.posIndices[0], n0 = poly.normIndices[0], u0 = poly.uvIndices[0];

                for (int k = 2; k < vc; k++) {
                    int p1 = poly.posIndices[k - 1], n1 = poly.normIndices[k - 1], u1 = poly.uvIndices[k - 1];
                    int p2 = poly.posIndices[k], n2 = poly.normIndices[k], u2 = poly.uvIndices[k];

                    BakedVertex bv0 = bakeSingleVertex(p0, n0, u0, posAcc, normAcc, uvAcc, matrix, normalMatrix, unitMeter, upAxis, vPos, vNorm);
                    BakedVertex bv1 = bakeSingleVertex(p1, n1, u1, posAcc, normAcc, uvAcc, matrix, normalMatrix, unitMeter, upAxis, vPos, vNorm);
                    BakedVertex bv2 = bakeSingleVertex(p2, n2, u2, posAcc, normAcc, uvAcc, matrix, normalMatrix, unitMeter, upAxis, vPos, vNorm);

                    if (bv0 == null || bv1 == null || bv2 == null || isDegenerateTriangle(bv0, bv1, bv2)) continue;

                    // 遇到无顶点法线时，利用叉积实时计算几何面平坦法线
                    if (normAcc == null) {
                        Vector3f edge1 = new Vector3f(bv1.pos).sub(bv0.pos);
                        Vector3f edge2 = new Vector3f(bv2.pos).sub(bv0.pos);
                        Vector3f faceNorm = edge1.cross(edge2).normalize();
                        bv0.norm.set(faceNorm); bv1.norm.set(faceNorm); bv2.norm.set(faceNorm);
                    }

                    // 修复负缩放致使的背面剔除隐形
                    if (swapWindingOrder) {
                        targetPool.add(bv0); targetPool.add(bv2); targetPool.add(bv1);
                    } else {
                        targetPool.add(bv0); targetPool.add(bv1); targetPool.add(bv2);
                    }
                }
            }
        }
    }

    private static BakedVertex bakeSingleVertex(int pIdx, int nIdx, int uIdx,
                                                AccessorData posAcc, AccessorData normAcc, AccessorData uvAcc,
                                                Matrix4f mat, Matrix4f normMat,
                                                float scale, String upAxis,
                                                Vector4f vPos, Vector3f vNorm) {
        if (!isValidAccessorIndex(posAcc, pIdx, 3)) return null;
        int pBase = posAcc.offset + pIdx * posAcc.stride;
        if (pBase + 2 >= posAcc.rawFloats.length) return null;

        BakedVertex bv = new BakedVertex();

        // 1. 读取原始局部坐标并应用缩放系数至标准米
        float rx = posAcc.rawFloats[pBase] * scale;
        float ry = posAcc.rawFloats[pBase + 1] * scale;
        float rz = posAcc.rawFloats[pBase + 2] * scale;

        // 2. 累加绝对场景树矩阵变换
        vPos.set(rx, ry, rz, 1.0f);
        mat.transform(vPos);

        // 3. 终点权威映射：将任意源轴向无缝过渡至 Minecraft 世界空间 (Y_UP)
        if ("Y_UP".equals(upAxis)) {
            // Z_UP 到 Y_UP 的标准转换法则
            bv.pos.set(vPos.x, -vPos.z, vPos.y);
        } else {
            // 模型原生即为 Y_UP，直接接收
            bv.pos.set(vPos.x, vPos.y, vPos.z);
        }

        // 4. 处理法线逆转置与空间转换
        if (normAcc != null && nIdx >= 0 && isValidAccessorIndex(normAcc, nIdx, 3)) {
            int nBase = normAcc.offset + nIdx * normAcc.stride;
            if (nBase + 2 < normAcc.rawFloats.length) {
                vNorm.set(normAcc.rawFloats[nBase], normAcc.rawFloats[nBase + 1], normAcc.rawFloats[nBase + 2]);
                normMat.transformDirection(vNorm).normalize();

                if ("Y_UP".equals(upAxis)) {
                    bv.norm.set(vNorm.x, -vNorm.z, vNorm.y);
                } else {
                    bv.norm.set(vNorm.x, vNorm.y, vNorm.z);
                }
            }
        }

        // 5. 提取 UV 通道 (执行 V 轴图像接口倒置)
        if (uvAcc != null && uIdx >= 0 && isValidAccessorIndex(uvAcc, uIdx, 2)) {
            int uBase = uvAcc.offset + uIdx * uvAcc.stride;
            if (uBase + 1 < uvAcc.rawFloats.length) {
                bv.uv[0] = uvAcc.rawFloats[uBase];
                bv.uv[1] = 1.0f - uvAcc.rawFloats[uBase + 1];
            }
        }

        return bv;
    }

    private static boolean isValidAccessorIndex(AccessorData acc, int index, int components) {
        if (acc == null || acc.rawFloats == null || index < 0 || acc.stride <= 0) return false;
        if (acc.count > 0 && index >= acc.count) return false;
        int base = acc.offset + index * acc.stride;
        return base >= 0 && base + components - 1 < acc.rawFloats.length;
    }

    private static boolean isDegenerateTriangle(BakedVertex a, BakedVertex b, BakedVertex c) {
        float abx = b.pos.x - a.pos.x, aby = b.pos.y - a.pos.y, abz = b.pos.z - a.pos.z;
        float acx = c.pos.x - a.pos.x, acy = c.pos.y - a.pos.y, acz = c.pos.z - a.pos.z;
        float cx = aby * acz - abz * acy;
        float cy = abz * acx - abx * acz;
        float cz = abx * acy - aby * acx;
        return cx * cx + cy * cy + cz * cz < 1.0e-12f;
    }
}
