package me.mzy.beamcraft.client.model;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

public class DaeMeshLoader {

    public static class RawGeometry {
        public float[] positions;
        public int vertexCount;
    }

    // 🚀 升级后的全局缓存，Key 格式为 "vehicleName:meshName"
    public static final Map<String, RawGeometry> MESH_CACHE = new HashMap<>();

    /**
     * 入口：传入 vehicles 根目录 (run/mods/beamcraft/vehicles/)
     */
    public static void scanAndLoadAllVehicles(File vehiclesRootDir) {
        if (!vehiclesRootDir.exists()) {
            vehiclesRootDir.mkdirs();
            return;
        }

        System.out.println("====== 🔍 启动 DAE 视觉资产全能扫描 ======");
        MESH_CACHE.clear();

        File[] files = vehiclesRootDir.listFiles();
        if (files == null) return;

        for (File file : files) {
            String name = file.getName();

            // 1. 处理普通文件夹解压资产
            if (file.isDirectory()) {
                scanFolderForDae(file);
            }
            // 2. 处理 ZIP 压缩包资产
            else if (name.toLowerCase().endsWith(".zip")) {
                scanZipForDae(file);
            }
        }

        System.out.println("📦 视觉资产载入完毕！当前显存快照总数: " + MESH_CACHE.size());
    }

    // ================= 穿透读取 ZIP 压缩包 =================
    private static void scanZipForDae(File zipFile) {
        try (ZipFile zf = new ZipFile(zipFile)) {
            Enumeration<? extends ZipEntry> entries = zf.entries();

            while (entries.hasMoreElements()) {
                ZipEntry entry = entries.nextElement();
                String entryName = entry.getName();

                // 过滤 Mac 垃圾文件，只抓取 vehicles 目录下的 .dae
                if (!entry.isDirectory() && !entryName.contains("__MACOSX") && entryName.toLowerCase().endsWith(".dae")) {

                    // 🚀 核心修复：从路径内部精准提取真实的车辆代号！
                    // 路径格式通常为: "vehicles/pickup/pickup_chassis.dae"
                    String trueNamespace = extractTrueVehicleName(entryName);
                    if (trueNamespace == null) continue;

                    try (InputStream is = zf.getInputStream(entry)) {
                        parseDaeStream(is, trueNamespace, entryName);
                    }
                }
            }
        } catch (Exception e) {
            System.err.println("🚨 穿透读取 ZIP 失败: " + zipFile.getName());
        }
    }

    // ================= 递归普通文件夹 =================
    private static void scanFolderForDae(File rootFolder) {
        // 直接寻找解压后的 vehicles 目录
        File innerVehiclesDir = new File(rootFolder, "vehicles");
        if (!innerVehiclesDir.exists() || !innerVehiclesDir.isDirectory()) {
            // 如果顶层没有 vehicles 目录，尝试直接遍历（兼容裸文件）
            innerVehiclesDir = rootFolder;
        }

        File[] vehicleFolders = innerVehiclesDir.listFiles();
        if (vehicleFolders == null) return;

        for (File vFolder : vehicleFolders) {
            if (vFolder.isDirectory()) {
                // 🚀 此时 vFolder 的名字就是绝对纯正的真实车辆名！(例如 "pickup")
                String trueNamespace = vFolder.getName();
                processDaeFilesRecursively(vFolder, trueNamespace);
            }
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
                    parseDaeStream(is, namespace, f.getName());
                } catch (Exception ignored) {}
            }
        }
    }

    // 🛠️ 辅助提取路径中的真实车名工具
    private static String extractTrueVehicleName(String zipEntryPath) {
        String[] parts = zipEntryPath.split("/");
        for (int i = 0; i < parts.length - 1; i++) {
            if (parts[i].equalsIgnoreCase("vehicles")) {
                // 返回 "vehicles/" 后面的那一个层级
                return parts[i + 1];
            }
        }
        return null;
    }

    // ================= 核心 XML 提取器 (兼容 InputStream) =================
    private static void parseDaeStream(InputStream is, String namespace, String sourceName) {
        try {
            DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
            // 防止 XML 实体注入攻击，并提升纯数据解析性能
            dbf.setFeature("http://apache.org/xml/features/disallow-doctype-decl", true);
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(is);
            doc.getDocumentElement().normalize();

            NodeList geomList = doc.getElementsByTagName("geometry");

            for (int i = 0; i < geomList.getLength(); i++) {
                Element geomNode = (Element) geomList.item(i);
                String rawMeshId = geomNode.getAttribute("id");

                NodeList floatArrayList = geomNode.getElementsByTagName("float_array");
                if (floatArrayList.getLength() == 0) continue;

                Element floatArrayElement = (Element) floatArrayList.item(0);
                String rawNumbers = floatArrayElement.getTextContent().trim();
                if (rawNumbers.isEmpty()) continue;

                String[] strValues = rawNumbers.split("\\s+");
                ArrayList<Float> safeFloats = new ArrayList<>(strValues.length);

                for (String s : strValues) {
                    if (!s.isEmpty()) {
                        try { safeFloats.add(Float.parseFloat(s)); } catch (NumberFormatException ignored) {}
                    }
                }

                if (safeFloats.isEmpty()) continue;

                float[] posArray = new float[safeFloats.size()];
                for (int j = 0; j < safeFloats.size(); j++) {
                    posArray[j] = safeFloats.get(j);
                }

                RawGeometry geom = new RawGeometry();
                geom.positions = posArray;
                geom.vertexCount = posArray.length / 3;

                // 🚀 终极防护：强制组合命名空间，绝对杜绝跨车同名覆盖！
                String scopedKey = namespace + ":" + rawMeshId;
                MESH_CACHE.put(scopedKey, geom);
                //System.out.println("   -> 成功提取网格: [" + scopedKey + "] | 顶点数: " + geom.vertexCount);
            }
        } catch (Exception e) {
            System.err.println("⚠️ 解析 DAE 流异常 (" + sourceName + "): " + e.getMessage());
        }
    }
}