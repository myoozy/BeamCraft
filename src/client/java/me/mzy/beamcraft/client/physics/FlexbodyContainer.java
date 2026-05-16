package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.render.VehicleRenderBuffer;
import me.mzy.beamcraft.utility.Utility;

import java.util.*;

/**
 * Flexbody 容器 SoA
 * 负责存储网格绑定定义、O(1) 节点组(CSR表) 以及展平的顶点蒙皮数据
 */
public class FlexbodyContainer {
    public static final int INIT_FLEX_CAP = 16;
    public String vehicleNamespace = "";
    public boolean isSkinningBound = false;

    public VehicleRenderBuffer vboBuffer = new VehicleRenderBuffer();

    // ==========================================
    // 1. 原始 JBeam 定义层 (Mesh 级别 SoA)
    // ==========================================
    public int meshCount = 0;
    public String[] meshName = new String[INIT_FLEX_CAP];
    // 存储每个 mesh 绑定的目标 Group 名称列表
    public List<String>[] targetGroups = new List[INIT_FLEX_CAP];
    // 基础变换矩阵参数
    public double[] posX = new double[INIT_FLEX_CAP], posY = new double[INIT_FLEX_CAP], posZ = new double[INIT_FLEX_CAP];
    public double[] rotX = new double[INIT_FLEX_CAP], rotY = new double[INIT_FLEX_CAP], rotZ = new double[INIT_FLEX_CAP];
    public double[] scaleX = new double[INIT_FLEX_CAP], scaleY = new double[INIT_FLEX_CAP], scaleZ = new double[INIT_FLEX_CAP];
    public int[] partId = new int[INIT_FLEX_CAP];
    public JBeamAssembler.TransformContext[] slotContext = new JBeamAssembler.TransformContext[INIT_FLEX_CAP];

    // ==========================================
    // 2. O(1) 静态节点组查询表 (CSR 格式)
    // ==========================================
    public final Map<String, Integer> groupNameToId = new HashMap<>();
    public int[] groupNodeOffsets = new int[0];
    public int[] groupNodeCounts = new int[0];
    public int[] flatGroupNodes = new int[0];

    // ==========================================
    // 3. 运行时蒙皮数据层 (Vertex 级别展平 SoA)
    // ==========================================
    public int totalVertexCount = 0;
    // 映射的物理节点索引 (根据官方文档，映射到 Center, Vx, Vy, Vz)
    public int[] vCenterNode = new int[INIT_FLEX_CAP];
    public int[] vVxNode = new int[INIT_FLEX_CAP];
    public int[] vVyNode = new int[INIT_FLEX_CAP];
    public int[] vVzNode = new int[INIT_FLEX_CAP];
    // 对应的逆变空间蒙皮权重
    public float[] vWeightX = new float[INIT_FLEX_CAP];
    public float[] vWeightY = new float[INIT_FLEX_CAP];
    public float[] vWeightZ = new float[INIT_FLEX_CAP];
    public float[] vNormWeightX = new float[INIT_FLEX_CAP];
    public float[] vNormWeightY = new float[INIT_FLEX_CAP];
    public float[] vNormWeightZ = new float[INIT_FLEX_CAP];
    public boolean[] vUseCrossZ; // 当找不到正交 Vz 时设为 true，靠叉乘推导

    // CPU 蒙皮实时计算输出缓冲 (供 VBO 极速拉取)
    public float[] skinnedPosX = new float[INIT_FLEX_CAP];
    public float[] skinnedPosY = new float[INIT_FLEX_CAP];
    public float[] skinnedPosZ = new float[INIT_FLEX_CAP];
    public float[] normalX = new float[INIT_FLEX_CAP];
    public float[] normalY = new float[INIT_FLEX_CAP];
    public float[] normalZ = new float[INIT_FLEX_CAP];
    public float[] uvU = new float[INIT_FLEX_CAP];
    public float[] uvV = new float[INIT_FLEX_CAP];

    private void ensureCapacity() {
        // 判断是否需要扩容：当前 meshCount 达到数组长度上限
        if (meshCount >= meshName.length) {
            int newSize = meshName.length * 2;

            // ==========================
            // 1. Mesh 定义层 SoA 数组
            // ==========================
            meshName = Utility.expand(meshName, newSize);
            targetGroups = java.util.Arrays.copyOf(targetGroups, newSize);

            posX = Utility.expand(posX, newSize);
            posY = Utility.expand(posY, newSize);
            posZ = Utility.expand(posZ, newSize);

            rotX = Utility.expand(rotX, newSize);
            rotY = Utility.expand(rotY, newSize);
            rotZ = Utility.expand(rotZ, newSize);

            scaleX = Utility.expand(scaleX, newSize);
            scaleY = Utility.expand(scaleY, newSize);
            scaleZ = Utility.expand(scaleZ, newSize);

            partId = Utility.expand(partId, newSize);
            slotContext = java.util.Arrays.copyOf(slotContext, newSize);

            // ==========================
            // 2. 运行时蒙皮数据层 SoA 数组
            // ==========================
            vCenterNode = Utility.expand(vCenterNode, newSize);
            vVxNode = Utility.expand(vVxNode, newSize);
            vVyNode = Utility.expand(vVyNode, newSize);
            vVzNode = Utility.expand(vVzNode, newSize);

            vWeightX = Utility.expand(vWeightX, newSize);
            vWeightY = Utility.expand(vWeightY, newSize);
            vWeightZ = Utility.expand(vWeightZ, newSize);

            // 懒初始化数组特殊处理
            if (vUseCrossZ != null) {
                vUseCrossZ = Utility.expand(vUseCrossZ, newSize);
            }

            skinnedPosX = Utility.expand(skinnedPosX, newSize);
            skinnedPosY = Utility.expand(skinnedPosY, newSize);
            skinnedPosZ = Utility.expand(skinnedPosZ, newSize);

            normalX = Utility.expand(normalX, newSize);
            normalY = Utility.expand(normalY, newSize);
            normalZ = Utility.expand(normalZ, newSize);

            uvU = Utility.expand(uvU, newSize);
            uvV = Utility.expand(uvV, newSize);

            System.out.println("⚠️ [FlexbodyContainer] Resized flex capacity to: " + newSize);
        }
    }

    public void clear() {
        meshCount = 0;
        totalVertexCount = 0;
        groupNameToId.clear();
        isSkinningBound = false;
        // 🌟 清理数据时，顺便释放显存，防止内存泄漏！
        if (vboBuffer != null) {
            vboBuffer.free();
        }
    }

    /**
     * 传入BeamNG原始坐标系
     * @param name
     * @param groups
     * @param px
     * @param py
     * @param pz
     * @param rx
     * @param ry
     * @param rz
     * @param sx
     * @param sy
     * @param sz
     * @param pId
     * @param ctx 插槽的变换上下文
     * @return
     */
    public int registerFlexbody(String name, String namespace, List<String> groups,
                                double px, double py, double pz,
                                double rx, double ry, double rz,
                                double sx, double sy, double sz,
                                int pId, JBeamAssembler.TransformContext ctx) {
        ensureCapacity();

        int idx = meshCount;
        meshName[idx] = name;
        targetGroups[idx] = groups;
        // 保持存入原始未转换的 JSON 参数
        posX[idx] = px; posY[idx] = py; posZ[idx] = pz;
        rotX[idx] = rx; rotY[idx] = ry; rotZ[idx] = rz;
        scaleX[idx] = sx; scaleY[idx] = sy; scaleZ[idx] = sz;
        partId[idx] = pId;
        slotContext[idx] = ctx;
        vehicleNamespace = namespace;

        meshCount++;
        return idx;
    }

    /**
     * 在车辆 Setup 阶段调用，将离散的 Assigned Groups 编译为绝对连续的 CSR 寻址表
     */
    public void compileGroupsCSR(NodeContainer nodes) {
        // 不需要单独的 gCounter 变量
        Map<Integer, List<Integer>> tempGToList = new HashMap<>();

        for (int i = 0; i < nodes.count; i++) {
            List<String> groups = nodes.assignedGroups[i];
            if (groups == null) continue;

            for (String gName : groups) {
                // 🚀 直接用 groupNameToId.size()。每次放新 Key 时，size 都会加 1
                // 这样既避开了 Lambda 闭包限制，逻辑也更简洁。
                Integer gId = groupNameToId.get(gName);
                if (gId == null) {
                    gId = groupNameToId.size();
                    groupNameToId.put(gName, gId);
                }
                tempGToList.computeIfAbsent(gId, k -> new ArrayList<>()).add(i);
            }
        }

        // 2. 初始化 CSR 数组
        int numGroups = groupNameToId.size();
        groupNodeOffsets = new int[numGroups];
        groupNodeCounts = new int[numGroups];

        int totalRefs = 0;
        for (int gId = 0; gId < numGroups; gId++) {
            totalRefs += tempGToList.getOrDefault(gId, new ArrayList<>()).size();
        }
        flatGroupNodes = new int[totalRefs];

        // 3. 连续填充
        int flatPtr = 0;
        for (int gId = 0; gId < numGroups; gId++) {
            List<Integer> list = tempGToList.getOrDefault(gId, new ArrayList<>());
            groupNodeOffsets[gId] = flatPtr;
            groupNodeCounts[gId] = list.size();
            for (int nodeId : list) {
                flatGroupNodes[flatPtr++] = nodeId;
            }
        }
    }

    /**
     * 由客户端 Binder 在计算出总顶点数后调用，一次性精准分配热数据缓冲
     */
    public void allocateSkinningBuffers(int totalVerts) {
        this.totalVertexCount = totalVerts;
        if (totalVerts == 0) return;

        this.vCenterNode = new int[totalVerts];
        this.vVxNode     = new int[totalVerts];
        this.vVyNode     = new int[totalVerts];
        this.vVzNode     = new int[totalVerts];

        this.vWeightX    = new float[totalVerts];
        this.vWeightY    = new float[totalVerts];
        this.vWeightZ    = new float[totalVerts];

        this.vNormWeightX = new float[totalVerts];
        this.vNormWeightY = new float[totalVerts];
        this.vNormWeightZ = new float[totalVerts];

        this.vUseCrossZ  = new boolean[totalVerts];

        this.skinnedPosX = new float[totalVerts];
        this.skinnedPosY = new float[totalVerts];
        this.skinnedPosZ = new float[totalVerts];

        this.normalX     = new float[totalVerts];
        this.normalY     = new float[totalVerts];
        this.normalZ     = new float[totalVerts];

        this.uvU         = new float[totalVerts];
        this.uvV         = new float[totalVerts];
    }
}