package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.HashMap;
import java.util.Map;

public class WheelContainer {
    public static final int INIT_WHEEL_CAP = 8;
    public static final int MAX_RAYS = 16; // 1D 数组的最大射线分配空间

    public int count = 0;
    public Map<String, Integer> nameToIndex = new HashMap<>();

    // 基础属性 SoA
    public String[] name = new String[INIT_WHEEL_CAP];
    public int[] node1 = new int[INIT_WHEEL_CAP];
    public int[] node2 = new int[INIT_WHEEL_CAP];
    public int[] numRays = new int[INIT_WHEEL_CAP];

    // 物理参数
    public double[] hubRadius = new double[INIT_WHEEL_CAP];
    public double[] tireRadius = new double[INIT_WHEEL_CAP];
    public double[] tireWidth = new double[INIT_WHEEL_CAP];
    public double[] frictionCoef = new double[INIT_WHEEL_CAP];
    public double[] pressurePSI = new double[INIT_WHEEL_CAP];

    // 🚀 一维展平数组：内存地址 100% 连续
    // 寻址方式： index = (wheelIndex * MAX_RAYS) + rayIndex
    public int[] hubInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] hubOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];

    // 储存三角形的index (必须确保它们在数组中连续排列)
    public int[] tireTriangleIdxStart = new int[INIT_WHEEL_CAP];
    public int[] tireTriangleIdxEnd = new int[INIT_WHEEL_CAP];

    private final SoftBodyVehicle vehicle;

    public WheelContainer(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    /**
     * 生成轮毂 (Hub)
     */
    public void generateHub(
            // --- 基础标识与连接 ---
            String wheelName, int n1, int n2, Integer nodeS, Integer nodeArm, int wheelDir, int rays,
            // --- 轮毂几何 ---
            double radius, double width, double offset,
            // --- 轮毂物理属性 ---
            double nodeWeight, double frictionCoef,
            // --- 轮毂梁参数 (前缀 Hub) ---
            double hubBeamSpring, double hubBeamDamp, double hubBeamDeform, double hubBeamStrength,
            double hubTreadSpring, double hubTreadDamp,
            double hubPeriphSpring, double hubPeriphDamp,
            double hubSideSpring, double hubSideDamp,
            double hubReinfSpring, double hubReinfDamp,
            // --- 碰撞与材质 ---
            boolean hubTriCollision, boolean hubSide1TriCollision, boolean hubSide2TriCollision,
            String hubNodeMaterial,
            // --- 轮毂盖参数 (带 Hubcap 前缀) ---
            boolean enableHubcaps, String hubcapBreakGroup, String hubcapGroup,
            boolean hubcapCollision, boolean hubcapSelfCollision, boolean enableExtraHubcapBeams,
            double hubcapOffset, double hubcapWidth, double hubcapRadius,
            double hubcapBeamSpring, double hubcapBeamDamp, double hubcapBeamDeform, double hubcapBeamStrength,
            double hubcapAttachSpring, double hubcapAttachDamp, double hubcapAttachDeform, double hubcapAttachStrength,
            double hubcapSupportDeform, double hubcapSupportStrength,
            double hubcapNodeWeight, double hubcapCenterWeight, String hubcapMaterial, double hubcapFriction,
            // --- 简化专用 ---
            double simpleRadius
    ) {
        ensureWheelCapacity();
        int wIdx = count;
        nameToIndex.put(wheelName, wIdx);

        name[wIdx] = wheelName;
        node1[wIdx] = n1;
        node2[wIdx] = n2;
        numRays[wIdx] = rays > 0 ? Math.min(rays, MAX_RAYS) : MAX_RAYS;
        hubRadius[wIdx] = radius;

        int partId = vehicle.nodes.partId[n1];
        int baseOffset = wIdx * MAX_RAYS;

        // 1. 基底向量计算
        double[] uX = {0}, uY = {0}, uZ = {0};
        double[] vX = {0}, vY = {0}, vZ = {0};
        double[] axisX = {0}, axisY = {0}, axisZ = {0};
        calculateWheelBasis(n1, n2, wheelDir, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        // 🚀 1. 算出 n1 和 n2 的物理中点
        double midX = (vehicle.nodes.posX[n1] + vehicle.nodes.posX[n2]) * 0.5;
        double midY = (vehicle.nodes.posY[n1] + vehicle.nodes.posY[n2]) * 0.5;
        double midZ = (vehicle.nodes.posZ[n1] + vehicle.nodes.posZ[n2]) * 0.5;

        // 🚀 2. 基于中点施加 Offset 偏距 (减号保持不变，因为 axisX 指向外侧，减去负 offset 刚好向外拓展)
        double centerX = midX - axisX[0] * offset;
        double centerY = midY - axisY[0] * offset;
        double centerZ = midZ - axisZ[0] * offset;

        // 2. 生成 Hub 节点
        for (int i = 0; i < rays; i++) {
            double angle = (2.0 * Math.PI * i) / rays;
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);

            double rayX = uX[0] * cosA + vX[0] * sinA;
            double rayY = uY[0] * cosA + vY[0] * sinA;
            double rayZ = uZ[0] * cosA + vZ[0] * sinA;

            // 内外圈各占 width 的一半
            double inX = centerX + rayX * radius - axisX[0] * (width * 0.5);
            double inY = centerY + rayY * radius - axisY[0] * (width * 0.5);
            double inZ = centerZ + rayZ * radius - axisZ[0] * (width * 0.5);

            double outX = centerX + rayX * radius + axisX[0] * (width * 0.5);
            double outY = centerY + rayY * radius + axisY[0] * (width * 0.5);
            double outZ = centerZ + rayZ * radius + axisZ[0] * (width * 0.5);

            // 生成物理节点
            vehicle.nodes.addNode(wheelName + "_hub_in_" + i, inX, inY, inZ, nodeWeight, frictionCoef, 0.0, partId, true, false);
            hubInnerNodes[baseOffset + i] = vehicle.nodes.count - 1;

            vehicle.nodes.addNode(wheelName + "_hub_out_" + i, outX, outY, outZ, nodeWeight, frictionCoef, 0.0, partId, true, false);
            hubOuterNodes[baseOffset + i] = vehicle.nodes.count - 1;
        }

        boolean COLLISION = false;

        // 3. 生成物理拓扑 (Beams)
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;
            int hInCur = hubInnerNodes[baseOffset + i], hInNext = hubInnerNodes[baseOffset + next];
            int hOutCur = hubOuterNodes[baseOffset + i], hOutNext = hubOuterNodes[baseOffset + next];

            // ================= 1. 轮辋蒙皮 =================
            // 周长支撑 (Tread)
            addFastBeam(hInCur, hInNext, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hOutCur, hOutNext, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);

            // 横向支撑与 X 型交叉防扭曲 (Periphery)
            //addFastBeam(hInCur, hOutCur, hubPeriS, hubPeriD, deform, hubBeamStrength); // 直连  <--直连和交叉只能二选一，不然会不稳定，根据观察，BeamNG只有交叉梁
            addFastBeam(hInCur, hOutNext, hubPeriphSpring, hubPeriphDamp, hubBeamDeform, hubBeamStrength); // 交叉 1
            addFastBeam(hOutCur, hInNext, hubPeriphSpring, hubPeriphDamp, hubBeamDeform, hubBeamStrength); // 交叉 2

            // ================= 2. 自行车交叉辐条 (Spokes) =================
            // a) 直连辐条 (内圈连内侧轴，外圈连外侧轴)
            addFastBeam(hOutCur, n1, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hInCur, n2, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);

            // b) 交叉辐条 (内圈连外侧轴，外圈连内侧轴)
            addFastBeam(hOutCur, n2, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hInCur, n1, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);

            // ================= 3. 稳定节点支撑 (nodeS) =================
            // 将轮毂内外圈所有节点都与 nodeS 相连，分摊 n2 的受力
            if (nodeS != null) {
                addFastBeam(hInCur, nodeS, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
                addFastBeam(hOutCur, nodeS, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
            }
        }

        count++;
    }

    /**
     * 生成轮胎 (Tire)
     */
    public void generateTire(
            // --- 基础标识与连接 ---
            String wheelName, int n1, int n2, int wheelDir, int rays,
            // --- 几何尺寸 ---
            double radius, double width, double offset,
            // --- 基本物理属性 ---
            double nodeWeight, double frictionCoef, double pressurePSI,
            // --- 轮胎专用摩擦/形变系数 ---
            double slidingFrictionCoef, double stribeckVelMult, double stribeckExponent,
            double treadCoef, double noLoadCoef, double loadSensitivitySlope, double fullLoadCoef,
            double softnessCoef, double maxPressurePSI,
            // --- 空气阻力 ---
            double dragCoef, double skinDragCoef,
            // --- 各向异性梁参数 (前缀 Tread / Periph / Side / Reinf) ---
            double treadSpring, double treadDamp, double treadDeform, double treadStrength,
            double periSpring, double periDamp, double periDeform, double periStrength,
            double sideSpring, double sideDamp, double sideSpringExp, double sideDampExp,
            double sideDeform, double sideStrength,
            double reinfSpring, double reinfDamp, double reinfDeform, double reinfStrength,
            double treadReinfSpring, double treadReinfDamp,
            double periReinfSpring, double periReinfDamp,
            double sideReinfSpring, double sideReinfDamp, double sideReinfSpringExp, double sideReinfDampExp,
            // --- 功能开关 (enableXxx) ---
            boolean enableTireLBeams, boolean enableTireReinfBeams, boolean enableTireSideReinfBeams,
            boolean enableTreadReinfBeams, boolean enableTirePeripheryReinfBeams, boolean enableTireSupportBeams,
            double supportBeamSpring, double supportBeamDamp,
            // --- 碰撞与材质 ---
            boolean triCollision, boolean treadTriCollision, boolean side1TriCollision, boolean side2TriCollision,
            String nodeMaterial,
            // --- 刹车参数 (全部带 brake 前缀) ---
            double brakeTorque, double parkingTorque, double brakeSpring,
            boolean enableBrakeThermals, double brakeDiameter, double brakeMass,
            String brakeType, String rotorMaterial, double brakeVentingCoef, String padMaterial,
            double brakeInputSplit, double brakeSplitCoef,
            double squealCoefNatural, double squealCoefLowSpeed, double squealCoefGlazing,
            boolean enableABS, double absSlipRatioTarget, double absHz,
            double brakePressureInDelay, double brakePressureOutDelay
    ) {

        if (!nameToIndex.containsKey(wheelName)) return;

        ensureWheelCapacity();

        int wIdx = nameToIndex.get(wheelName);
        int baseOffset = wIdx * MAX_RAYS;
        int partId = vehicle.nodes.partId[n1];

        tireRadius[wIdx] = radius;
        tireWidth[wIdx] = width;
        this.frictionCoef[wIdx] = frictionCoef;
        this.pressurePSI[wIdx] = pressurePSI;

        double[] uX = {0}, uY = {0}, uZ = {0};
        double[] vX = {0}, vY = {0}, vZ = {0};
        double[] axisX = {0}, axisY = {0}, axisZ = {0};
        calculateWheelBasis(n1, n2, wheelDir, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        // 🚀 1. 算出 n1 和 n2 的物理中点
        double midX = (vehicle.nodes.posX[n1] + vehicle.nodes.posX[n2]) * 0.5;
        double midY = (vehicle.nodes.posY[n1] + vehicle.nodes.posY[n2]) * 0.5;
        double midZ = (vehicle.nodes.posZ[n1] + vehicle.nodes.posZ[n2]) * 0.5;

        // 🚀 2. 基于中点施加 Offset 偏距 (减号保持不变，因为 axisX 指向外侧，减去负 offset 刚好向外拓展)
        double centerX = midX - axisX[0] * offset;
        double centerY = midY - axisY[0] * offset;
        double centerZ = midZ - axisZ[0] * offset;

        // 1. 生成轮胎外圈节点
        for (int i = 0; i < rays; i++) {
            double angle = (2.0 * Math.PI * i) / rays;
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);

            double rayX = uX[0] * cosA + vX[0] * sinA;
            double rayY = uY[0] * cosA + vY[0] * sinA;
            double rayZ = uZ[0] * cosA + vZ[0] * sinA;

            double inX = centerX + rayX * radius - axisX[0] * (width * 0.5);
            double inY = centerY + rayY * radius - axisY[0] * (width * 0.5);
            double inZ = centerZ + rayZ * radius - axisZ[0] * (width * 0.5);

            double outX = centerX + rayX * radius + axisX[0] * (width * 0.5);
            double outY = centerY + rayY * radius + axisY[0] * (width * 0.5);
            double outZ = centerZ + rayZ * radius + axisZ[0] * (width * 0.5);

            vehicle.nodes.addTireNode(wheelName + "_tire_in_" + i, inX, inY, inZ, nodeWeight,
                    frictionCoef, slidingFrictionCoef, partId, true, false, wIdx);
            tireInnerNodes[baseOffset + i] = vehicle.nodes.count - 1;

            vehicle.nodes.addTireNode(wheelName + "_tire_out_" + i, outX, outY, outZ, nodeWeight,
                    frictionCoef, slidingFrictionCoef, partId, true, false, wIdx);
            tireOuterNodes[baseOffset + i] = vehicle.nodes.count - 1;
        }

        boolean COLLISION = false;

        // 2. 缝合轮胎三角形与梁
        tireTriangleIdxStart[wIdx] = vehicle.triangles.count;
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;

            int hInCur = hubInnerNodes[baseOffset + i], hInNext = hubInnerNodes[baseOffset + next];
            int hOutCur = hubOuterNodes[baseOffset + i], hOutNext = hubOuterNodes[baseOffset + next];
            int tInCur = tireInnerNodes[baseOffset + i], tInNext = tireInnerNodes[baseOffset + next];
            int tOutCur = tireOuterNodes[baseOffset + i], tOutNext = tireOuterNodes[baseOffset + next];

            // 侧壁面：内侧环带 (Hub Inner -> Tire Inner)
            vehicle.triangles.addTriangle(hInCur, hInNext, tInNext, partId, COLLISION);
            vehicle.triangles.addTriangle(hInCur, tInNext, tInCur, partId, COLLISION);

            // 侧壁面：外侧环带 (Hub Outer -> Tire Outer)
            vehicle.triangles.addTriangle(hOutCur, tOutCur, tOutNext, partId, COLLISION);
            vehicle.triangles.addTriangle(hOutCur, tOutNext, hOutNext, partId, COLLISION);

            // 胎面 (Tire Inner -> Tire Outer)
            vehicle.triangles.addTriangle(tInCur, tInNext, tOutNext, partId, COLLISION);
            vehicle.triangles.addTriangle(tInCur, tOutNext, tOutCur, partId, COLLISION);

            // 轮胎与轮辋接触面（纯粹用于闭合散度体积，绝对关闭碰撞）
            vehicle.triangles.addTriangle(hInCur, hOutNext, hInNext, partId, false);
            vehicle.triangles.addTriangle(hInCur, hOutCur, hOutNext, partId, false);

            // 胎面 加强筋 (i 连 i+2)
            int next2 = (i + 2) % rays;
            int tInNext2 = tireInnerNodes[baseOffset + next2];
            int tOutNext2 = tireOuterNodes[baseOffset + next2];
            // ========================================================
            // 1. 圆周梁 (Periphery Beams) —— 维持周长，主导纵向抓地力
            // ========================================================
            // 普通圆周梁 (沿 i 连 i+1)
            addFastBeam(tInCur,  tInNext,  periSpring, periDamp, periDeform, periStrength);
            addFastBeam(tOutCur, tOutNext, periSpring, periDamp, periDeform, periStrength);

            // 圆周加强筋 (沿 i 连 i+2，文档中的 circumference +-2 nodes)
            if (enableTirePeripheryReinfBeams) {
                addFastBeam(tInCur,  tInNext2,  periReinfSpring, periReinfDamp, periDeform, periStrength);
                addFastBeam(tOutCur, tOutNext2, periReinfSpring, periReinfDamp, periDeform, periStrength);
            }

            // ========================================================
            // 2. 胎面横向梁 (Tread Beams) —— 跨宽度，主导过弯侧向支撑
            // ========================================================
            // 普通胎面横向支撑 (1根直连 + 2根交叉)
            addFastBeam(tInCur,  tOutCur,  treadSpring, treadDamp, treadDeform, treadStrength);
            addFastBeam(tInCur,  tOutNext, treadSpring, treadDamp, treadDeform, treadStrength);
            addFastBeam(tOutCur, tInNext,  treadSpring, treadDamp, treadDeform, treadStrength);

            // 胎面加强筋 (跨宽度 且 跨圆周的大交叉，文档中的 across +-2 nodes)
            if (enableTreadReinfBeams) {
                addFastBeam(tInCur,  tOutNext2, treadReinfSpring, treadReinfDamp, treadDeform, treadStrength);
                addFastBeam(tOutCur, tInNext2,  treadReinfSpring, treadReinfDamp, treadDeform, treadStrength);
            }

            // ========================================================
            // 3. 侧壁梁 (Sidewall Beams) —— 连 Hub 和 Tire，由气压主导
            // ========================================================
            // 普通侧壁支撑 (沿半径直连)
            addFastBeam(hInCur,  tInCur,  sideSpring, sideDamp, sideDeform, sideStrength);
            addFastBeam(hOutCur, tOutCur, sideSpring, sideDamp, sideDeform, sideStrength);

            // 侧壁加强筋 (侧壁交叉防扭曲，连目标环带的 i+2，文档中的 sidewall +-2 nodes)
            if (enableTireSideReinfBeams) {
                addFastBeam(hInCur,  tInNext2,  sideReinfSpring, sideReinfDamp, sideDeform, sideStrength);
                addFastBeam(hOutCur, tOutNext2, sideReinfSpring, sideReinfDamp, sideDeform, sideStrength);
            }

            // ========================================================
            // 4. 内部截面大支撑 (wheelReinfBeam)
            // ========================================================
            // 穿过空气腔，连接内侧 Hub 和 外侧 Tire，防止轮胎截面横向塌陷
            if (enableTireReinfBeams) {
                addFastBeam(hInCur,  tOutNext, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
                addFastBeam(hOutCur, tInNext, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
            }

            if (enableTireLBeams) {
                // 交叉对角线 1：共享点 tIn，连接 hIn 和 tOut
                addFastLBeam(hInCur, tOutCur, tInCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);

                // 交叉对角线 2：共享点 tOut，连接 hOut 和 tIn
                addFastLBeam(hOutCur, tInCur, tOutCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
            }

            // ========================================================
            // 5. 防瘪兜底梁 (Tire Support Beams) —— 仅做物理限位缓冲
            // ========================================================
            if (enableTireSupportBeams) {
                // 这里的梁应当存入 supportBeams 容器，并且设置 beamPrecompression（如 0.85）
                // 使得它们平时处于松弛状态，只有当轮胎快要彻底压死碰壁时才提供极强的推力
                // vehicle.supportBeams.addBeam(...);
            }
        }
        tireTriangleIdxEnd[wIdx] = vehicle.triangles.count - 1;
    }

    private void addFastBeam(int id1, int id2, double spring, double damp, double deform, double strength) {
        double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        vehicle.normalBeams.addBeam(id1, id2, dist, spring, damp, deform, strength, 1.0, 0.0, 0.0);
    }

    private void addFastLBeam(int id1, int id2, int id3, double spring, double damp, double deform, double strength) {
        double dx;
        double dy;
        double dz;
        dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist12 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        dx = vehicle.nodes.posX[id3] - vehicle.nodes.posX[id1];
        dy = vehicle.nodes.posY[id3] - vehicle.nodes.posY[id1];
        dz = vehicle.nodes.posZ[id3] - vehicle.nodes.posZ[id1];
        double dist13 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        dx = vehicle.nodes.posX[id3] - vehicle.nodes.posX[id2];
        dy = vehicle.nodes.posY[id3] - vehicle.nodes.posY[id2];
        dz = vehicle.nodes.posZ[id3] - vehicle.nodes.posZ[id2];
        double dist23 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        vehicle.lBeams.addBeam(id1, id2, id3, dist12, dist13, dist23, spring, damp, deform, strength, 1.0, 0.0, 0.0);
    }

    private void calculateWheelBasis(int n1, int n2, int wheelDir, double[] ax, double[] ay, double[] az, double[] ux, double[] uy, double[] uz, double[] vx, double[] vy, double[] vz) {
        double n1x = vehicle.nodes.posX[n1], n1y = vehicle.nodes.posY[n1], n1z = vehicle.nodes.posZ[n1];
        double n2x = vehicle.nodes.posX[n2], n2y = vehicle.nodes.posY[n2], n2z = vehicle.nodes.posZ[n2];

        // n1 永远是外侧，n2 是内侧。因此 n1 - n2 永远指向车外
        ax[0] = n1x - n2x; ay[0] = n1y - n2y; az[0] = n1z - n2z;
        double len = Math.sqrt(ax[0]*ax[0] + ay[0]*ay[0] + az[0]*az[0]);
        if (len > 0) { ax[0]/=len; ay[0]/=len; az[0]/=len; }

        if (Math.abs(ay[0]) > 0.9) { ux[0] = 1; uy[0] = 0; uz[0] = 0; }
        else { ux[0] = 0; uy[0] = 1; uz[0] = 0; }

        vx[0] = ay[0] * uz[0] - az[0] * uy[0];
        vy[0] = az[0] * ux[0] - ax[0] * uz[0];
        vz[0] = ax[0] * uy[0] - ay[0] * ux[0];
        double vLen = Math.sqrt(vx[0]*vx[0] + vy[0]*vy[0] + vz[0]*vz[0]);
        if (vLen > 0) { vx[0]/=vLen; vy[0]/=vLen; vz[0]/=vLen; }

        ux[0] = vy[0] * az[0] - vz[0] * ay[0];
        uy[0] = vz[0] * ax[0] - vx[0] * az[0];
        uz[0] = vx[0] * ay[0] - vy[0] * ax[0];

        if (wheelDir < 0) {
            ux[0] = -ux[0]; uy[0] = -uy[0]; uz[0] = -uz[0];
        }
    }

    private void ensureWheelCapacity() {
        if (count >= name.length) {
            int newSize = name.length * 2;
            // 扩容普通动态数组
            name = Utility.expand(name, newSize);
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            numRays = Utility.expand(numRays, newSize);
            hubRadius = Utility.expand(hubRadius, newSize);
            tireRadius = Utility.expand(tireRadius, newSize);
            tireWidth = Utility.expand(tireWidth, newSize);
            frictionCoef = Utility.expand(frictionCoef, newSize);
            pressurePSI = Utility.expand(pressurePSI, newSize);

            tireTriangleIdxStart = Utility.expand(tireTriangleIdxStart, newSize);
            tireTriangleIdxEnd = Utility.expand(tireTriangleIdxEnd, newSize);

            // 扩容展平数组（每个车轮 MAX_RAYS 个射线槽位）
            int newFlatSize = newSize * MAX_RAYS;
            hubInnerNodes = Utility.expand(hubInnerNodes, newFlatSize);
            hubOuterNodes = Utility.expand(hubOuterNodes, newFlatSize);
            tireInnerNodes = Utility.expand(tireInnerNodes, newFlatSize);
            tireOuterNodes = Utility.expand(tireOuterNodes, newFlatSize);

            System.out.println("⚠️ [WheelContainer] Resized to: " + newSize + " wheels, flat size: " + newFlatSize);
        }
    }
}