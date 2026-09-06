package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.HashMap;
import java.util.List;
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
    public int[] wheelDir = new int[INIT_WHEEL_CAP];
    public int[] numRays = new int[INIT_WHEEL_CAP];

    // 物理参数
    public float[] hubRadius = new float[INIT_WHEEL_CAP];
    public float[] tireRadius = new float[INIT_WHEEL_CAP];
    public float[] tireWidth = new float[INIT_WHEEL_CAP];
    public float[] pressurePSI = new float[INIT_WHEEL_CAP];

    // Service-brake configuration and per-wheel pressure state.
    public float[] brakeTorque = new float[INIT_WHEEL_CAP];
    public float[] parkingTorque = new float[INIT_WHEEL_CAP];
    public float[] brakeSpring = new float[INIT_WHEEL_CAP];
    public float[] brakeInputSplit = new float[INIT_WHEEL_CAP];
    public float[] brakeSplitCoef = new float[INIT_WHEEL_CAP];
    public float[] brakePressureInDelay = new float[INIT_WHEEL_CAP];
    public float[] brakePressureOutDelay = new float[INIT_WHEEL_CAP];
    public float[] serviceBrakeTorque = new float[INIT_WHEEL_CAP];
    public float[] brakeAngle = new float[INIT_WHEEL_CAP];

    // 轮胎节点摩擦参数
    public float[] frictionCoef         = new float[INIT_WHEEL_CAP];
    public float[] slidingFrictionCoef  = new float[INIT_WHEEL_CAP];
    public float[] stribeckVelMult      = new float[INIT_WHEEL_CAP];
    public float[] stribeckExponent     = new float[INIT_WHEEL_CAP];
    public float[] treadCoef            = new float[INIT_WHEEL_CAP];
    public float[] noLoadCoef           = new float[INIT_WHEEL_CAP];
    public float[] loadSensitivitySlope = new float[INIT_WHEEL_CAP];
    public float[] fullLoadCoef         = new float[INIT_WHEEL_CAP];
    public float[] softnessCoef         = new float[INIT_WHEEL_CAP];

    // 🚀 一维展平数组：内存地址 100% 连续
    // 寻址方式： index = (wheelIndex * MAX_RAYS) + rayIndex
    public int[] hubInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] hubOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];

    // 储存三角形的index (必须确保它们在数组中连续排列)
    public int[] tireTriangleIdxStart = new int[INIT_WHEEL_CAP];
    public int[] tireTriangleIdxEnd = new int[INIT_WHEEL_CAP];

    public float[] initialVolume = new float[INIT_WHEEL_CAP];

    public float[] prevVolume = new float[INIT_WHEEL_CAP];
    public float[] normalSign = new float[INIT_WHEEL_CAP];

    public boolean[] isDeflated = new boolean[INIT_WHEEL_CAP];

    private final SoftBodyVehicle vehicle;

    public WheelContainer(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    /**
     * 生成轮毂 (Hub)
     */
    public void generateHub(PhysicsSpecs.WheelHubSpec spec) {
        buildHub(spec);
    }

    private void buildHub(PhysicsSpecs.WheelHubSpec spec) {
        String wheelName = spec.wheelName();
        int n1 = spec.node1();
        int n2 = spec.node2();
        Integer nodeS = spec.nodeS();
        int wheelDir = spec.wheelDir();
        int rays = spec.rays();
        double radius = spec.radius();
        double width = spec.width();
        double offset = spec.offset();
        double nodeWeight = spec.nodeWeight();
        double frictionCoef = spec.frictionCoef();
        double hubBeamDeform = spec.hubBeamDeform();
        double hubBeamStrength = spec.hubBeamStrength();
        double hubTreadSpring = spec.hubTreadSpring();
        double hubTreadDamp = spec.hubTreadDamp();
        double hubPeriphSpring = spec.hubPeriphSpring();
        double hubPeriphDamp = spec.hubPeriphDamp();
        double hubSideSpring = spec.hubSideSpring();
        double hubSideDamp = spec.hubSideDamp();
        String hubGroup = spec.hubGroup();
        ensureWheelCapacity();
        int wIdx = count;
        nameToIndex.put(wheelName, wIdx);

        name[wIdx] = wheelName;
        node1[wIdx] = n1;
        node2[wIdx] = n2;
        this.wheelDir[wIdx] = wheelDir >= 0 ? 1 : -1;
        numRays[wIdx] = rays > 0 ? Math.min(rays, MAX_RAYS) : MAX_RAYS;
        hubRadius[wIdx] = (float) radius;
        this.isDeflated[wIdx] = false;

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
            hubInnerNodes[baseOffset + i] = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_hub_in_" + i, (float) inX, (float) inY, (float) inZ, (float) nodeWeight,
                    (float) frictionCoef, 0.0f, partId,
                    true, false, List.of(hubGroup)));

            hubOuterNodes[baseOffset + i]  = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_hub_out_" + i, (float) outX, (float) outY, (float) outZ, (float) nodeWeight,
                    (float) frictionCoef, 0.0f, partId,
                    true, false, List.of(hubGroup)));
        }

        // 3. 生成物理拓扑 (Beams)
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;
            int hInCur = hubInnerNodes[baseOffset + i], hInNext = hubInnerNodes[baseOffset + next];
            int hOutCur = hubOuterNodes[baseOffset + i], hOutNext = hubOuterNodes[baseOffset + next];

            // ================= 1. 轮辋蒙皮 =================
            // 周长支撑 (Tread)
            int treadInIdx = addFastBeam(hInCur, hInNext, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);
            int treadOutIdx = addFastBeam(hOutCur, hOutNext, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);
            vehicle.normalBeams.bindToTire(treadInIdx, wIdx);
            vehicle.normalBeams.bindToTire(treadOutIdx, wIdx);

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
    public void generateTire(PhysicsSpecs.WheelTireSpec spec) {
        buildTire(spec);
    }

    private void buildTire(PhysicsSpecs.WheelTireSpec spec) {
        String wheelName = spec.wheelName();
        int n1 = spec.node1();
        int n2 = spec.node2();
        int wheelDir = spec.wheelDir();
        int rays = spec.rays();
        double radius = spec.radius();
        double width = spec.width();
        double offset = spec.offset();
        double nodeWeight = spec.nodeWeight();
        double frictionCoef = spec.frictionCoef();
        double pressurePSI = spec.pressurePSI();
        double slidingFrictionCoef = spec.slidingFrictionCoef();
        double stribeckVelMult = spec.stribeckVelMult();
        double stribeckExponent = spec.stribeckExponent();
        double treadCoef = spec.treadCoef();
        double noLoadCoef = spec.noLoadCoef();
        double loadSensitivitySlope = spec.loadSensitivitySlope();
        double fullLoadCoef = spec.fullLoadCoef();
        double softnessCoef = spec.softnessCoef();
        double treadSpring = spec.treadSpring();
        double treadDamp = spec.treadDamp();
        double treadDeform = spec.treadDeform();
        double treadStrength = spec.treadStrength();
        double periSpring = spec.periSpring();
        double periDamp = spec.periDamp();
        double periDeform = spec.periDeform();
        double periStrength = spec.periStrength();
        double sideSpring = spec.sideSpring();
        double sideDamp = spec.sideDamp();
        double sideSpringExp = spec.sideSpringExp();
        double sideDampExp = spec.sideDampExp();
        double sideTransZone = spec.sideTransZone();
        double sideDeform = spec.sideDeform();
        double sideStrength = spec.sideStrength();
        double reinfSpring = spec.reinfSpring();
        double reinfDamp = spec.reinfDamp();
        double reinfDeform = spec.reinfDeform();
        double reinfStrength = spec.reinfStrength();
        double treadReinfSpring = spec.treadReinfSpring();
        double treadReinfDamp = spec.treadReinfDamp();
        double periReinfSpring = spec.periReinfSpring();
        double periReinfDamp = spec.periReinfDamp();
        double sideReinfSpring = spec.sideReinfSpring();
        double sideReinfDamp = spec.sideReinfDamp();
        double sideReinfSpringExp = spec.sideReinfSpringExp();
        double sideReinfDampExp = spec.sideReinfDampExp();
        boolean enableTireLBeams = spec.enableTireLBeams();
        boolean enableTireReinfBeams = spec.enableTireReinfBeams();
        boolean enableTireSideReinfBeams = spec.enableTireSideReinfBeams();
        boolean enableTreadReinfBeams = spec.enableTreadReinfBeams();
        boolean enableTirePeripheryReinfBeams = spec.enableTirePeripheryReinfBeams();
        boolean enableTireSupportBeams = spec.enableTireSupportBeams();
        String group = spec.group();

        if (!nameToIndex.containsKey(wheelName)) return;

        ensureWheelCapacity();

        int wIdx = nameToIndex.get(wheelName);
        int baseOffset = wIdx * MAX_RAYS;
        int partId = vehicle.nodes.partId[n1];

        tireRadius[wIdx] = (float) radius;
        tireWidth[wIdx] = (float) width;
        this.pressurePSI[wIdx] = (float) pressurePSI;
        this.isDeflated[wIdx] = false;

        this.frictionCoef[wIdx] = (float) frictionCoef;
        this.slidingFrictionCoef[wIdx] = (float) slidingFrictionCoef;
        this.stribeckVelMult[wIdx] = (float) stribeckVelMult;
        this.stribeckExponent[wIdx] = (float) stribeckExponent;
        this.treadCoef[wIdx] = (float) treadCoef;
        this.noLoadCoef[wIdx] = (float) noLoadCoef;
        this.loadSensitivitySlope[wIdx] = (float) loadSensitivitySlope;
        this.fullLoadCoef[wIdx] = (float) fullLoadCoef;
        this.softnessCoef[wIdx] = (float) softnessCoef;
        this.brakeTorque[wIdx] = Math.max(0.0f, (float) spec.brakeTorque());
        this.parkingTorque[wIdx] = Math.max(0.0f, (float) spec.parkingTorque());
        this.brakeSpring[wIdx] = Math.max(0.0f, (float) spec.brakeSpring());
        this.brakeInputSplit[wIdx] = Math.clamp((float) spec.brakeInputSplit(), 0.0f, 1.0f);
        this.brakeSplitCoef[wIdx] = Math.clamp((float) spec.brakeSplitCoef(), 0.0f, 1.0f);
        this.brakePressureInDelay[wIdx] = Math.max(0.0f, (float) spec.brakePressureInDelay());
        this.brakePressureOutDelay[wIdx] = Math.max(0.0f, (float) spec.brakePressureOutDelay());
        this.serviceBrakeTorque[wIdx] = 0.0f;
        this.brakeAngle[wIdx] = 0.0f;

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

            int idxIn = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_tire_in_" + i, (float) inX, (float) inY, (float) inZ, (float) nodeWeight,
                    (float) frictionCoef, (float) slidingFrictionCoef, partId,
                    true, false, List.of(group)));
            tireInnerNodes[baseOffset + i] = idxIn;
            vehicle.nodes.bindToTire(idxIn, wIdx);

            int idxOut = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_tire_out_" + i, (float) outX, (float) outY, (float) outZ, (float) nodeWeight,
                    (float) frictionCoef, (float) slidingFrictionCoef, partId,
                    true, false, List.of(group)));
            tireOuterNodes[baseOffset + i] = idxOut;
            vehicle.nodes.bindToTire(idxOut, wIdx);
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
            addTriangle(hInCur, hInNext, tInNext, partId, COLLISION);
            addTriangle(hInCur, tInNext, tInCur, partId, COLLISION);

            // 侧壁面：外侧环带 (Hub Outer -> Tire Outer)
            addTriangle(hOutCur, tOutCur, tOutNext, partId, COLLISION);
            addTriangle(hOutCur, tOutNext, hOutNext, partId, COLLISION);

            // 胎面 (Tire Inner -> Tire Outer)
            addTriangle(tInCur, tInNext, tOutNext, partId, COLLISION);
            addTriangle(tInCur, tOutNext, tOutCur, partId, COLLISION);

            // 轮胎与轮辋接触面（纯粹用于闭合散度体积，绝对关闭碰撞）
            addTriangle(hInCur, hOutNext, hInNext, partId, false);
            addTriangle(hInCur, hOutCur, hOutNext, partId, false);

            // 胎面 加强筋 (i 连 i+2)
            int next2 = (i + 2) % rays;
            int tInNext2 = tireInnerNodes[baseOffset + next2];
            int tOutNext2 = tireOuterNodes[baseOffset + next2];
            // ========================================================
            // 1. 圆周梁 (Periphery Beams) —— 维持周长，主导纵向抓地力
            // ========================================================
            // 普通圆周梁 (沿 i 连 i+1)
            int periInIdx = addFastBeam(tInCur,  tInNext,  periSpring, periDamp, periDeform, periStrength);
            int periOutIdx = addFastBeam(tOutCur, tOutNext, periSpring, periDamp, periDeform, periStrength);
            vehicle.normalBeams.bindToTire(periInIdx, wIdx);
            vehicle.normalBeams.bindToTire(periOutIdx, wIdx);

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
            int sideInIdx = addFastAnisotropicBeam(hInCur,  tInCur,  sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);
            int sideOutIdx = addFastAnisotropicBeam(hOutCur, tOutCur, sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);
            vehicle.anisotropicBeams.bindToTire(sideInIdx, wIdx);
            vehicle.anisotropicBeams.bindToTire(sideOutIdx, wIdx);

            // 侧壁加强筋 (侧壁交叉防扭曲，连目标环带的 i+2，文档中的 sidewall +-2 nodes)
            if (enableTireSideReinfBeams) {
                addFastAnisotropicBeam(hInCur,  tInNext2,  sideReinfSpring, sideReinfDamp, sideDeform, sideStrength,
                        sideReinfSpringExp, sideReinfDampExp, sideTransZone);
                addFastAnisotropicBeam(hOutCur, tOutNext2, sideReinfSpring, sideReinfDamp, sideDeform, sideStrength,
                        sideReinfSpringExp, sideReinfDampExp, sideTransZone);
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
                // TODO: 优先级不高
                // 这里的梁应当存入 supportBeams 容器，并且设置 beamPrecompression（如 0.85）
                // 使得它们平时处于松弛状态，只有当轮胎快要彻底压死碰壁时才提供极强的推力
                // vehicle.supportBeams.addBeam(...);
            }
        }
        tireTriangleIdxEnd[wIdx] = vehicle.triangles.count - 1;

        // 废弃圆柱公式，使用离散网格精准求积，保证初始内外压强比绝对为 1.0
        double volSum = 0.0;
        for (int i = tireTriangleIdxStart[wIdx]; i <= tireTriangleIdxEnd[wIdx]; i++) {
            int nA = vehicle.triangles.node1[i];
            int nB = vehicle.triangles.node2[i];
            int nC = vehicle.triangles.node3[i];

            double ax = vehicle.nodes.posX[nA], ay = vehicle.nodes.posY[nA], az = vehicle.nodes.posZ[nA];
            double bx = vehicle.nodes.posX[nB], by = vehicle.nodes.posY[nB], bz = vehicle.nodes.posZ[nB];
            double cx = vehicle.nodes.posX[nC], cy = vehicle.nodes.posY[nC], cz = vehicle.nodes.posZ[nC];

            double crossX = by * cz - bz * cy;
            double crossY = bz * cx - bx * cz;
            double crossZ = bx * cy - by * cx;

            volSum += (ax * crossX + ay * crossY + az * crossZ);
        }
        // 记录绝对静止体积
        initialVolume[wIdx] = (float) Math.abs(volSum / 6.0);

        // 不要忘记初始化！！！
        prevVolume[wIdx] = initialVolume[wIdx];
        normalSign[wIdx] = (volSum < 0.0) ? -1.0f : 1.0f;
    }

    private int addFastBeam(int id1, int id2, double spring, double damp, double deform, double strength) {
        double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return vehicle.normalBeams.addBeam(
                beamSpec(BeamContainer.BEAM_NORMAL, spring, damp, deform, strength),
                id1, id2, (float) dist
        );
    }

    private int addFastLBeam(int id1, int id2, int id3, double spring, double damp, double deform, double strength) {
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
        return vehicle.lBeams.addBeam(
                beamSpec(BeamContainer.BEAM_LBEAM, spring, damp, deform, strength),
                id1, id2, id3, (float) dist12, (float) dist13, (float) dist23
        );
    }

    private int addFastAnisotropicBeam(int id1, int id2, double spring, double damp, double deform, double strength, double springExp, double dampExp, double transitionZone) {
        double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return vehicle.anisotropicBeams.addBeam(
                beamSpec(BeamContainer.BEAM_ANISOTROPIC, spring, damp, deform, strength,
                        springExp, dampExp, transitionZone),
                id1, id2, (float) dist
        );
    }

    private PhysicsSpecs.BeamSpec beamSpec(int type, double spring, double damp, double deform, double strength) {
        return beamSpec(type, spring, damp, deform, strength, spring, damp, 0.0);
    }

    private PhysicsSpecs.BeamSpec beamSpec(int type, double spring, double damp, double deform, double strength,
                                           double springExpansion, double dampExpansion, double transitionZone) {
        return new PhysicsSpecs.BeamSpec(
                type, null, null, null,
                null, 0, false,
                (float) spring, (float) damp, (float) deform, (float) strength,
                1.0f, 0.0f, 0.0f,
                1.0f, 1.0f, -1.0f, -1.0f,
                (float) spring, (float) damp,
                -1.0f, -1.0f, -1.0f, -1.0f,
                (float) springExpansion, (float) dampExpansion, (float) transitionZone,
                PhysicsWorld.KINDA_BIG_NUMBER
        );
    }

    private void addTriangle(int n1, int n2, int n3, int partId, boolean collision) {
        vehicle.triangles.addTriangle(
                new PhysicsSpecs.TriangleSpec(null, null, null, List.of(), partId, collision),
                n1, n2, n3);
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
    }

    private void ensureWheelCapacity() {
        if (count >= name.length) {
            int newSize = name.length * 2;
            // 扩容普通动态数组
            name = Utility.expand(name, newSize);
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            wheelDir = Utility.expand(wheelDir, newSize);
            numRays = Utility.expand(numRays, newSize);
            hubRadius = Utility.expand(hubRadius, newSize);
            tireRadius = Utility.expand(tireRadius, newSize);
            tireWidth = Utility.expand(tireWidth, newSize);
            pressurePSI = Utility.expand(pressurePSI, newSize);
            brakeTorque = Utility.expand(brakeTorque, newSize);
            parkingTorque = Utility.expand(parkingTorque, newSize);
            brakeSpring = Utility.expand(brakeSpring, newSize);
            brakeInputSplit = Utility.expand(brakeInputSplit, newSize);
            brakeSplitCoef = Utility.expand(brakeSplitCoef, newSize);
            brakePressureInDelay = Utility.expand(brakePressureInDelay, newSize);
            brakePressureOutDelay = Utility.expand(brakePressureOutDelay, newSize);
            serviceBrakeTorque = Utility.expand(serviceBrakeTorque, newSize);
            brakeAngle = Utility.expand(brakeAngle, newSize);

            frictionCoef = Utility.expand(frictionCoef, newSize);
            slidingFrictionCoef = Utility.expand(slidingFrictionCoef, newSize);
            stribeckVelMult = Utility.expand(stribeckVelMult, newSize);
            stribeckExponent = Utility.expand(stribeckExponent, newSize);
            treadCoef = Utility.expand(treadCoef, newSize);
            noLoadCoef = Utility.expand(noLoadCoef, newSize);
            loadSensitivitySlope = Utility.expand(loadSensitivitySlope, newSize);
            fullLoadCoef = Utility.expand(fullLoadCoef, newSize);
            softnessCoef = Utility.expand(softnessCoef, newSize);

            tireTriangleIdxStart = Utility.expand(tireTriangleIdxStart, newSize);
            tireTriangleIdxEnd = Utility.expand(tireTriangleIdxEnd, newSize);

            initialVolume = Utility.expand(initialVolume, newSize);
            prevVolume = Utility.expand(prevVolume, newSize);
            normalSign = Utility.expand(normalSign, newSize);

            // 扩容展平数组（每个车轮 MAX_RAYS 个射线槽位）
            int newFlatSize = newSize * MAX_RAYS;
            hubInnerNodes = Utility.expand(hubInnerNodes, newFlatSize);
            hubOuterNodes = Utility.expand(hubOuterNodes, newFlatSize);
            tireInnerNodes = Utility.expand(tireInnerNodes, newFlatSize);
            tireOuterNodes = Utility.expand(tireOuterNodes, newFlatSize);

            isDeflated = Utility.expand(isDeflated, newFlatSize);

            System.out.println("⚠️ [WheelContainer] Resized to: " + newSize + " wheels, flat size: " + newFlatSize);
        }
    }

    public void deflateWheel(int idx) {
        if (idx >= 0 && idx < count) {
            if (!isDeflated[idx])isDeflated[idx] = true;
        }
    }

    /**
     * Returns the angular velocity of the soft-body wheel hub about its current
     * axle.  The sign is normalized with the JBeam wheelDir value, so wheels on
     * opposite sides of the vehicle report the same sign while rolling forward.
     */
    public float getAngularVelocity(int wheelIdx) {
        if (wheelIdx < 0 || wheelIdx >= count) return 0.0f;

        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return 0.0f;

        // Expose the same forward-positive convention as the powertrain. The old
        // node1-node2 axis made a forward-rolling wheel report a negative AV.
        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return 0.0f;
        double direction = wheelDir[wheelIdx] >= 0 ? 1.0 : -1.0;
        ax = ax * direction / axisLength;
        ay = ay * direction / axisLength;
        az = az * direction / axisLength;

        double totalMass = 0.0;
        double cx = 0.0, cy = 0.0, cz = 0.0;
        double cvx = 0.0, cvy = 0.0, cvz = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            double innerMass = Math.max(0.0, nodes.mass[inner]);
            double outerMass = Math.max(0.0, nodes.mass[outer]);
            totalMass += innerMass + outerMass;
            cx += nodes.posX[inner] * innerMass + nodes.posX[outer] * outerMass;
            cy += nodes.posY[inner] * innerMass + nodes.posY[outer] * outerMass;
            cz += nodes.posZ[inner] * innerMass + nodes.posZ[outer] * outerMass;
            cvx += nodes.velX[inner] * innerMass + nodes.velX[outer] * outerMass;
            cvy += nodes.velY[inner] * innerMass + nodes.velY[outer] * outerMass;
            cvz += nodes.velZ[inner] * innerMass + nodes.velZ[outer] * outerMass;
        }
        if (totalMass < 1e-9) return 0.0f;
        cx /= totalMass; cy /= totalMass; cz /= totalMass;
        cvx /= totalMass; cvy /= totalMass; cvz /= totalMass;

        double angularMomentum = 0.0;
        double inertia = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            angularMomentum += angularMomentum(nodes, inner, cx, cy, cz, cvx, cvy, cvz, ax, ay, az);
            angularMomentum += angularMomentum(nodes, outer, cx, cy, cz, cvx, cvy, cvz, ax, ay, az);
            inertia += polarInertia(nodes, inner, cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, outer, cx, cy, cz, ax, ay, az);
        }
        return inertia > 1e-9 ? (float) (angularMomentum / inertia) : 0.0f;
    }

    /** Returns the hub and tire nodes' instantaneous polar inertia. */
    public float getRotationalInertia(int wheelIdx) {
        if (wheelIdx < 0 || wheelIdx >= count) return 0.0f;
        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return 0.0f;

        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return 0.0f;
        ax /= axisLength; ay /= axisLength; az /= axisLength;

        double cx = (nodes.posX[node1[wheelIdx]] + nodes.posX[node2[wheelIdx]]) * 0.5;
        double cy = (nodes.posY[node1[wheelIdx]] + nodes.posY[node2[wheelIdx]]) * 0.5;
        double cz = (nodes.posZ[node1[wheelIdx]] + nodes.posZ[node2[wheelIdx]]) * 0.5;
        double inertia = 0.0;
        boolean hasTire = tireRadius[wheelIdx] > 0.0f;
        for (int ray = 0; ray < rays; ray++) {
            inertia += polarInertia(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
            if (hasTire) {
                inertia += polarInertia(nodes, tireInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
                inertia += polarInertia(nodes, tireOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
            }
        }
        return (float) inertia;
    }

    /** Applies a pure axle torque to the hub ring without adding net force. */
    public void applyDriveTorque(int wheelIdx, float torque) {
        if (wheelIdx < 0 || wheelIdx >= count || Math.abs(torque) < 1e-8f) return;
        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return;

        // Keep applied torque and reported AV on the same forward-positive axis.
        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return;
        double direction = wheelDir[wheelIdx] >= 0 ? 1.0 : -1.0;
        ax = ax * direction / axisLength;
        ay = ay * direction / axisLength;
        az = az * direction / axisLength;

        double totalMass = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            double innerMass = Math.max(0.0, nodes.mass[inner]);
            double outerMass = Math.max(0.0, nodes.mass[outer]);
            totalMass += innerMass + outerMass;
            cx += nodes.posX[inner] * innerMass + nodes.posX[outer] * outerMass;
            cy += nodes.posY[inner] * innerMass + nodes.posY[outer] * outerMass;
            cz += nodes.posZ[inner] * innerMass + nodes.posZ[outer] * outerMass;
        }
        if (totalMass < 1e-9) return;
        cx /= totalMass; cy /= totalMass; cz /= totalMass;

        double inertia = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            inertia += polarInertia(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
        }
        if (inertia < 1e-9) return;
        double angularAcceleration = torque / inertia;
        for (int ray = 0; ray < rays; ray++) {
            applyAngularForce(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az, angularAcceleration);
            applyAngularForce(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az, angularAcceleration);
        }
    }

    /** Applies the JBeam service brake curve and pressure delays to every wheel. */
    public void applyServiceBrakes(float brakeInput, float dt) {
        applyBrakes(brakeInput, 0.0f, dt);
    }

    /** Applies service and parking-brake inputs; parking input is intentionally unbound for now. */
    public void applyBrakes(float brakeInput, float parkingBrakeInput, float dt) {
        if (dt <= 0.0f) return;
        float input = Math.clamp(brakeInput, 0.0f, 1.0f);
        float parkingInput = Math.clamp(parkingBrakeInput, 0.0f, 1.0f);
        for (int wheel = 0; wheel < count; wheel++) {
            float maximum = brakeTorque[wheel];
            float target = calculateServiceBrakeTorque(maximum, input,
                    brakeInputSplit[wheel], brakeSplitCoef[wheel]);
            float delay = target > serviceBrakeTorque[wheel]
                    ? brakePressureInDelay[wheel] : brakePressureOutDelay[wheel];
            float rate = delay > 1.0e-6f ? maximum / delay : Float.POSITIVE_INFINITY;
            serviceBrakeTorque[wheel] = moveTowards(serviceBrakeTorque[wheel], target, rate * dt);

            float capacity = Math.max(serviceBrakeTorque[wheel], parkingTorque[wheel] * parkingInput);
            float stiffness = Math.max(Math.max(brakeTorque[wheel], parkingTorque[wheel]), 1.0f)
                    * brakeSpring[wheel];
            if (capacity <= 1.0e-8f || stiffness <= 1.0e-8f) {
                brakeAngle[wheel] = 0.0f;
                continue;
            }

            float angularVelocity = getAngularVelocity(wheel);
            if (brakeAngle[wheel] * angularVelocity < 0.0f) {
                brakeAngle[wheel] = 0.0f;
            }
            float angleLimit = capacity / stiffness;
            brakeAngle[wheel] = Math.clamp(
                    brakeAngle[wheel] + angularVelocity * dt,
                    -angleLimit,
                    angleLimit);

            float compliantTorque = Math.abs(brakeAngle[wheel]) * stiffness;
            float stoppingTorque = Math.abs(angularVelocity) * getHubRotationalInertia(wheel) / dt;
            float appliedTorque = Math.min(compliantTorque, stoppingTorque);
            applyDriveTorque(wheel, -Math.copySign(appliedTorque, angularVelocity));
        }
    }

    static float calculateServiceBrakeTorque(float maximum, float input, float split, float splitCoef) {
        float clampedInput = Math.clamp(input, 0.0f, 1.0f);
        float clampedSplit = Math.clamp(split, 0.0f, 1.0f);
        float clampedCoef = Math.clamp(splitCoef, 0.0f, 1.0f);
        return Math.max(0.0f, maximum) * (Math.min(clampedInput, clampedSplit)
                + Math.max(clampedInput - clampedSplit, 0.0f) * clampedCoef);
    }

    private float getHubRotationalInertia(int wheelIdx) {
        if (wheelIdx < 0 || wheelIdx >= count) return 0.0f;
        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return 0.0f;

        double ax = nodes.posX[node1[wheelIdx]] - nodes.posX[node2[wheelIdx]];
        double ay = nodes.posY[node1[wheelIdx]] - nodes.posY[node2[wheelIdx]];
        double az = nodes.posZ[node1[wheelIdx]] - nodes.posZ[node2[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return 0.0f;
        ax /= axisLength;
        ay /= axisLength;
        az /= axisLength;

        double totalMass = 0.0;
        double cx = 0.0, cy = 0.0, cz = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            double innerMass = Math.max(0.0, nodes.mass[inner]);
            double outerMass = Math.max(0.0, nodes.mass[outer]);
            totalMass += innerMass + outerMass;
            cx += nodes.posX[inner] * innerMass + nodes.posX[outer] * outerMass;
            cy += nodes.posY[inner] * innerMass + nodes.posY[outer] * outerMass;
            cz += nodes.posZ[inner] * innerMass + nodes.posZ[outer] * outerMass;
        }
        if (totalMass < 1e-9) return 0.0f;
        cx /= totalMass;
        cy /= totalMass;
        cz /= totalMass;
        double inertia = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            inertia += polarInertia(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
        }
        return (float) inertia;
    }

    private static float moveTowards(float current, float target, float maximumDelta) {
        if (maximumDelta == Float.POSITIVE_INFINITY) return target;
        if (current < target) return Math.min(current + maximumDelta, target);
        return Math.max(current - maximumDelta, target);
    }

    private static double angularMomentum(NodeContainer nodes, int node, double cx, double cy, double cz,
                                          double cvx, double cvy, double cvz,
                                          double ax, double ay, double az) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double vx = nodes.velX[node] - cvx, vy = nodes.velY[node] - cvy, vz = nodes.velZ[node] - cvz;
        double mass = Math.max(0.0, nodes.mass[node]);
        return mass * (ax * (ry * vz - rz * vy)
                + ay * (rz * vx - rx * vz) + az * (rx * vy - ry * vx));
    }

    private static double polarInertia(NodeContainer nodes, int node, double cx, double cy, double cz,
                                       double ax, double ay, double az) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double axial = rx * ax + ry * ay + rz * az;
        return Math.max(0.0, nodes.mass[node])
                * Math.max(0.0, rx * rx + ry * ry + rz * rz - axial * axial);
    }

    private static void applyAngularForce(NodeContainer nodes, int node, double cx, double cy, double cz,
                                          double ax, double ay, double az, double angularAcceleration) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double scale = angularAcceleration * Math.max(0.0, nodes.mass[node]);
        nodes.forceX[node] += (float) ((ay * rz - az * ry) * scale);
        nodes.forceY[node] += (float) ((az * rx - ax * rz) * scale);
        nodes.forceZ[node] += (float) ((ax * ry - ay * rx) * scale);
    }

    public void reset() {
        for (int i = 0; i < count; i++) {
            isDeflated[i] = false;
            serviceBrakeTorque[i] = 0.0f;
            brakeAngle[i] = 0.0f;
        }
    }

    public void clear() {
        count = 0;
        nameToIndex.clear();
    }
}
