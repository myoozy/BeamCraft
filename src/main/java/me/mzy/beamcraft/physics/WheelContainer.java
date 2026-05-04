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

    private final SoftBodyVehicle vehicle;

    public WheelContainer(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
    }

    /**
     * 生成轮毂 (Hub)
     */
    public void generateHub(String wheelName, int n1, int n2, Integer nodeS, Integer nodeArm, int rays,
                            double radius, double width, double offset,
                            double nodeWeight, double frictionCoef,
                            double hubTreadS, double hubTreadD,
                            double hubPeriS, double hubPeriD,
                            double hubSideS, double hubSideD) {
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
        calculateWheelBasis(n1, n2, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        // n1 是车轮的绝对几何原点，offset 沿着轮轴方向偏移
        double centerX = vehicle.nodes.posX[n1] + axisX[0] * offset;
        double centerY = vehicle.nodes.posY[n1] + axisY[0] * offset;
        double centerZ = vehicle.nodes.posZ[n1] + axisZ[0] * offset;

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

        // 默认的塑性变形和断裂强度（合金轮毂非常坚硬）
        double deform = 50000, strength = 500000;
        boolean COLLISION = false;

        // 3. 生成物理拓扑 (Beams)
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;
            int hInCur = hubInnerNodes[baseOffset + i], hInNext = hubInnerNodes[baseOffset + next];
            int hOutCur = hubOuterNodes[baseOffset + i], hOutNext = hubOuterNodes[baseOffset + next];

            // --- 碰撞面 ---
            vehicle.triangles.addTriangle(n1, hInNext, hInCur, partId, COLLISION);
            vehicle.triangles.addTriangle(n2, hOutCur, hOutNext, partId, COLLISION);

            // ================= 1. 轮辋蒙皮 =================
            // 周长支撑 (Tread)
            addFastBeam(hInCur, hInNext, hubTreadS, hubTreadD, deform, strength);
            addFastBeam(hOutCur, hOutNext, hubTreadS, hubTreadD, deform, strength);

            // 横向支撑与 X 型交叉防扭曲 (Periphery)
            addFastBeam(hInCur, hOutCur, hubPeriS, hubPeriD, deform, strength); // 直连
            addFastBeam(hInCur, hOutNext, hubPeriS, hubPeriD, deform, strength); // 交叉 1
            addFastBeam(hOutCur, hInNext, hubPeriS, hubPeriD, deform, strength); // 交叉 2

            // ================= 2. 自行车交叉辐条 (Spokes) =================
            // a) 直连辐条 (内圈连内侧轴，外圈连外侧轴)
            addFastBeam(hInCur, n1, hubSideS, hubSideD, deform, strength);
            addFastBeam(hOutCur, n2, hubSideS, hubSideD, deform, strength);

            // b) 交叉辐条 (内圈连外侧轴，外圈连内侧轴)
            addFastBeam(hInCur, n2, hubSideS, hubSideD, deform, strength);
            addFastBeam(hOutCur, n1, hubSideS, hubSideD, deform, strength);

            // ================= 3. 稳定节点支撑 (nodeS) =================
            // 将轮毂内外圈所有节点都与 nodeS 相连，分摊 n2 的受力
            if (nodeS != null) {
                addFastBeam(hInCur, nodeS, hubSideS, hubSideD, deform, strength);
                addFastBeam(hOutCur, nodeS, hubSideS, hubSideD, deform, strength);
            }
        }

        count++;
    }

    /**
     * 生成轮胎 (Tire)
     */
    public void generateTire(String wheelName, int n1, int n2, int rays, double radius, double width, double offset,
                             double nodeWeight, double frictionCoef, double pressure,
                             double treadSpring, double treadDamp, double periSpring, double periDamp,
                             double sideSpring, double sideDamp, double reinfS, double reinfD) {

        if (!nameToIndex.containsKey(wheelName)) return;

        ensureWheelCapacity();

        int wIdx = nameToIndex.get(wheelName);
        int baseOffset = wIdx * MAX_RAYS;
        int partId = vehicle.nodes.partId[n1];

        tireRadius[wIdx] = radius;
        tireWidth[wIdx] = width;
        this.frictionCoef[wIdx] = frictionCoef;
        this.pressurePSI[wIdx] = pressure;

        double[] uX = {0}, uY = {0}, uZ = {0};
        double[] vX = {0}, vY = {0}, vZ = {0};
        double[] axisX = {0}, axisY = {0}, axisZ = {0};
        calculateWheelBasis(n1, n2, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        double centerX = vehicle.nodes.posX[n1] + axisX[0] * offset;
        double centerY = vehicle.nodes.posY[n1] + axisY[0] * offset;
        double centerZ = vehicle.nodes.posZ[n1] + axisZ[0] * offset;

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

            vehicle.nodes.addNode(wheelName + "_tire_in_" + i, inX, inY, inZ, nodeWeight, frictionCoef, 0.0, partId, true, false);
            tireInnerNodes[baseOffset + i] = vehicle.nodes.count - 1;

            vehicle.nodes.addNode(wheelName + "_tire_out_" + i, outX, outY, outZ, nodeWeight, frictionCoef, 0.0, partId, true, false);
            tireOuterNodes[baseOffset + i] = vehicle.nodes.count - 1;
        }

        double deform = 22000, strength = 30000;
        boolean COLLISION = false;

        // 2. 缝合轮胎三角形与梁
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

            // 预计算气压带来的额外刚度
            double[] pressureBonus = calculatePressureBonus(wIdx, pressure);
            double finalTreadS = treadSpring + pressureBonus[0];
            double finalPeriS = periSpring + pressureBonus[1];
            double finalSideS = sideSpring + pressureBonus[2];

            // 轮胎周长支撑
            addFastBeam(tInCur, tInNext, finalTreadS, treadDamp, deform, strength);
            addFastBeam(tOutCur, tOutNext, finalTreadS, treadDamp, deform, strength);

            // 🚀 胎面 加强筋 (i 连 i+2)
            int next2 = (i + 2) % rays;
            int tInNext2 = tireInnerNodes[baseOffset + next2];
            int tOutNext2 = tireOuterNodes[baseOffset + next2];

            // 这里使用 TreadReinfS (胎面加强刚度)
            addFastBeam(tInCur, tInNext2, finalTreadS, treadDamp, deform, strength);
            addFastBeam(tOutCur, tOutNext2, finalTreadS, treadDamp, deform, strength);

            // 轮胎横向支撑
            addFastBeam(tInCur, tOutCur, finalPeriS, periDamp, deform, strength);
            addFastBeam(tInCur, tOutNext, finalPeriS, periDamp, deform, strength);
            addFastBeam(tOutCur, tInNext, finalPeriS, periDamp, deform, strength);

            // 轮胎侧壁支撑 (这里是将 Tire 挂在 Hub 上的关键)
            addFastBeam(hInCur, tInCur, finalSideS, sideDamp, deform, strength);
            addFastBeam(hOutCur, tOutCur, finalSideS, sideDamp, deform, strength);
            addFastBeam(hInCur, tOutCur, finalSideS, sideDamp, deform, strength);
            addFastBeam(hOutCur, tInCur, finalSideS, sideDamp, deform, strength);
        }
    }

        private void addFastBeam(int id1, int id2, double spring, double damp, double defrom, double strength) {
            double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
            double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
            double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
            double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);

            double m1 = vehicle.nodes.mass[id1];
            double m2 = vehicle.nodes.mass[id2];
            double reducedMass = m1 * m2 / (m1 + m2);
            vehicle.normalBeams.addBeam(id1, id2, dist, reducedMass, spring, damp, defrom, strength, 1.0, 0.0, 0.0);
        }

    private void calculateWheelBasis(int n1, int n2, double[] ax, double[] ay, double[] az, double[] ux, double[] uy, double[] uz, double[] vx, double[] vy, double[] vz) {
        double n1x = vehicle.nodes.posX[n1], n1y = vehicle.nodes.posY[n1], n1z = vehicle.nodes.posZ[n1];
        double n2x = vehicle.nodes.posX[n2], n2y = vehicle.nodes.posY[n2], n2z = vehicle.nodes.posZ[n2];

        ax[0] = n2x - n1x; ay[0] = n2y - n1y; az[0] = n2z - n1z;
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

    /**
     * 纯物理推导：将气压转换为附加强度 (无魔法数字)
     * 返回一个包含三个元素的数组: [TreadBonus, PeriBonus, SideBonus]
     */
    public double[] calculatePressureBonus(int wheelIndex, double currentPSI) {
        // 1 PSI ≈ 6894.76 Pa (N/m^2)
        double pressurePa = currentPSI * 6894.76;

        int rays = numRays[wheelIndex];
        double rTire = tireRadius[wheelIndex];
        double rHub = hubRadius[wheelIndex];
        double width = tireWidth[wheelIndex];

        // 1. 面积计算 (Area)
        // 胎面单扇区面积 = 弧长 * 宽度
        double treadArea = ((2.0 * Math.PI * rTire) / rays) * width;
        // 单侧胎壁单扇区面积 = 扇环面积
        double sideArea = (Math.PI * (rTire * rTire - rHub * rHub)) / rays;

        // 2. 形变参考量 (Delta X)
        // 假设刚度的物理意义是：当轮胎被压扁胎壁高度的 100% 时，恰好提供等同于该区域气压总推力的反力
        double sidewallHeight = rTire - rHub;
        double refDeflection = sidewallHeight * 100; // 参考形变量

        // 3. 刚度计算 (k = F / dx = (P * A) / dx)
        double treadBonus = (pressurePa * treadArea) / refDeflection;
        double sideBonus = (pressurePa * sideArea) / refDeflection;

        // 轮胎宽度方向的横向支撑 (Periphery) 共享胎面的压力效应，但通常打个折以模拟张力分配
        double periBonus = treadBonus * 0.5;

        return new double[]{treadBonus, periBonus, sideBonus};
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