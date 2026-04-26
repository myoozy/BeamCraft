package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;

public class TorsionBarContainer {
    public static final int INIT_TORSION_CAP = 64;

    // === 5. 扭杆数据 ===
    public int count = 0;
    public int[] node1 = new int[INIT_TORSION_CAP];
    public int[] node2 = new int[INIT_TORSION_CAP];
    public int[] node3 = new int[INIT_TORSION_CAP];
    public int[] node4 = new int[INIT_TORSION_CAP];

    public double[] restAngle = new double[INIT_TORSION_CAP]; // 静止角度
    public double[] baseRestAngle = new double[INIT_TORSION_CAP];
    public double[] spring = new double[INIT_TORSION_CAP];
    public double[] damp = new double[INIT_TORSION_CAP];
    public double[] deform = new double[INIT_TORSION_CAP];
    public double[] strength = new double[INIT_TORSION_CAP];
    public boolean[] broken = new boolean[INIT_TORSION_CAP];

    private void ensureCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            node3 = Utility.expand(node3, newSize);
            node4 = Utility.expand(node4, newSize);
            restAngle = Utility.expand(restAngle, newSize);
            baseRestAngle =  Utility.expand(baseRestAngle, newSize);
            spring =  Utility.expand(spring, newSize);
            damp = Utility.expand(damp, newSize);
            deform = Utility.expand(deform, newSize);
            strength = Utility.expand(strength, newSize);
            broken = Utility.expand(broken, newSize);
            System.out.println("⚠️ [TorsionBarContainer] Resized to: " + newSize);
        }
    }

    public void addTorsionBar(int index1, int index2, int index3, int index4,
                              double posX1, double posX2, double posX3, double posX4,
                              double posY1, double posY2, double posY3, double posY4,
                              double posZ1, double posZ2, double posZ3, double posZ4,
                              double torsionSpring, double torsionDamp, double torsionDeform, double torsionStrength) {
        ensureCapacity();

        int n1 = index1; int n2 = index2;
        int n3 = index3; int n4 = index4;

        node1[count] = n1; node2[count] = n2;
        node3[count] = n3; node4[count] = n4;

        spring[count] = torsionSpring; damp[count] = torsionDamp;
        deform[count] = torsionDeform; strength[count] = torsionStrength;
        broken[count] = false;

        // 获取初始坐标
        double x1 = posX1, y1 = posY1, z1 = posZ1;
        double x2 = posX2, y2 = posY2, z2 = posZ2;
        double x3 = posX3, y3 = posY3, z3 = posZ3;
        double x4 = posX4, y4 = posY4, z4 = posZ4;

        // 算出 3 根向量: b1, b2, b3
        double b1x = x2 - x1, b1y = y2 - y1, b1z = z2 - z1;
        double b2x = x3 - x2, b2y = y3 - y2, b2z = z3 - z2;
        double b3x = x4 - x3, b3y = y4 - y3, b3z = z4 - z3;

        // 算出两个面的法向量 (叉乘): c1 = b1 x b2, c2 = b2 x b3
        double c1x = b1y * b2z - b1z * b2y;
        double c1y = b1z * b2x - b1x * b2z;
        double c1z = b1x * b2y - b1y * b2x;

        double c2x = b2y * b3z - b2z * b3y;
        double c2y = b2z * b3x - b2x * b3z;
        double c2z = b2x * b3y - b2y * b3x;

        // 利用 atan2 计算二面角 (Dihedral Angle)
        double b2_mag = Math.sqrt(b2x*b2x + b2y*b2y + b2z*b2z);
        double c1Xc2_x = c1y * c2z - c1z * c2y;
        double c1Xc2_y = c1z * c2x - c1x * c2z;
        double c1Xc2_z = c1x * c2y - c1y * c2x;

        double dot1 = (c1Xc2_x * b2x + c1Xc2_y * b2y + c1Xc2_z * b2z) / b2_mag;
        double dot2 = c1x * c2x + c1y * c2y + c1z * c2z;

        double angle = Math.atan2(dot1, dot2);

        restAngle[count] = angle;
        baseRestAngle[count] = restAngle[count];

        count++;
    }

    public void clear() {
        count = 0;
        reset();
    }
    
    public void reset() {
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            restAngle[i] = baseRestAngle[i];
        }
    }
}