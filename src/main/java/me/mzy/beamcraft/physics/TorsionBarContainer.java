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

        // --- 计算初始的二面角 (Rest Angle) ---
        // 向量：n2->n1, n2->n3, n2->n4
        double v1x = posX1 - posX2; double v1y = posY1 - posY2; double v1z = posZ1 - posZ2;
        double v2x = posX3 - posX2; double v2y = posY3 - posY2; double v2z = posZ3 - posZ2;
        double v3x = posX4 - posX2; double v3y = posY4 - posY2; double v3z = posZ4 - posZ2;

        // 计算两个面的法向量 (叉乘)
        double n1x = v1y*v2z - v1z*v2y; double n1y = v1z*v2x - v1x*v2z; double n1z = v1x*v2y - v1y*v2x;
        double n2x = v3y*v2z - v3z*v2y; double n2y = v3z*v2x - v3x*v2z; double n2z = v3x*v2y - v3y*v2x;

        // 计算法向量的模长
        double len1 = Math.sqrt(n1x*n1x + n1y*n1y + n1z*n1z);
        double len2 = Math.sqrt(n2x*n2x + n2y*n2y + n2z*n2z);

        // 避免除零
        if (len1 > 0.0001 && len2 > 0.0001) {
            // 点乘求夹角 cos 值
            double dot = (n1x*n2x + n1y*n2y + n1z*n2z) / (len1 * len2);
            dot = Math.max(-1.0, Math.min(1.0, dot)); // 限制在 [-1, 1] 防止浮点溢出
            restAngle[count] = Math.acos(dot); // 记录原厂角度 (弧度)
        } else {
            restAngle[count] = 0;
        }

        count++;
    }

    public void clear() {
        count = 0;
        // 重置断裂状态
        for (int i = 0; i < broken.length; i++) {
            broken[i] = false;
        }
    }
}