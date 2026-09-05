package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class TorsionBarContainer {
    public static final int INIT_TORSION_CAP = 64;

    // === 5. 扭杆数据 ===
    public int count = 0;
    public int[] node1 = new int[INIT_TORSION_CAP];
    public int[] node2 = new int[INIT_TORSION_CAP];
    public int[] node3 = new int[INIT_TORSION_CAP];
    public int[] node4 = new int[INIT_TORSION_CAP];

    public float[] restAngle = new float[INIT_TORSION_CAP]; // 静止角度
    public float[] baseRestAngle = new float[INIT_TORSION_CAP];
    /** Runtime actuator offset from the deformable neutral angle, in radians. */
    public float[] actuationAngle = new float[INIT_TORSION_CAP];
    public float[] spring = new float[INIT_TORSION_CAP];
    public float[] damp = new float[INIT_TORSION_CAP];
    public float[] spring2 = new float[INIT_TORSION_CAP];
    public float[] damp2 = new float[INIT_TORSION_CAP];
    public float[] deform = new float[INIT_TORSION_CAP];
    public float[] baseDeform = new float[INIT_TORSION_CAP];
    public float[] strength = new float[INIT_TORSION_CAP];
    public float[] precompressionAngle = new float[INIT_TORSION_CAP];
    public float[] precompressionState = new float[INIT_TORSION_CAP];
    public float[] precompressionTime = new float[INIT_TORSION_CAP];
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
            actuationAngle = Utility.expand(actuationAngle, newSize);
            spring =  Utility.expand(spring, newSize);
            damp = Utility.expand(damp, newSize);
            spring2 = Utility.expand(spring2, newSize);
            damp2 = Utility.expand(damp2, newSize);
            deform = Utility.expand(deform, newSize);
            baseDeform = Utility.expand(baseDeform, newSize);
            strength = Utility.expand(strength, newSize);
            precompressionAngle = Utility.expand(precompressionAngle, newSize);
            precompressionState = Utility.expand(precompressionState, newSize);
            precompressionTime = Utility.expand(precompressionTime, newSize);
            broken = Utility.expand(broken, newSize);
            System.out.println("⚠️ [TorsionBarContainer] Resized to: " + newSize);
        }
    }

    public int addTorsionBar(PhysicsSpecs.TorsionBarSpec spec, int index1, int index2, int index3, int index4, NodeContainer nodes) {
        ensureCapacity();

        int n1 = index1; int n2 = index2;
        int n3 = index3; int n4 = index4;

        node1[count] = n1; node2[count] = n2;
        node3[count] = n3; node4[count] = n4;

        spring[count] = spec.spring(); damp[count] = spec.damp();
        spring2[count] = spec.spring2(); damp2[count] = spec.damp2();
        deform[count] = spec.deform(); baseDeform[count] = spec.deform();
        strength[count] = spec.strength();
        precompressionAngle[count] = spec.precompressionAngle();
        precompressionTime[count] = spec.precompressionTime();
        precompressionState[count] = spec.precompressionTime() > 0.0f
                ? 0.0f : spec.precompressionAngle();
        broken[count] = false;

        // 获取初始坐标
        double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
        double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
        double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];
        double x4 = nodes.posX[n4], y4 = nodes.posY[n4], z4 = nodes.posZ[n4];

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

        restAngle[count] = (float) angle;
        baseRestAngle[count] = restAngle[count];
        actuationAngle[count] = 0.0f;
        if (Double.isNaN(angle)) broken[count] = true;

        return count++;
    }

    public void clear() {
        count = 0;
        reset();
    }
    
    public void reset() {
        for (int i = 0; i < count; i++) {
            broken[i] = false;
            restAngle[i] = baseRestAngle[i];
            actuationAngle[i] = 0.0f;
            precompressionState[i] = precompressionTime[i] > 0.0f
                    ? 0.0f : precompressionAngle[i];
            deform[i] = baseDeform[i];
        }
    }
}
