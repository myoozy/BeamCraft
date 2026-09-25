package me.mzy.beamcraft.utility;

public final class Utility {
    private static final double EIGEN_EPSILON = 1.0e-9;

    private Utility() {}

    public static double[] expand(double[] arr, int newSize) { return java.util.Arrays.copyOf(arr, newSize); }
    public static float[] expand(float[] arr, int newSize) { return java.util.Arrays.copyOf(arr, newSize); }
    public static int[] expand(int[] arr, int newSize) { return java.util.Arrays.copyOf(arr, newSize); }
    public static boolean[] expand(boolean[] arr, int newSize) { return java.util.Arrays.copyOf(arr, newSize); }
    public static String[] expand(String[] arr, int newSize) { return java.util.Arrays.copyOf(arr, newSize); }

    public static double invSqrt(double x) {
        double xhalf = 0.5d * x;
        long i = Double.doubleToLongBits(x);
        i = 0x5fe6eb50c7b537a9L - (i >> 1);
        x = Double.longBitsToDouble(i);
        x = x * (1.5d - xhalf * x * x);
        return x;
    }

    public static float invSqrt(float x) {
        float xhalf = 0.5f * x;
        int i = Float.floatToIntBits(x);
        i = 0x5f3759df - (i >> 1);
        x = Float.intBitsToFloat(i);
        x = x * (1.5f - xhalf * x * x);
        return x;
    }

    public static float positive(float value) {
        return Math.max(0.0f, value);
    }

    public static float maxPositive(float... values) {
        float result = 0.0f;
        for (float value : values) result = Math.max(result, value);
        return result;
    }

    /** Converts an orthonormal row-major rotation matrix to an (x,y,z,w) quaternion. */
    public static void quaternionFromRotationRows(float[] q,
                                                  float m00, float m01, float m02,
                                                  float m10, float m11, float m12,
                                                  float m20, float m21, float m22) {
        float fourX2 = 1.0f + m00 - m11 - m22;
        float fourY2 = 1.0f - m00 + m11 - m22;
        float fourZ2 = 1.0f - m00 - m11 + m22;
        float fourW2 = 1.0f + m00 + m11 + m22;

        int largest = 0;
        float largestSquared4 = fourW2;
        if (fourX2 > largestSquared4) { largest = 1; largestSquared4 = fourX2; }
        if (fourY2 > largestSquared4) { largest = 2; largestSquared4 = fourY2; }
        if (fourZ2 > largestSquared4) { largest = 3; largestSquared4 = fourZ2; }

        float component = 0.5f * (float) Math.sqrt(Math.max(0.0f, largestSquared4));
        float scale = component > 1.0e-8f ? 0.25f / component : 0.0f;
        switch (largest) {
            case 1 -> {
                q[0] = component;
                q[1] = (m01 + m10) * scale;
                q[2] = (m02 + m20) * scale;
                q[3] = (m21 - m12) * scale;
            }
            case 2 -> {
                q[0] = (m01 + m10) * scale;
                q[1] = component;
                q[2] = (m12 + m21) * scale;
                q[3] = (m02 - m20) * scale;
            }
            case 3 -> {
                q[0] = (m02 + m20) * scale;
                q[1] = (m12 + m21) * scale;
                q[2] = component;
                q[3] = (m10 - m01) * scale;
            }
            default -> {
                q[0] = (m21 - m12) * scale;
                q[1] = (m02 - m20) * scale;
                q[2] = (m10 - m01) * scale;
                q[3] = component;
            }
        }
    }

    /** Premultiplies by an intrinsic Y-Z-X Euler rotation. */
    public static void premultiplyEulerYzx(float[] q, float x, float y, float z) {
        premultiplyQuaternion(q, (float) Math.sin(x * 0.5f), 0.0f, 0.0f,
                (float) Math.cos(x * 0.5f));
        premultiplyQuaternion(q, 0.0f, 0.0f, (float) Math.sin(z * 0.5f),
                (float) Math.cos(z * 0.5f));
        premultiplyQuaternion(q, 0.0f, (float) Math.sin(y * 0.5f), 0.0f,
                (float) Math.cos(y * 0.5f));
    }

    /** Stores the vector part of q^-1 * (v,0) * q at arbitrary output indices. */
    public static void rotateVectorByInverseQuaternion(
            float[] out, int xIndex, int yIndex, int zIndex,
            float qx, float qy, float qz, float qw,
            float vx, float vy, float vz) {
        float ax = qw * vx - qy * vz + qz * vy;
        float ay = qw * vy - qz * vx + qx * vz;
        float az = qw * vz - qx * vy + qy * vx;
        float aw = qx * vx + qy * vy + qz * vz;
        out[xIndex] = aw * qx + ax * qw + ay * qz - az * qy;
        out[yIndex] = aw * qy - ax * qz + ay * qw + az * qx;
        out[zIndex] = aw * qz + ax * qy - ay * qx + az * qw;
    }

    private static void premultiplyQuaternion(float[] q,
                                              float leftX, float leftY,
                                              float leftZ, float leftW) {
        float rightX = q[0], rightY = q[1], rightZ = q[2], rightW = q[3];
        q[0] = leftW * rightX + leftX * rightW + leftY * rightZ - leftZ * rightY;
        q[1] = leftW * rightY - leftX * rightZ + leftY * rightW + leftZ * rightX;
        q[2] = leftW * rightZ + leftX * rightY - leftY * rightX + leftZ * rightW;
        q[3] = leftW * rightW - leftX * rightX - leftY * rightY - leftZ * rightZ;
    }

    public static float reducedMass(float mass1, float mass2) {
        if (mass1 <= 0.0f || mass2 <= 0.0f) return 0.0f;
        return mass1 * mass2 / (mass1 + mass2);
    }

    /**
     * Caps two non-negative coefficients to a shared sum while preserving the
     * smaller coefficient whenever the budget permits it.
     */
    public static FloatPair capPairToSum(float first, float second, float totalCeiling) {
        float a = positive(first);
        float b = positive(second);
        float ceiling = positive(totalCeiling);
        if (a + b <= ceiling) return new FloatPair(first, second);

        float half = ceiling * 0.5f;
        if (a <= b) {
            float keptA = Math.min(a, half);
            return new FloatPair(keptA, Math.min(b, ceiling - keptA));
        }
        float keptB = Math.min(b, half);
        return new FloatPair(Math.min(a, ceiling - keptB), keptB);
    }

    /** Returns the largest eigenvalue of a real symmetric 3x3 matrix. */
    public static double maxEigenvalueSym3x3(double a00, double a01, double a02,
                                             double a11, double a12, double a22) {
        double trace = a00 + a11 + a22;
        double q = a00 * a11 + a00 * a22 + a11 * a22
                - a01 * a01 - a02 * a02 - a12 * a12;
        double determinant = a00 * (a11 * a22 - a12 * a12)
                - a01 * (a01 * a22 - a12 * a02)
                + a02 * (a01 * a12 - a11 * a02);

        double p = q - trace * trace / 3.0;
        double depressedQ = -2.0 * trace * trace * trace / 27.0
                + trace * q / 3.0 - determinant;
        double scale = Math.max(1.0, Math.abs(trace));
        if (Math.abs(p) <= 1.0e-14 * scale * scale) {
            return trace / 3.0 - Math.cbrt(depressedQ);
        }

        p = Math.min(p, 0.0);
        double radius = 2.0 * Math.sqrt(-p / 3.0);
        if (radius == 0.0) return trace / 3.0;
        double argument = (3.0 * depressedQ / (2.0 * p)) * Math.sqrt(-3.0 / p);
        argument = Math.clamp(argument, -1.0, 1.0);
        double angle = Math.acos(argument) / 3.0;
        return radius * Math.cos(angle) + trace / 3.0;
    }

    /** Returns the dominant eigenvalue and a corresponding unit eigenvector. */
    public static SymmetricEigenpair3 dominantEigenpairSym3x3(
            double a00, double a01, double a02,
            double a11, double a12, double a22) {
        double value = maxEigenvalueSym3x3(a00, a01, a02, a11, a12, a22);
        if (!(value > EIGEN_EPSILON) || !Double.isFinite(value)) {
            return new SymmetricEigenpair3(Math.max(0.0, value), 1.0, 0.0, 0.0);
        }

        double x;
        double y;
        double z;
        if (a00 >= a11 && a00 >= a22) {
            x = 1.0; y = 0.0; z = 0.0;
        } else if (a11 >= a22) {
            x = 0.0; y = 1.0; z = 0.0;
        } else {
            x = 0.0; y = 0.0; z = 1.0;
        }

        for (int i = 0; i < 24; i++) {
            double nextX = a00 * x + a01 * y + a02 * z;
            double nextY = a01 * x + a11 * y + a12 * z;
            double nextZ = a02 * x + a12 * y + a22 * z;
            double length = Math.sqrt(nextX * nextX + nextY * nextY + nextZ * nextZ);
            if (!(length > EIGEN_EPSILON) || !Double.isFinite(length)) break;
            x = nextX / length;
            y = nextY / length;
            z = nextZ / length;
        }
        return new SymmetricEigenpair3(value, x, y, z);
    }

    public record FloatPair(float first, float second) {}

    public record SymmetricEigenpair3(double value, double x, double y, double z) {}
}
