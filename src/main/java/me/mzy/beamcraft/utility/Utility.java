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
