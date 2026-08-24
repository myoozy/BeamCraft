package me.mzy.beamcraft.utility;

public class Utility {
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
}
