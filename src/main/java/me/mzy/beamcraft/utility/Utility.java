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
}
