package me.mzy.beamcraft.physics;

// 引入 Minecraft 内置的 Fastutil 库
import it.unimi.dsi.fastutil.longs.Long2ObjectMap;
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap;

public class VoxelSnapshot {
    public static final byte TYPE_AIR = 0;
    public static final byte TYPE_FULL = 1;
    public static final byte TYPE_COMPLEX = 2;

    public static class VoxelCell {
        public final byte type;
        public final float[] aabbs;
        public VoxelCell(byte type, float[] aabbs) {
            this.type = type;
            this.aabbs = aabbs;
        }
    }

    // 核心替换：使用无装箱的高性能基本类型 Map
    private final Long2ObjectMap<VoxelCell> cache = new Long2ObjectOpenHashMap<>();

    private static final VoxelCell CELL_AIR = new VoxelCell(TYPE_AIR, null);
    private static final VoxelCell CELL_FULL = new VoxelCell(TYPE_FULL, null);

    public void clear() {
        cache.clear();
    }

    public boolean hasCache(long posLong) {
        return cache.containsKey(posLong); // 这里的 posLong 不会触发自动装箱
    }

    public void cacheBlock(long posLong, byte type, float[] complexAabbs) {
        if (type == TYPE_AIR) {
            cache.put(posLong, CELL_AIR);
        } else if (type == TYPE_FULL) {
            cache.put(posLong, CELL_FULL);
        } else {
            cache.put(posLong, new VoxelCell(TYPE_COMPLEX, complexAabbs));
        }
    }

    public boolean isSolid(double worldX, double worldY, double worldZ) {
        int bx = (int) Math.floor(worldX);
        int by = (int) Math.floor(worldY);
        int bz = (int) Math.floor(worldZ);

        long posLong = asLong(bx, by, bz);

        // Fastutil 的 get 方法，时间复杂度极低的 O(1) 且无垃圾产生
        VoxelCell cell = cache.get(posLong);

        if (cell == null || cell.type == TYPE_AIR) return false;
        if (cell.type == TYPE_FULL) return true;

        double fracX = worldX - bx;
        double fracY = worldY - by;
        double fracZ = worldZ - bz;

        float[] aabbs = cell.aabbs;
        for (int i = 0; i < aabbs.length; i += 6) {
            if (fracX >= aabbs[i] && fracX <= aabbs[i+3] &&
                    fracY >= aabbs[i+1] && fracY <= aabbs[i+4] &&
                    fracZ >= aabbs[i+2] && fracZ <= aabbs[i+5]) {
                return true;
            }
        }
        return false;
    }

    public static long asLong(int x, int y, int z) {
        long l = 0L;
        l |= ((long) x & 0x3FFFFFFL) << 38;
        l |= ((long) y & 0xFFFL);
        l |= ((long) z & 0x3FFFFFFL) << 12;
        return l;
    }
}