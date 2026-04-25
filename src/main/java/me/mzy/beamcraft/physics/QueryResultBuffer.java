package me.mzy.beamcraft.physics;

import java.util.Arrays;

public class QueryResultBuffer {
    public SoftBodyVehicle[] vehicles = new SoftBodyVehicle[128];
    public int[] nodeIds = new int[128];
    public int count = 0;

    public void clear() {
        count = 0;
    }

    public void add(SoftBodyVehicle v, int id) {
        // 动态扩容，通常一次查询不会超过 128 个点，所以极少触发扩容
        if (count >= vehicles.length) {
            vehicles = Arrays.copyOf(vehicles, vehicles.length * 2);
            nodeIds = Arrays.copyOf(nodeIds, nodeIds.length * 2);
        }
        vehicles[count] = v;
        nodeIds[count] = id;
        count++;
    }
}
