package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TriangleContainer {
    public static final int INIT_TRI_CAP = 128;

    public int count = 0;
    public int[] node1 = new int[INIT_TRI_CAP];
    public int[] node2 = new int[INIT_TRI_CAP];
    public int[] node3 = new int[INIT_TRI_CAP];

    // 三角面的零件归属
    public int[] partId = new int[INIT_TRI_CAP];

    public boolean[] collision = new boolean[INIT_TRI_CAP];
    public boolean[] broken = new boolean[INIT_TRI_CAP];
    public List<String>[] assignedBreakGroups = new List[INIT_TRI_CAP];

    private Map<Long, int[]> edgeToTriangles = Map.of();
    private Map<String, int[]> breakGroupToTriangles = Map.of();

    private void ensureTriangleCapacity() {
        if (count >= node1.length) {
            int newSize = node1.length * 2;
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            node3 = Utility.expand(node3, newSize);
            partId = Utility.expand(partId, newSize);
            collision = Utility.expand(collision, newSize);
            broken = Utility.expand(broken, newSize);
            assignedBreakGroups = Arrays.copyOf(assignedBreakGroups, newSize);
            System.out.println("⚠️ [TriangleContainer] Resized to: " + newSize);
        }
    }

    public void addTriangle(PhysicsSpecs.TriangleSpec spec, int index1, int index2, int index3) {
        ensureTriangleCapacity();

        node1[count] = index1;
        node2[count] = index2;
        node3[count] = index3;
        partId[count] = spec.partId();
        collision[count] = spec.collision();
        broken[count] = false;
        assignedBreakGroups[count] = spec.breakGroups() == null || spec.breakGroups().isEmpty()
                ? null
                : new ArrayList<>(spec.breakGroups());
        count++;
    }

    public void buildBreakIndices() {
        Map<Long, List<Integer>> edgeLists = new HashMap<>();
        Map<String, List<Integer>> groupLists = new HashMap<>();

        for (int i = 0; i < count; i++) {
            addEdge(edgeLists, node1[i], node2[i], i);
            addEdge(edgeLists, node2[i], node3[i], i);
            addEdge(edgeLists, node3[i], node1[i], i);

            List<String> groups = assignedBreakGroups[i];
            if (groups != null) {
                for (String group : groups) {
                    groupLists.computeIfAbsent(group, ignored -> new ArrayList<>()).add(i);
                }
            }
        }

        edgeToTriangles = freezeIndex(edgeLists);
        breakGroupToTriangles = freezeIndex(groupLists);
    }

    public void breakByEdge(int nodeA, int nodeB) {
        markBroken(edgeToTriangles.get(edgeKey(nodeA, nodeB)));
    }

    public void breakByGroup(String group) {
        markBroken(breakGroupToTriangles.get(group));
    }

    public void reset() {
        Arrays.fill(broken, 0, count, false);
    }

    private static void addEdge(Map<Long, List<Integer>> index, int nodeA, int nodeB, int triangleIdx) {
        index.computeIfAbsent(edgeKey(nodeA, nodeB), ignored -> new ArrayList<>()).add(triangleIdx);
    }

    private static long edgeKey(int nodeA, int nodeB) {
        int min = Math.min(nodeA, nodeB);
        int max = Math.max(nodeA, nodeB);
        return ((long) min << 32) | (max & 0xffffffffL);
    }

    private static <K> Map<K, int[]> freezeIndex(Map<K, List<Integer>> source) {
        Map<K, int[]> result = new HashMap<>(source.size());
        source.forEach((key, indices) -> result.put(
                key,
                indices.stream().mapToInt(Integer::intValue).toArray()
        ));
        return result;
    }

    private void markBroken(int[] triangleIndices) {
        if (triangleIndices == null) return;
        for (int triangleIdx : triangleIndices) {
            broken[triangleIdx] = true;
        }
    }

    public void clear() {
        count = 0;
        edgeToTriangles = Map.of();
        breakGroupToTriangles = Map.of();
    }
}
