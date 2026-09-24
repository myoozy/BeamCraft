package me.mzy.beamcraft.client.physics;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * Immutable node adjacency derived from a vehicle's authored beam constraints.
 *
 * <p>The complete graph answers general connectivity questions. The cohesive
 * graph is deliberately narrower: it excludes one-way support beams and beams
 * carrying a break group, because neither is a reliable indication that two
 * nodes will continue to move as one local structure after damage.</p>
 */
public final class BeamGraph {
    private static final int EDGE_CONNECTED = 1;
    private static final int EDGE_COHESIVE = 2;
    private static final int EDGE_BREAK_GROUP = 4;

    private final int[][] neighbors;
    private final int[][] cohesiveNeighbors;
    private final Map<Long, Integer> edgeFlags;

    private BeamGraph(int[][] neighbors, int[][] cohesiveNeighbors, Map<Long, Integer> edgeFlags) {
        this.neighbors = neighbors;
        this.cohesiveNeighbors = cohesiveNeighbors;
        this.edgeFlags = Map.copyOf(edgeFlags);
    }

    public static BeamGraph from(SoftBodyVehicle vehicle) {
        Builder builder = new Builder(vehicle.nodes.count);
        builder.addContainer(vehicle.normalBeams, true);
        builder.addContainer(vehicle.supportBeams, false);
        builder.addContainer(vehicle.boundedBeams, true);
        builder.addLBeams(vehicle.lBeams);
        builder.addContainer(vehicle.anisotropicBeams, true);
        return builder.build();
    }

    public int nodeCount() {
        return neighbors.length;
    }

    public boolean directlyConnected(int first, int second) {
        return hasFlag(first, second, EDGE_CONNECTED);
    }

    public boolean cohesivelyConnected(int first, int second) {
        return hasFlag(first, second, EDGE_COHESIVE);
    }

    /** True when every direct constraint for the pair belongs to a break group. */
    public boolean connectedOnlyAcrossBreakGroup(int first, int second) {
        int flags = flags(first, second);
        return (flags & EDGE_CONNECTED) != 0
                && (flags & EDGE_BREAK_GROUP) != 0
                && (flags & EDGE_COHESIVE) == 0;
    }

    /**
     * Returns the shortest hop count up to {@code maximumHops}, or {@code -1}
     * when no path exists inside that radius.
     */
    public int hopDistance(int start, int target, int maximumHops, boolean cohesiveOnly) {
        if (!validNode(start) || !validNode(target) || maximumHops < 0) return -1;
        if (start == target) return 0;
        int[][] graph = cohesiveOnly ? cohesiveNeighbors : neighbors;
        if (Arrays.binarySearch(graph[start], target) >= 0) return 1;
        if (maximumHops < 2) return -1;
        for (int neighbor : graph[start]) {
            if (Arrays.binarySearch(graph[neighbor], target) >= 0) return 2;
        }
        if (maximumHops == 2) return -1;
        boolean[] visited = new boolean[graph.length];
        int[] queue = new int[graph.length];
        int[] depths = new int[graph.length];
        int read = 0, write = 0;
        visited[start] = true;
        queue[write] = start;
        depths[write++] = 0;
        while (read < write) {
            int node = queue[read];
            int depth = depths[read++];
            if (depth >= maximumHops) continue;
            for (int neighbor : graph[node]) {
                if (visited[neighbor]) continue;
                int nextDepth = depth + 1;
                if (neighbor == target) return nextDepth;
                visited[neighbor] = true;
                queue[write] = neighbor;
                depths[write++] = nextDepth;
            }
        }
        return -1;
    }

    private boolean hasFlag(int first, int second, int flag) {
        return (flags(first, second) & flag) != 0;
    }

    private int flags(int first, int second) {
        if (!validNode(first) || !validNode(second) || first == second) return 0;
        return edgeFlags.getOrDefault(edgeKey(first, second), 0);
    }

    private boolean validNode(int node) {
        return node >= 0 && node < neighbors.length;
    }

    private static long edgeKey(int first, int second) {
        int low = Math.min(first, second);
        int high = Math.max(first, second);
        return ((long) low << 32) | (high & 0xffffffffL);
    }

    private static final class Builder {
        private final int nodeCount;
        private final Map<Long, Integer> edges = new HashMap<>();

        Builder(int nodeCount) {
            this.nodeCount = Math.max(0, nodeCount);
        }

        void addContainer(BeamContainer beams, boolean canBeCohesive) {
            for (int beam = 0; beam < beams.count; beam++) {
                addEdge(beams.node1[beam], beams.node2[beam], canBeCohesive,
                        hasBreakGroup(beams, beam));
            }
        }

        void addLBeams(LBeamContainer beams) {
            for (int beam = 0; beam < beams.count; beam++) {
                boolean breakGroup = hasBreakGroup(beams, beam);
                int first = beams.node1[beam];
                int second = beams.node2[beam];
                int pivot = beams.node3[beam];
                addEdge(first, second, true, breakGroup);
                addEdge(first, pivot, true, breakGroup);
                addEdge(second, pivot, true, breakGroup);
            }
        }

        private void addEdge(int first, int second, boolean canBeCohesive, boolean breakGroup) {
            if (first < 0 || second < 0 || first >= nodeCount || second >= nodeCount || first == second) return;
            int flags = EDGE_CONNECTED;
            if (breakGroup) flags |= EDGE_BREAK_GROUP;
            if (canBeCohesive && !breakGroup) flags |= EDGE_COHESIVE;
            edges.merge(edgeKey(first, second), flags, (existing, added) -> existing | added);
        }

        BeamGraph build() {
            int[] degree = new int[nodeCount];
            int[] cohesiveDegree = new int[nodeCount];
            for (Map.Entry<Long, Integer> edge : edges.entrySet()) {
                int first = (int) (edge.getKey() >>> 32);
                int second = (int) (long) edge.getKey();
                degree[first]++;
                degree[second]++;
                if ((edge.getValue() & EDGE_COHESIVE) != 0) {
                    cohesiveDegree[first]++;
                    cohesiveDegree[second]++;
                }
            }
            int[][] neighbors = allocateRows(degree);
            int[][] cohesiveNeighbors = allocateRows(cohesiveDegree);
            int[] positions = new int[nodeCount];
            int[] cohesivePositions = new int[nodeCount];
            for (Map.Entry<Long, Integer> edge : edges.entrySet()) {
                int first = (int) (edge.getKey() >>> 32);
                int second = (int) (long) edge.getKey();
                neighbors[first][positions[first]++] = second;
                neighbors[second][positions[second]++] = first;
                if ((edge.getValue() & EDGE_COHESIVE) != 0) {
                    cohesiveNeighbors[first][cohesivePositions[first]++] = second;
                    cohesiveNeighbors[second][cohesivePositions[second]++] = first;
                }
            }
            for (int[] row : neighbors) Arrays.sort(row);
            for (int[] row : cohesiveNeighbors) Arrays.sort(row);
            return new BeamGraph(neighbors, cohesiveNeighbors, edges);
        }

        private static int[][] allocateRows(int[] sizes) {
            int[][] result = new int[sizes.length][];
            for (int node = 0; node < sizes.length; node++) result[node] = new int[sizes[node]];
            return result;
        }

        private static boolean hasBreakGroup(BeamContainer beams, int beam) {
            return beams.assignedBreakGroups != null
                    && beams.assignedBreakGroups[beam] != null
                    && !beams.assignedBreakGroups[beam].isEmpty();
        }
    }
}
