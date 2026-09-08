package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

import java.util.ArrayList;
import java.util.List;

public class FlexbodyBindingUtil {

    /** Number of locally-nearest nodes considered when building a vertex basis. */
    static final int MAX_BASIS_CANDIDATES = 16;
    /** BeamNG describes VX/VY as roughly perpendicular; this is the preferred lower bound. */
    static final double PREFERRED_BASIS_ANGLE_DEGREES = 45.0;
    /** Below this angle the planar solve is too ill-conditioned to use safely. */
    static final double MIN_USABLE_BASIS_ANGLE_DEGREES = 20.0;
    /** Bounds used by BeamNG's Flexbody Debug to flag potentially spiking locator coordinates. */
    static final double PREFERRED_LOCATOR_MIN = -0.5;
    static final double PREFERRED_LOCATOR_MAX = 1.5;
    /** Last-resort finite guard. Values this large are never a defensible local locator. */
    static final double MAX_SAFE_LOCATOR_MAGNITUDE = 15.0;
    static final double MIN_AXIS_LENGTH_SQUARED = 1.0e-6;
    static final double MIN_BASIS_NORMAL_LENGTH_SQUARED = 1.0e-14;
    static final double MIN_INPUT_NORMAL_LENGTH = 1.0e-5;
    static final double MIN_INVERSE_SCALE_MAGNITUDE = 1.0e-12;

    public static void performBinding(FlexbodyContainer flex, SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        if (flex.isSkinningBound || flex.meshCount == 0) return;

        int totalVerts = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            boolean valid = true;
            if (flex.targetGroups[m] != null && !flex.targetGroups[m].isEmpty()) {
                boolean foundAny = false;
                for (String gName : flex.targetGroups[m]) {
                    if (flex.groupNameToId.containsKey(gName)) {
                        foundAny = true; break;
                    }
                }
                if (!foundAny) valid = false;
            }

            // 如果判定为幽灵网格，直接把它的名字清空。
            // 这样不仅这里不会统计它的顶点，后期的 Renderer 也会因为名字为空找不到模型而自动跳过！
            if (!valid) {
                flex.meshName[m] = "";
            } else {
                DaeMeshLoader.RawGeometry geom = DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[m]);
                if (geom != null) totalVerts += geom.vertexCount;
            }
        }

        flex.allocateSkinningBuffers(totalVerts);
        if (totalVerts == 0) {
            flex.isSkinningBound = true;
            return;
        }

        int ptr = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            // 直接判断名字是否为空，跳过被我们“处决”的幽灵网格
            if (flex.meshName[m].isEmpty()) continue;

            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[m]);
            if (geom == null) continue;

            List<Integer> primaryPool = new ArrayList<>();
            boolean[] addedToPool = new boolean[nodes.count];
            if (flex.targetGroups[m] != null && !flex.targetGroups[m].isEmpty()) {
                for (String gName : flex.targetGroups[m]) {
                    Integer gId = flex.groupNameToId.get(gName);
                    if (gId != null) {
                        int start = flex.groupNodeOffsets[gId];
                        int count = flex.groupNodeCounts[gId];
                        for (int i = 0; i < count; i++) {
                            int node = flex.flatGroupNodes[start + i];
                            if (!addedToPool[node]) {
                                addedToPool[node] = true;
                                primaryPool.add(node);
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < nodes.count; i++) primaryPool.add(i);
            }

            float[] pos = geom.positions;
            float[] norms = geom.normals;
            float[] uvs = geom.uvs;

            double sX = flex.scaleX[m], sY = flex.scaleY[m], sZ = flex.scaleZ[m];
            double pX = flex.posX[m],   pY = flex.posY[m],   pZ = flex.posZ[m];

            double rZ = Math.toRadians(flex.rotZ[m]);
            double rX = Math.toRadians(flex.rotX[m]);
            double rY = Math.toRadians(flex.rotY[m]);

            double cosZ = Math.cos(rZ), sinZ = Math.sin(rZ);
            double cosX = Math.cos(rX), sinX = Math.sin(rX);
            double cosY = Math.cos(rY), sinY = Math.sin(rY);

            JBeamAssembler.TransformContext slotCtx = flex.slotContext[m];

            for (int v = 0; v < geom.vertexCount; v++) {

                double origX = pos[v * 3], origY = pos[v * 3 + 1], origZ = pos[v * 3 + 2];
                origX *= sX; origY *= sY; origZ *= sZ;

                double x1 = origX * cosZ - origY * sinZ, y1 = origX * sinZ + origY * cosZ, z1 = origZ;
                double x2 = x1, y2 = y1 * cosX - z1 * sinX, z2 = y1 * sinX + z1 * cosX;
                double x3 = x2 * cosY + z2 * sinY, y3 = y2, z3 = -x2 * sinY + z2 * cosY;

                double lX = x3 + pX, lY = y3 + pY, lZ = z3 + pZ;
                double[] globalP = slotCtx != null ? slotCtx.transformNode(lX, lY, lZ) : new double[]{lX, lY, lZ};

                double staticMcX = +globalP[0];
                double staticMcY = +globalP[2];
                double staticMcZ = -globalP[1];

                flex.skinnedPosX[ptr] = (float) staticMcX;
                flex.skinnedPosY[ptr] = (float) staticMcY;
                flex.skinnedPosZ[ptr] = (float) staticMcZ;

                double nOrigX = 0, nOrigY = 0, nOrigZ = 1;
                if (norms != null && v * 3 + 2 < norms.length) {
                    // A normal is transformed by the inverse transpose of the
                    // position scale, not by the position scale itself.
                    double rawNx = inverseScaleNormal(norms[v * 3], sX);
                    double rawNy = inverseScaleNormal(norms[v * 3 + 1], sY);
                    double rawNz = inverseScaleNormal(norms[v * 3 + 2], sZ);
                    double nx1 = rawNx * cosZ - rawNy * sinZ, ny1 = rawNx * sinZ + rawNy * cosZ, nz1 = rawNz;
                    double nx2 = nx1, ny2 = ny1 * cosX - nz1 * sinX, nz2 = ny1 * sinX + nz1 * cosX;
                    double nx3 = nx2 * cosY + nz2 * sinY, ny3 = ny2, nz3 = -nx2 * sinY + nz2 * cosY;

                    double[] gNorm = slotCtx != null ? slotCtx.transformNode(nx3, ny3, nz3) : new double[]{nx3, ny3, nz3};
                    double[] gOrigin = slotCtx != null ? slotCtx.transformNode(0, 0, 0) : new double[]{0, 0, 0};

                    nOrigX = +(gNorm[0] - gOrigin[0]);
                    nOrigY = +(gNorm[2] - gOrigin[2]);
                    nOrigZ = -(gNorm[1] - gOrigin[1]);
                }

                // 彻底砍掉 globalPool 备用池逻辑！
                // 如果在自己的专属 Group 里找不到合适的投射面，乖乖原位退化成货斗门上的刚体，绝不越界去抓车身！
                boolean success = calculateDecoupledWeights(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, nOrigX, nOrigY, nOrigZ, primaryPool);
                if (!success) {
                    applyFallbackRigidBinding(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, nOrigX, nOrigY, nOrigZ, primaryPool);
                }

                if (uvs != null && v * 2 + 1 < uvs.length) {
                    flex.uvU[ptr] = uvs[v * 2]; flex.uvV[ptr] = uvs[v * 2 + 1];
                } else {
                    flex.uvU[ptr] = 0.0f; flex.uvV[ptr] = 0.0f;
                }
                ptr++;
            }
        }
        flex.isSkinningBound = true;
        System.out.println("🎨 工业级平滑蒙皮出厂绑定完美闭环！总渲染点数: " + flex.totalVertexCount);
    }

    private static double inverseScaleNormal(double component, double scale) {
        return Math.abs(scale) > MIN_INVERSE_SCALE_MAGNITUDE ? component / scale : 0.0;
    }

    static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                              double vx, double vy, double vz,
                                              double normX, double normY, double normZ, List<Integer> pool) {
        if (pool.size() < 3) return false;

        int centerNode = pool.getFirst();
        double nearestDistanceSquared = Double.POSITIVE_INFINITY;
        for (int node : pool) {
            double dx = vx - nodes.baseX[node], dy = vy - nodes.baseY[node], dz = vz - nodes.baseZ[node];
            double distanceSquared = dx * dx + dy * dy + dz * dz;
            if (distanceSquared < nearestDistanceSquared) {
                nearestDistanceSquared = distanceSquared;
                centerNode = node;
            }
        }

        int capacity = Math.min(MAX_BASIS_CANDIDATES, pool.size() - 1);
        int[] candidates = new int[capacity];
        double[] distances = new double[capacity];
        java.util.Arrays.fill(distances, Double.POSITIVE_INFINITY);
        int candidateCount = 0;
        for (int node : pool) {
            if (node == centerNode) continue;
            double dx = vx - nodes.baseX[node], dy = vy - nodes.baseY[node], dz = vz - nodes.baseZ[node];
            double distanceSquared = dx * dx + dy * dy + dz * dz;
            if (candidateCount == capacity && distanceSquared >= distances[capacity - 1]) continue;
            int insert = Math.min(candidateCount, capacity - 1);
            while (insert > 0 && distanceSquared < distances[insert - 1]) insert--;
            if (insert >= capacity) continue;
            int moveCount = Math.min(candidateCount, capacity - 1) - insert;
            if (moveCount > 0) {
                System.arraycopy(candidates, insert, candidates, insert + 1, moveCount);
                System.arraycopy(distances, insert, distances, insert + 1, moveCount);
            }
            candidates[insert] = node;
            distances[insert] = distanceSquared;
            if (candidateCount < capacity) candidateCount++;
        }
        if (candidateCount < 2) return false;

        BasisSolution best = new BasisSolution();
        boolean foundBasis = false;
        for (int first = 0; first < candidateCount - 1; first++) {
            for (int second = first + 1; second < candidateCount; second++) {
                foundBasis |= considerBasis(nodes, centerNode, candidates[first], candidates[second],
                        vx, vy, vz, distances[first] + distances[second], best);
            }
        }
        if (!foundBasis) return false;

        float normalWeightX = 0, normalWeightY = 0, normalWeightZ = 1;
        if (flex.vNormWeightX != null) {
            double normalLength = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            if (normalLength > MIN_INPUT_NORMAL_LENGTH) {
                double inX = normX / normalLength, inY = normY / normalLength, inZ = normZ / normalLength;
                double normalZ = inX * best.nX + inY * best.nY + inZ * best.nZ;
                double planarX = inX - normalZ * best.nX;
                double planarY = inY - normalZ * best.nY;
                double planarZ = inZ - normalZ * best.nZ;
                double dotU = planarX * best.uX + planarY * best.uY + planarZ * best.uZ;
                double dotV = planarX * best.vX + planarY * best.vY + planarZ * best.vZ;
                normalWeightX = (float) ((dotU * best.vLengthSquared - dotV * best.axisDot) * best.inverseDeterminant);
                normalWeightY = (float) ((dotV * best.uLengthSquared - dotU * best.axisDot) * best.inverseDeterminant);
                normalWeightZ = (float) normalZ;
            }
            flex.vNormWeightX[ptr] = normalWeightX;
            flex.vNormWeightY[ptr] = normalWeightY;
            flex.vNormWeightZ[ptr] = normalWeightZ;
        }

        flex.vCenterNode[ptr] = centerNode;
        flex.vVxNode[ptr] = best.vxNode;
        flex.vVyNode[ptr] = best.vyNode;
        flex.vWeightX[ptr] = best.weightX;
        flex.vWeightY[ptr] = best.weightY;
        flex.vWeightZ[ptr] = (float) best.weightZ;
        flex.vUseCrossZ[ptr] = true;
        return true;
    }

    private static boolean considerBasis(NodeContainer nodes, int centerNode, int vxNode, int vyNode,
                                         double px, double py, double pz, double pairDistanceSquared,
                                         BasisSolution best) {
        double cx = nodes.baseX[centerNode], cy = nodes.baseY[centerNode], cz = nodes.baseZ[centerNode];
        double uX = nodes.baseX[vxNode] - cx, uY = nodes.baseY[vxNode] - cy, uZ = nodes.baseZ[vxNode] - cz;
        double vX = nodes.baseX[vyNode] - cx, vY = nodes.baseY[vyNode] - cy, vZ = nodes.baseZ[vyNode] - cz;
        double uLengthSquared = uX * uX + uY * uY + uZ * uZ;
        double vLengthSquared = vX * vX + vY * vY + vZ * vZ;
        if (uLengthSquared < MIN_AXIS_LENGTH_SQUARED || vLengthSquared < MIN_AXIS_LENGTH_SQUARED) return false;

        double axisDot = uX * vX + uY * vY + uZ * vZ;
        double cosineSquared = axisDot * axisDot / (uLengthSquared * vLengthSquared);
        double sineSquared = 1.0 - Math.min(1.0, Math.max(0.0, cosineSquared));
        if (sineSquared < squaredSine(MIN_USABLE_BASIS_ANGLE_DEGREES)) return false;

        double crossX = uY * vZ - uZ * vY;
        double crossY = uZ * vX - uX * vZ;
        double crossZ = uX * vY - uY * vX;
        double crossLengthSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
        if (crossLengthSquared < MIN_BASIS_NORMAL_LENGTH_SQUARED) return false;
        double inverseCrossLength = 1.0 / Math.sqrt(crossLengthSquared);
        double nX = crossX * inverseCrossLength, nY = crossY * inverseCrossLength, nZ = crossZ * inverseCrossLength;

        double dX = px - cx, dY = py - cy, dZ = pz - cz;
        double weightZ = dX * nX + dY * nY + dZ * nZ;
        double planarX = dX - weightZ * nX, planarY = dY - weightZ * nY, planarZ = dZ - weightZ * nZ;
        double determinant = uLengthSquared * vLengthSquared - axisDot * axisDot;
        if (determinant <= MIN_BASIS_NORMAL_LENGTH_SQUARED) return false;
        double inverseDeterminant = 1.0 / determinant;
        double dotU = planarX * uX + planarY * uY + planarZ * uZ;
        double dotV = planarX * vX + planarY * vY + planarZ * vZ;
        float weightX = (float) ((dotU * vLengthSquared - dotV * axisDot) * inverseDeterminant);
        float weightY = (float) ((dotV * uLengthSquared - dotU * axisDot) * inverseDeterminant);
        if (!Float.isFinite(weightX) || !Float.isFinite(weightY) || !Double.isFinite(weightZ)
                || Math.abs(weightX) > MAX_SAFE_LOCATOR_MAGNITUDE
                || Math.abs(weightY) > MAX_SAFE_LOCATOR_MAGNITUDE
                || Math.abs(weightZ) > MAX_SAFE_LOCATOR_MAGNITUDE) return false;

        boolean preferredAngle = sineSquared >= squaredSine(PREFERRED_BASIS_ANGLE_DEGREES);
        boolean preferredCoordinates = withinPreferredLocatorBounds(weightX) && withinPreferredLocatorBounds(weightY);
        int preferenceTier = BasisSolution.preferenceTier(preferredAngle, preferredCoordinates);
        if (!best.accepts(preferenceTier, pairDistanceSquared, sineSquared)) return false;
        best.set(vxNode, vyNode, uX, uY, uZ, vX, vY, vZ, nX, nY, nZ,
                uLengthSquared, vLengthSquared, axisDot, inverseDeterminant, weightX, weightY, weightZ,
                pairDistanceSquared, sineSquared, preferenceTier);
        return true;
    }

    private static double squaredSine(double degrees) {
        double sine = Math.sin(Math.toRadians(degrees));
        return sine * sine;
    }

    private static boolean withinPreferredLocatorBounds(double value) {
        return value >= PREFERRED_LOCATOR_MIN && value <= PREFERRED_LOCATOR_MAX;
    }

    private static final class BasisSolution {
        int vxNode, vyNode;
        double uX, uY, uZ, vX, vY, vZ, nX, nY, nZ;
        double uLengthSquared, vLengthSquared, axisDot, inverseDeterminant;
        float weightX, weightY;
        double weightZ;
        double pairDistanceSquared = Double.POSITIVE_INFINITY;
        double sineSquared;
        int preferenceTier = Integer.MAX_VALUE;

        static int preferenceTier(boolean preferredAngle, boolean preferredCoordinates) {
            if (preferredAngle && preferredCoordinates) return 0;
            if (preferredAngle) return 1;
            if (preferredCoordinates) return 2;
            return 3;
        }

        boolean accepts(int tier, double distanceSquared, double candidateSineSquared) {
            if (tier != preferenceTier) return tier < preferenceTier;
            int distanceOrder = Double.compare(distanceSquared, pairDistanceSquared);
            return distanceOrder < 0 || (distanceOrder == 0 && candidateSineSquared > sineSquared);
        }

        void set(int vxNode, int vyNode,
                 double uX, double uY, double uZ, double vX, double vY, double vZ,
                 double nX, double nY, double nZ,
                 double uLengthSquared, double vLengthSquared, double axisDot, double inverseDeterminant,
                 float weightX, float weightY, double weightZ,
                 double pairDistanceSquared, double sineSquared, int preferenceTier) {
            this.vxNode = vxNode;
            this.vyNode = vyNode;
            this.uX = uX; this.uY = uY; this.uZ = uZ;
            this.vX = vX; this.vY = vY; this.vZ = vZ;
            this.nX = nX; this.nY = nY; this.nZ = nZ;
            this.uLengthSquared = uLengthSquared;
            this.vLengthSquared = vLengthSquared;
            this.axisDot = axisDot;
            this.inverseDeterminant = inverseDeterminant;
            this.weightX = weightX;
            this.weightY = weightY;
            this.weightZ = weightZ;
            this.pairDistanceSquared = pairDistanceSquared;
            this.sineSquared = sineSquared;
            this.preferenceTier = preferenceTier;
        }
    }

    private static void applyFallbackRigidBinding(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                                  double vx, double vy, double vz,
                                                  double normX, double normY, double normZ, List<Integer> pool) {
        int bestC = pool.isEmpty() ? 0 : pool.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (int n : pool) {
            double dx = vx - nodes.baseX[n], dy = vy - nodes.baseY[n], dz = vz - nodes.baseZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq) { minDistSq = dSq; bestC = n; }
        }

        flex.vCenterNode[ptr] = bestC;
        flex.vVxNode[ptr]     = bestC;
        flex.vVyNode[ptr]     = bestC;
        flex.vWeightX[ptr]    = 0.0f; flex.vWeightY[ptr]    = 0.0f; flex.vWeightZ[ptr]    = 0.0f;
        flex.vUseCrossZ[ptr]  = false;

        flex.skinnedPosX[ptr] = (float) (vx - nodes.baseX[bestC]);
        flex.skinnedPosY[ptr] = (float) (vy - nodes.baseY[bestC]);
        flex.skinnedPosZ[ptr] = (float) (vz - nodes.baseZ[bestC]);

        if (flex.vNormWeightX != null) {
            double nLen = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            flex.vNormWeightX[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normX / nLen : 0);
            flex.vNormWeightY[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normY / nLen : 1);
            flex.vNormWeightZ[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normZ / nLen : 0);
        }
    }
}
