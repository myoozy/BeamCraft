package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

import java.util.ArrayList;
import java.util.List;

public class FlexbodyBindingUtil {

    public static void performBinding(FlexbodyContainer flex, SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        if (flex.isSkinningBound || flex.meshCount == 0) return;

        int totalVerts = 0;
        for (int m = 0; m < flex.meshCount; m++) {
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom != null) totalVerts += geom.vertexCount;
        }

        flex.allocateSkinningBuffers(totalVerts);
        if (totalVerts == 0) {
            flex.isSkinningBound = true;
            return;
        }

        int ptr = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            // 提取目标模型允许绑定的物理节点群组约束池
            List<Integer> targetPool = new ArrayList<>();
            if (flex.targetGroups[m] != null) {
                for (String gName : flex.targetGroups[m]) {
                    Integer gId = flex.groupNameToId.get(gName);
                    if (gId != null) {
                        int start = flex.groupNodeOffsets[gId];
                        int count = flex.groupNodeCounts[gId];
                        for (int i = 0; i < count; i++) targetPool.add(flex.flatGroupNodes[start + i]);
                    }
                }
            }

            List<Integer> globalPool = new ArrayList<>(nodes.count);
            for (int i = 0; i < nodes.count; i++) globalPool.add(i);
            List<Integer> primaryPool = targetPool.isEmpty() ? globalPool : targetPool;

            float[] pos = geom.positions;
            float[] uvs = geom.uvs;

            // 提取 BeamNG 空间定义属性
            double sX = flex.scaleX[m], sY = flex.scaleY[m], sZ = flex.scaleZ[m];
            double pX = flex.posX[m],   pY = flex.posY[m],   pZ = flex.posZ[m];

            // 预计算内禀 Euler +Z+X+Y 旋转体系分量
            double rZ = Math.toRadians(flex.rotZ[m]);
            double rX = Math.toRadians(flex.rotX[m]);
            double rY = Math.toRadians(flex.rotY[m]);

            double cosZ = Math.cos(rZ), sinZ = Math.sin(rZ);
            double cosX = Math.cos(rX), sinX = Math.sin(rX);
            double cosY = Math.cos(rY), sinY = Math.sin(rY);

            JBeamAssembler.TransformContext slotCtx = flex.slotContext[m];

            for (int v = 0; v < geom.vertexCount; v++) {
                // =====================================================================
                // 🚀 完整出厂级联流水线：严格在原生 Z-up 空间完成组装映射
                // =====================================================================
                double origX = pos[v * 3];
                double origY = pos[v * 3 + 1];
                double origZ = pos[v * 3 + 2];

                // 1. 局部基础放缩
                origX *= sX; origY *= sY; origZ *= sZ;

                // 2. 局部内禀 Euler 顺次连乘旋转 (+Z+X+Y 准则)
                double x1 = origX * cosZ - origY * sinZ;
                double y1 = origX * sinZ + origY * cosZ;
                double z1 = origZ;

                double x2 = x1;
                double y2 = y1 * cosX - z1 * sinX;
                double z2 = y1 * sinX + z1 * cosX;

                double x3 = x2 * cosY + z2 * sinY;
                double y3 = y2;
                double z3 = -x2 * sinY + z2 * cosY;

                // 3. 叠加局部定义平移
                double lX = x3 + pX, lY = y3 + pY, lZ = z3 + pZ;

                // 4. 穿透应用零件插槽上下文级联结构 (包含 nodeOffset, nodeMove 与镜像翻转)
                double[] globalP = slotCtx != null ? slotCtx.transformNode(lX, lY, lZ) : new double[]{lX, lY, lZ};

                // 5. 🎯 世界交接闭环：转轴洗净至 Minecraft 的 Y-up 物理空间
                // 完美匹配 JBeamParser.java 的绝对出厂静止态参考系
                double staticMcX = +globalP[0];
                double staticMcY = +globalP[2];
                double staticMcZ = -globalP[1];

                flex.skinnedPosX[ptr] = (float) staticMcX;
                flex.skinnedPosY[ptr] = (float) staticMcY;
                flex.skinnedPosZ[ptr] = (float) staticMcZ;

                // 极速 O(N) 伴随节点搜索与混合解耦权重生成
                boolean success = calculateDecoupledWeights(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, primaryPool);
                if (!success && primaryPool != globalPool) {
                    success = calculateDecoupledWeights(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, globalPool);
                }
                if (!success) {
                    applyFallbackRigidBinding(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, primaryPool);
                }

                if (uvs != null && v * 2 + 1 < uvs.length) {
                    flex.uvU[ptr] = uvs[v * 2];
                    flex.uvV[ptr] = uvs[v * 2 + 1];
                } else {
                    flex.uvU[ptr] = 0.0f; flex.uvV[ptr] = 0.0f;
                }
                ptr++;
            }
        }
        flex.isSkinningBound = true;
        System.out.println("🎨 工业级动态混合软体蒙皮绑定通关！总顶点数: " + flex.totalVertexCount);
    }

    /**
     * 数学极简与绝对稳健的解耦投影权重解算器 (完全契合末端管线乘加流)
     */
    private static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                                     double vx, double vy, double vz, List<Integer> pool) {
        int poolSize = pool.size();
        if (poolSize < 3) return false;

        // 1. 线性搜寻欧几里得距离最近的原点伴随节点 REF -> O(N)
        int bestRef = pool.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            double dx = vx - nodes.posX[n], dy = vy - nodes.posY[n], dz = vz - nodes.posZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq) {
                minDistSq = dSq;
                bestRef = n;
            }
        }

        // 2. 线性搜寻次近节点作为 X 面内伴随轴 NX -> O(N)
        int bestNx = bestRef;
        double minDistNxSq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            if (n == bestRef) continue;
            double dx = vx - nodes.posX[n], dy = vy - nodes.posY[n], dz = vz - nodes.posZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistNxSq) {
                minDistNxSq = dSq;
                bestNx = n;
            }
        }

        if (bestNx == bestRef) return false;

        double cx = nodes.posX[bestRef], cy = nodes.posY[bestRef], cz = nodes.posZ[bestRef];
        double uX = nodes.posX[bestNx] - cx, uY = nodes.posY[bestNx] - cy, uZ = nodes.posZ[bestNx] - cz;
        double lenUSq = uX * uX + uY * uY + uZ * uZ;
        if (lenUSq < 1e-6) return false;
        double invLenU = 1.0 / Math.sqrt(lenUSq);

        // 3. 线性搜寻夹角达标的正交第三节点 NY -> O(N)
        int bestNy = bestRef;
        double minDistNySq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            if (n == bestRef || n == bestNx) continue;

            double wX = nodes.posX[n] - cx, wY = nodes.posY[n] - cy, wZ = nodes.posZ[n] - cz;
            double lenWSq = wX * wX + wY * wY + wZ * wZ;
            if (lenWSq < 1e-6) continue;

            // Rigs of Rods 原版夹角罗盘验证：强制保证基底平面结构极其扎实
            double dot = (wX * uX + wY * uY + wZ * uZ) * invLenU / Math.sqrt(lenWSq);
            if (Math.abs(dot) > 0.85) continue;

            double dx = vx - nodes.posX[n], dy = vy - nodes.posY[n], dz = vz - nodes.posZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistNySq) {
                minDistNySq = dSq;
                bestNy = n;
            }
        }

        // 放宽约束补救：选取正交结构投影支撑最佳的物理节点
        if (bestNy == bestRef) {
            double maxPerpSq = -1.0;
            double normUx = uX * invLenU, normUy = uY * invLenU, normUz = uZ * invLenU;
            for (int i = 0; i < poolSize; i++) {
                int n = pool.get(i);
                if (n == bestRef || n == bestNx) continue;
                double dx = nodes.posX[n] - cx, dy = nodes.posY[n] - cy, dz = nodes.posZ[n] - cz;
                double proj = dx * normUx + dy * normUy + dz * normUz;
                double pX = dx - proj * normUx, pY = dy - proj * normUy, pZ = dz - proj * normUz;
                double perpSq = pX * pX + pY * pY + pZ * pZ;
                if (perpSq > maxPerpSq && perpSq > 1e-5) {
                    maxPerpSq = perpSq;
                    bestNy = n;
                }
            }
        }

        if (bestNy == bestRef) return false;

        // 4. 提取本地伴随平面向量
        double vX = nodes.posX[bestNy] - cx, vY = nodes.posY[bestNy] - cy, vZ = nodes.posZ[bestNy] - cz;

        // 锁死单位化静止法线 (严格映射渲染端动态重构的归一化体系)
        double nX = uY * vZ - uZ * vY;
        double nY = uZ * vX - uX * vZ;
        double nZ = uX * vY - uY * vX;
        double lenN = Math.sqrt(nX * nX + nY * nY + nZ * nZ);
        if (lenN > 1e-7) {
            double invN = 1.0 / lenN;
            nX *= invN; nY *= invN; nZ *= invN;
        } else return false;

        // 5. 极其纯粹的解耦权重提取
        double dX = vx - cx, dY = vy - cy, dZ = vz - cz;

        // A. 绝对垂直厚度权重 wZ (纯物理米数)
        double wZ = dX * nX + dY * nY + dZ * nZ;

        // B. 剥离厚度分量得到纯面内向量
        double pX = dX - wZ * nX;
        double pY = dY - wZ * nY;
        double pZ = dZ - wZ * nZ;

        // C. 解算稳定对称 2D 点积方程直接提取面内仿射分量
        double u_u = uX * uX + uY * uY + uZ * uZ;
        double u_v = uX * vX + uY * vY + uZ * vZ;
        double v_v = vX * vX + vY * vY + vZ * vZ;
        double d_u = pX * uX + pY * uY + pZ * uZ;
        double d_v = pX * vX + pY * vY + pZ * vZ;

        double det2D = u_u * v_v - u_v * u_v;
        if (Math.abs(det2D) < 1e-9) return false;

        double invDet2D = 1.0 / det2D;
        float wX = (float) ((d_u * v_v - d_v * u_v) * invDet2D);
        float wY = (float) ((d_v * u_u - d_u * u_v) * invDet2D);

        // 物理常识保护锁：单个蒙皮零件表面距离伴随骨架中心绝不应超出极限范围
        if (Float.isNaN(wX) || Float.isNaN(wY) || Float.isNaN((float)wZ) ||
                Math.abs(wX) > 8.0f || Math.abs(wY) > 8.0f || Math.abs(wZ) > 8.0f) {
            return false;
        }

        flex.vCenterNode[ptr] = bestRef;
        flex.vVxNode[ptr]     = bestNx;
        flex.vVyNode[ptr]     = bestNy;
        flex.vWeightX[ptr]    = wX;
        flex.vWeightY[ptr]    = wY;
        flex.vWeightZ[ptr]    = (float) wZ;
        flex.vUseCrossZ[ptr]  = true;
        return true;
    }

    private static void applyFallbackRigidBinding(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                                  double vx, double vy, double vz, List<Integer> pool) {
        int bestC = pool.isEmpty() ? 0 : pool.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (int n : pool) {
            double dx = vx - nodes.posX[n], dy = vy - nodes.posY[n], dz = vz - nodes.posZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq) {
                minDistSq = dSq;
                bestC = n;
            }
        }

        flex.vCenterNode[ptr] = bestC;
        flex.vVxNode[ptr]     = bestC;
        flex.vVyNode[ptr]     = bestC;
        flex.vWeightX[ptr]    = 0.0f;
        flex.vWeightY[ptr]    = 0.0f;
        flex.vWeightZ[ptr]    = 0.0f;
        flex.vUseCrossZ[ptr]  = false;

        flex.skinnedPosX[ptr] = (float) (vx - nodes.posX[bestC]);
        flex.skinnedPosY[ptr] = (float) (vy - nodes.posY[bestC]);
        flex.skinnedPosZ[ptr] = (float) (vz - nodes.posZ[bestC]);
    }
}