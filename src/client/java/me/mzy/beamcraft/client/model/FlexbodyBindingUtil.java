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
                String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
                DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
                if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
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

            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            List<Integer> primaryPool = new ArrayList<>();
            if (flex.targetGroups[m] != null && !flex.targetGroups[m].isEmpty()) {
                for (String gName : flex.targetGroups[m]) {
                    Integer gId = flex.groupNameToId.get(gName);
                    if (gId != null) {
                        int start = flex.groupNodeOffsets[gId];
                        int count = flex.groupNodeCounts[gId];
                        for (int i = 0; i < count; i++) primaryPool.add(flex.flatGroupNodes[start + i]);
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
                if ((v & 3) == 3) {
                    flex.skinnedPosX[ptr]  = flex.skinnedPosX[ptr - 1];
                    flex.skinnedPosY[ptr]  = flex.skinnedPosY[ptr - 1];
                    flex.skinnedPosZ[ptr]  = flex.skinnedPosZ[ptr - 1];
                    flex.vCenterNode[ptr]  = flex.vCenterNode[ptr - 1];
                    flex.vVxNode[ptr]      = flex.vVxNode[ptr - 1];
                    flex.vVyNode[ptr]      = flex.vVyNode[ptr - 1];
                    flex.vWeightX[ptr]     = flex.vWeightX[ptr - 1];
                    flex.vWeightY[ptr]     = flex.vWeightY[ptr - 1];
                    flex.vWeightZ[ptr]     = flex.vWeightZ[ptr - 1];
                    flex.vUseCrossZ[ptr]   = flex.vUseCrossZ[ptr - 1];

                    if (flex.vNormWeightX != null) {
                        flex.vNormWeightX[ptr] = flex.vNormWeightX[ptr - 1];
                        flex.vNormWeightY[ptr] = flex.vNormWeightY[ptr - 1];
                        flex.vNormWeightZ[ptr] = flex.vNormWeightZ[ptr - 1];
                    }

                    if (uvs != null && v * 2 + 1 < uvs.length) {
                        flex.uvU[ptr] = uvs[v * 2]; flex.uvV[ptr] = uvs[v * 2 + 1];
                    }
                    ptr++;
                    continue;
                }

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
                    double rawNx = norms[v * 3], rawNy = norms[v * 3 + 1], rawNz = norms[v * 3 + 2];
                    double nx1 = rawNx * cosZ - rawNy * sinZ, ny1 = rawNx * sinZ + rawNy * cosZ, nz1 = rawNz;
                    double nx2 = nx1, ny2 = ny1 * cosX - nz1 * sinX, nz2 = ny1 * sinX + nz1 * cosX;
                    double nx3 = nx2 * cosY + nz2 * sinY, ny3 = ny2, nz3 = -nx2 * sinY + nz2 * cosY;

                    double[] gNorm = slotCtx != null ? slotCtx.transformNode(nx3, ny3, nz3) : new double[]{nx3, ny3, nz3};
                    double[] gOrigin = slotCtx != null ? slotCtx.transformNode(0, 0, 0) : new double[]{0, 0, 0};

                    nOrigX = +(gNorm[0] - gOrigin[0]);
                    nOrigY = +(gNorm[2] - gOrigin[2]);
                    nOrigZ = -(gNorm[1] - gOrigin[1]);
                }

                // 🌟 核心修复 2：彻底砍掉 globalPool 备用池逻辑！
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

    private static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                                     double vx, double vy, double vz,
                                                     double normX, double normY, double normZ, List<Integer> pool) {
        int poolSize = pool.size();
        if (poolSize < 3) return false;

        int bestRef = pool.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            double dx = vx - nodes.baseX[n], dy = vy - nodes.baseY[n], dz = vz - nodes.baseZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq) { minDistSq = dSq; bestRef = n; }
        }

        int bestNx = bestRef;
        double minDistNxSq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            if (n == bestRef) continue;
            double dx = vx - nodes.baseX[n], dy = vy - nodes.baseY[n], dz = vz - nodes.baseZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistNxSq) { minDistNxSq = dSq; bestNx = n; }
        }

        if (bestNx == bestRef) return false;

        double cx = nodes.baseX[bestRef], cy = nodes.baseY[bestRef], cz = nodes.baseZ[bestRef];
        double uX = nodes.baseX[bestNx] - cx, uY = nodes.baseY[bestNx] - cy, uZ = nodes.baseZ[bestNx] - cz;
        double lenUSq = uX * uX + uY * uY + uZ * uZ;
        if (lenUSq < 1e-6) return false;
        double invLenU = 1.0 / Math.sqrt(lenUSq);

        int bestNy = bestRef;
        double minDistNySq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            if (n == bestRef || n == bestNx) continue;

            double wX = nodes.baseX[n] - cx, wY = nodes.baseY[n] - cy, wZ = nodes.baseZ[n] - cz;
            double lenWSq = wX * wX + wY * wY + wZ * wZ;
            if (lenWSq < 1e-6) continue;

            double dot = (wX * uX + wY * uY + wZ * uZ) * invLenU / Math.sqrt(lenWSq);
            // 放宽共线判定，只要不是绝对平行即可，依靠后续的降维投影自适应
            if (Math.abs(dot) > 0.95) continue;

            double dx = vx - nodes.baseX[n], dy = vy - nodes.baseY[n], dz = vz - nodes.baseZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistNySq) { minDistNySq = dSq; bestNy = n; }
        }

        if (bestNy == bestRef) return false;

        double vX = nodes.baseX[bestNy] - cx, vY = nodes.baseY[bestNy] - cy, vZ = nodes.baseZ[bestNy] - cz;

        double nX = uY * vZ - uZ * vY;
        double nY = uZ * vX - uX * vZ;
        double nZ = uX * vY - uY * vX;
        double lenN = Math.sqrt(nX * nX + nY * nY + nZ * nZ);
        if (lenN > 1e-7) {
            double invN = 1.0 / lenN;
            nX *= invN; nY *= invN; nZ *= invN;
        } else return false;

        double dX = vx - cx, dY = vy - cy, dZ = vz - cz;
        double wZ = dX * nX + dY * nY + dZ * nZ;
        double pX = dX - wZ * nX, pY = dY - wZ * nY, pZ = dZ - wZ * nZ;

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

        // 如果超出合理外延直接视为病态退化，触发 Rigid Follow，绝不让它乱长尖刺！
        if (Float.isNaN(wX) || Float.isNaN(wY) || Float.isNaN((float)wZ) ||
                Math.abs(wX) > 15.0f || Math.abs(wY) > 15.0f || Math.abs(wZ) > 15.0f) {
            return false;
        }

        float nwX = 0, nwY = 0, nwZ = 1;
        if (flex.vNormWeightX != null) {
            double normLen = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            if (normLen > 1e-5) {
                double inX = normX / normLen, inY = normY / normLen, inZ = normZ / normLen;
                double normWZ = inX * nX + inY * nY + inZ * nZ;
                double npX = inX - normWZ * nX, npY = inY - normWZ * nY, npZ = inZ - normWZ * nZ;
                double nd_u = npX * uX + npY * uY + npZ * uZ;
                double nd_v = npX * vX + npY * vY + npZ * vZ;
                nwX = (float) ((nd_u * v_v - nd_v * u_v) * invDet2D);
                nwY = (float) ((nd_v * u_u - nd_u * u_v) * invDet2D);
                nwZ = (float) normWZ;
            }
            flex.vNormWeightX[ptr] = nwX;
            flex.vNormWeightY[ptr] = nwY;
            flex.vNormWeightZ[ptr] = nwZ;
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
            flex.vNormWeightX[ptr] = (float)(nLen > 1e-5 ? normX / nLen : 0);
            flex.vNormWeightY[ptr] = (float)(nLen > 1e-5 ? normY / nLen : 1);
            flex.vNormWeightZ[ptr] = (float)(nLen > 1e-5 ? normZ / nLen : 0);
        }
    }
}