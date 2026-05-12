package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import java.util.ArrayList;
import java.util.List;

public class FlexbodyBindingUtil {

    public static void performBinding(FlexbodyContainer flex, NodeContainer nodes) {
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
            // 提取 JBeam 配置的相对初始插槽平移
            double px = flex.posX[m], py = flex.posY[m], pz = flex.posZ[m];

            for (int v = 0; v < geom.vertexCount; v++) {
                // 此时 pos 数组内的数据已经是完美转换后的 Minecraft 绝对坐标
                // 仅需直接叠加 JBeam 局部偏移量即可，彻底剔除二次翻转代码
                double vx = pos[v * 3] + px;
                double vy = pos[v * 3 + 1] + py;
                double vz = pos[v * 3 + 2] + pz;

                flex.skinnedPosX[ptr] = (float) vx;
                flex.skinnedPosY[ptr] = (float) vy;
                flex.skinnedPosZ[ptr] = (float) vz;

                // 注入蒙皮基底扫描，带有主池退化保护
                boolean success = tryCalculateWeights(flex, nodes, ptr, vx, vy, vz, primaryPool);
                if (!success && primaryPool != globalPool) {
                    tryCalculateWeights(flex, nodes, ptr, vx, vy, vz, globalPool);
                }

                if (uvs != null && v * 2 + 1 < uvs.length) {
                    flex.uvU[ptr] = uvs[v * 2];
                    flex.uvV[ptr] = uvs[v * 2 + 1];
                }
                ptr++;
            }
        }
        flex.isSkinningBound = true;
    }

    private static boolean tryCalculateWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                               double vx, double vy, double vz, List<Integer> pool) {
        int poolSize = pool.size();
        if (poolSize < 1) return false;

        int bestC = pool.get(0);
        double minDistSq = Double.MAX_VALUE;
        for (int i = 0; i < poolSize; i++) {
            int n = pool.get(i);
            double dx = vx - nodes.posX[n], dy = vy - nodes.posY[n], dz = vz - nodes.posZ[n];
            double dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq) {
                minDistSq = dSq;
                bestC = n;
            }
        }

        if (poolSize < 3) return false;

        int bestX = bestC, bestY = bestC;
        double bestScore = -Double.MAX_VALUE;
        double cx = nodes.posX[bestC], cy = nodes.posY[bestC], cz = nodes.posZ[bestC];

        for (int i = 0; i < poolSize; i++) {
            int nx = pool.get(i);
            if (nx == bestC) continue;
            double dxx = nodes.posX[nx] - cx, dxy = nodes.posY[nx] - cy, dxz = nodes.posZ[nx] - cz;
            double lenXSq = dxx * dxx + dxy * dxy + dxz * dxz;
            if (lenXSq < 1e-5) continue;
            double invLenX = 1.0 / Math.sqrt(lenXSq);

            for (int j = i + 1; j < poolSize; j++) {
                int ny = pool.get(j);
                if (ny == bestC) continue;
                double dyx = nodes.posX[ny] - cx, dyy = nodes.posY[ny] - cy, dyz = nodes.posZ[ny] - cz;
                double lenYSq = dyx * dyx + dyy * dyy + dyz * dyz;
                if (lenYSq < 1e-5) continue;

                double dot = (dxx * dyx + dxy * dyy + dxz * dyz) * invLenX * (1.0 / Math.sqrt(lenYSq));
                if (Math.abs(dot) > 0.75) continue;

                double score = (1.0 - Math.abs(dot)) - (lenXSq + lenYSq) * 0.05;
                if (score > bestScore) {
                    bestScore = score;
                    bestX = nx; bestY = ny;
                }
            }
        }

        if (bestX == bestC || bestY == bestC) return false;

        double dxx = nodes.posX[bestX] - cx, dxy = nodes.posY[bestX] - cy, dxz = nodes.posZ[bestX] - cz;
        double dyx = nodes.posX[bestY] - cx, dyy = nodes.posY[bestY] - cy, dyz = nodes.posZ[bestY] - cz;

        double dzx = dxy * dyz - dxz * dyy;
        double dzy = dxz * dyx - dxx * dyz;
        double dzz = dxx * dyy - dxy * dyx;

        double lenZ = Math.sqrt(dzx * dzx + dzy * dzy + dzz * dzz);
        if (lenZ > 1e-7) {
            double invZ = 1.0 / lenZ;
            dzx *= invZ; dzy *= invZ; dzz *= invZ;
        } else return false;

        double tx = vx - cx, ty = vy - cy, tz = vz - cz;
        double det = dxx * (dyy * dzz - dyz * dzy) - dyx * (dxy * dzz - dxz * dzy) + dzx * (dxy * dyz - dxz * dyy);

        if (Math.abs(det) < 1e-7) return false;

        double invDet = 1.0 / det;
        flex.vCenterNode[ptr] = bestC;
        flex.vVxNode[ptr]     = bestX;
        flex.vVyNode[ptr]     = bestY;
        flex.vWeightX[ptr]    = (float) ((tx * (dyy * dzz - dyz * dzy) - dyx * (ty * dzz - tz * dzy) + dzx * (ty * dyz - tz * dyy)) * invDet);
        flex.vWeightY[ptr]    = (float) ((dxx * (ty * dzz - tz * dzy) - tx * (dxy * dzz - dxz * dzy) + dzx * (dxy * tz - dxz * ty)) * invDet);
        flex.vWeightZ[ptr]    = (float) ((dxx * (dyy * tz - dyz * ty) - dyx * (dxy * tz - dxz * ty) + tx * (dxy * dyz - dxz * dyy)) * invDet);
        flex.vUseCrossZ[ptr]  = true;
        return true;
    }
}