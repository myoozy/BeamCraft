package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.physics.FlexbodyContainer;
import me.mzy.beamcraft.physics.NodeContainer;
import java.util.ArrayList;
import java.util.List;

/**
 * 客户端专属：软体视觉空间绑定器
 * 负责从只读的 DAE 缓存中拉取几何体，匹配物理节点，并写入通用数据容器
 */
public class FlexbodyBindingUtil {

    public static void performBinding(FlexbodyContainer flex, NodeContainer nodes) {
        if (flex.isSkinningBound || flex.meshCount == 0) {
            BeamCraft.LOGGER.info(flex.toString()+": isSkinningBound="+flex.isSkinningBound+"; meshCount="+flex.meshCount);
            return;
        }

        // 1. 预统计整车所需的总顶点数
        int totalVerts = 0;
        for (int m = 0; m < flex.meshCount; m++) {
            // 强制组合 "真实车名:网格名"
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);

            // 贴心保底：如果专属包里没找到，去公共 common 包里找一下通用部件（比如轮胎）
            if (geom == null) {
                geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            }

            if (geom != null) {
                totalVerts += geom.vertexCount;
            } else {
                // 开启高亮警告，彻底消灭隐形丢失
                System.err.println("⚠️ 警告：无法在显存中定位视觉网格快照 -> " + scopedKey);
            }
        }

        // 2. 跨端调用分配内存
        flex.allocateSkinningBuffers(totalVerts);
        if (totalVerts == 0) {
            flex.isSkinningBound = true;
            return;
        }

        int ptr = 0;

        // 3. 逐个网格零件遍历绑定
        for (int m = 0; m < flex.meshCount; m++) {
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            // 提取当前网格在 JBeam 中指定的 CSR 节点池
            List<Integer> allowedNodes = new ArrayList<>();
            if (flex.targetGroups[m] != null) {
                for (String gName : flex.targetGroups[m]) {
                    Integer gId = flex.groupNameToId.get(gName);
                    if (gId != null) {
                        int start = flex.groupNodeOffsets[gId];
                        int count = flex.groupNodeCounts[gId];
                        for (int i = 0; i < count; i++) {
                            allowedNodes.add(flex.flatGroupNodes[start + i]);
                        }
                    }
                }
            }

            // 兜底防御：如果 JBeam 没写 group，默认允许全车节点搜索
            if (allowedNodes.isEmpty()) {
                for (int i = 0; i < nodes.count; i++) allowedNodes.add(i);
            }

            int poolSize = allowedNodes.size();
            float[] pos = geom.positions;

            // JBeam 零件自身定义的初始固定偏移
            double partOffsetX = flex.posX[m];
            double partOffsetY = flex.posY[m];
            double partOffsetZ = flex.posZ[m];

            // 4. 遍历该网格的每一个原始顶点
            for (int v = 0; v < geom.vertexCount; v++) {
                double vx = pos[v * 3]     + partOffsetX;
                double vy = pos[v * 3 + 1] + partOffsetY;
                double vz = pos[v * 3 + 2] + partOffsetZ;

                // --- 步骤 A：极速寻找最近的 Center 节点 ---
                int bestC = allowedNodes.get(0);
                double minDistSq = Double.MAX_VALUE;
                for (int i = 0; i < poolSize; i++) {
                    int n = allowedNodes.get(i);
                    double dx = vx - nodes.posX[n];
                    double dy = vy - nodes.posY[n];
                    double dz = vz - nodes.posZ[n];
                    double dSq = dx * dx + dy * dy + dz * dz;
                    if (dSq < minDistSq) {
                        minDistSq = dSq;
                        bestC = n;
                    }
                }

                // --- 步骤 B：寻找构成正交基底的 Vx 和 Vy 节点 ---
                int bestX = bestC, bestY = bestC;
                double bestScore = -Double.MAX_VALUE;

                double cx = nodes.posX[bestC];
                double cy = nodes.posY[bestC];
                double cz = nodes.posZ[bestC];

                for (int i = 0; i < poolSize; i++) {
                    int nx = allowedNodes.get(i);
                    if (nx == bestC) continue;

                    double dxx = nodes.posX[nx] - cx;
                    double dxy = nodes.posY[nx] - cy;
                    double dxz = nodes.posZ[nx] - cz;
                    double lenXSq = dxx * dxx + dxy * dxy + dxz * dxz;
                    if (lenXSq < 1e-6) continue;
                    double invLenX = 1.0 / Math.sqrt(lenXSq);

                    for (int j = i + 1; j < poolSize; j++) {
                        int ny = allowedNodes.get(j);
                        if (ny == bestC) continue;

                        double dyx = nodes.posX[ny] - cx;
                        double dyy = nodes.posY[ny] - cy;
                        double dyz = nodes.posZ[ny] - cz;
                        double lenYSq = dyx * dyx + dyy * dyy + dyz * dyz;
                        if (lenYSq < 1e-6) continue;

                        // 计算夹角余弦值 (点积)
                        double dot = (dxx * dyx + dxy * dyy + dxz * dyz) * invLenX * (1.0 / Math.sqrt(lenYSq));

                        // 评分规则：极致惩罚共线 (dot 接近 1)，同时略微偏好距离较近的节点
                        double score = (1.0 - Math.abs(dot)) - (lenXSq + lenYSq) * 0.05;

                        if (score > bestScore) {
                            bestScore = score;
                            bestX = nx;
                            bestY = ny;
                        }
                    }
                }

                // --- 步骤 C：克莱姆法则求解逆变空间权重 ---
                double dxx = nodes.posX[bestX] - cx, dxy = nodes.posY[bestX] - cy, dxz = nodes.posZ[bestX] - cz;
                double dyx = nodes.posX[bestY] - cx, dyy = nodes.posY[bestY] - cy, dyz = nodes.posZ[bestY] - cz;

                // 默认采用叉乘构造正交的虚拟 Z 轴，完美契合车身薄壳蒙皮
                double dzx = dxy * dyz - dxz * dyy;
                double dzy = dxz * dyx - dxx * dyz;
                double dzz = dxx * dyy - dxy * dyx;

                double tx = vx - cx, ty = vy - cy, tz = vz - cz;

                // 3x3 矩阵行列式
                double det = dxx * (dyy * dzz - dyz * dzy) - dyx * (dxy * dzz - dxz * dzy) + dzx * (dxy * dyz - dxz * dyy);

                float wX = 0, wY = 0, wZ = 0;
                if (Math.abs(det) > 1e-9) {
                    double invDet = 1.0 / det;
                    wX = (float) ((tx * (dyy * dzz - dyz * dzy) - dyx * (ty * dzz - tz * dzy) + dzx * (ty * dyz - tz * dyy)) * invDet);
                    wY = (float) ((dxx * (ty * dzz - tz * dzy) - tx * (dxy * dzz - dxz * dzy) + dzx * (dxy * tz - dxz * ty)) * invDet);
                    wZ = (float) ((dxx * (dyy * tz - dyz * ty) - dyx * (dxy * tz - dxz * ty) + tx * (dxy * dyz - dxz * dyy)) * invDet);
                }

                // 写入实例的紧凑展平内存
                flex.vCenterNode[ptr] = bestC;
                flex.vVxNode[ptr]     = bestX;
                flex.vVyNode[ptr]     = bestY;
                flex.vWeightX[ptr]    = wX;
                flex.vWeightY[ptr]    = wY;
                flex.vWeightZ[ptr]    = wZ;
                flex.vUseCrossZ[ptr]  = true;

                // 写入 UV 坐标保底
                flex.uvU[ptr] = 0.0f;
                flex.uvV[ptr] = 0.0f;

                ptr++;
            }
        }

        // 5. 锁定当前实例的绑定状态
        flex.isSkinningBound = true;
    }
}