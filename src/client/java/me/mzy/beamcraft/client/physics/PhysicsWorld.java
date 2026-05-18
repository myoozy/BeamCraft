package me.mzy.beamcraft.client.physics;

import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import me.mzy.beamcraft.network.VehicleSyncPayload;

import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;

/**
 * Core physical world controller for beam-based vehicle simulation
 * Manages nodes, beams, collision caching and physics integration
 */
public class PhysicsWorld {
    public static final double GRAVITY = -9.81;
    public static final double SOUND_SPEED = 340;
    public static final double BLOCK_REBOUND = 0.0;
    public static final double BLOCK_FRICTION = 1.0;
    public static final double METAL_PLASTIC_FLOW_RATE = 10.0;
    public static final double KINDA_SMALL_NUMBER = 1e-8;
    public static final double KINDA_BIG_NUMBER = 1e8;
    public static final int MAX_AABB_SIZE = 5;
    public static final double invPhysicsDT = 2000;

    // 用于处理和mc世界的碰撞
    public final VoxelSnapshot voxelSnapshot = new VoxelSnapshot();
    BlockPos.Mutable mutablePos = new BlockPos.Mutable();

    // 处理软体之间的碰撞
    public final DynamicAxisSweep globalSap = new DynamicAxisSweep();
    public final SoftBodyCollisionManager collisionManager = new SoftBodyCollisionManager();

    private int nextVehicleId = 0;     // 永远递增的全局车牌号 (绝不重复)

    public final java.util.List<SoftBodyVehicle> vehicles = new java.util.concurrent.CopyOnWriteArrayList<>();

    public PhysicsWorld() {
        // Empty constructor, data will be injected by JBeam parser
    }

    public void addVehicle(SoftBodyVehicle vehicle) {
        if (vehicle == null || vehicles.contains(vehicle)) return;

        vehicle.vehicleId = nextVehicleId++;
        vehicles.add(vehicle);
    }

    /**
     * 安全地将车辆从物理世界中移除并销毁
     */
    public void removeVehicle(SoftBodyVehicle vehicle) {
        if (vehicle == null || !vehicles.contains(vehicle)) return;

        // 1. 从活跃列表中移除
        // 由于你的 vehicles 是 CopyOnWriteArrayList，这一步天生线程安全！
        // 哪怕物理线程正在 foreach 遍历所有车，移除操作也不会导致崩溃。
        vehicles.remove(vehicle);

        // 2. 释放该车辆占用的底层 SoA 数组内存
        vehicle.clear();

        System.out.println("🚗 Vehicle removed safely. ID: " + vehicle.vehicleId);

        // 注意：我们不需要去清理 SAP 树或者(SoftBody)CollisionManager！
        // 因为 SAP 树在宽阶段会调用 globalSap.clear() 并完全根据当前的 vehicles 列表重建。
        // CollisionManager也会在宽阶段自动 clearContacts()。
        // 这就是每次重建宽阶段架构的最大优势：没有状态残留！
    }

    public void clear() {
        vehicles.clear();
        collisionManager.clearContacts(); // 清理碰撞管理器
        System.out.println("🧹 Physics world data cleared");
    }

    /**
     * Main physics update loop
     */
    public void step(World mcWorld, double dt, double[] lastPhycisMsDetail) {
        int subSteps = (int)Math.ceil(dt * invPhysicsDT);
        double subDt = dt / subSteps;
        int broadphaseRate = 10;

        long t1 = System.nanoTime();

        voxelSnapshot.clear();
        // 让每辆车自己去扫描周围的方块并写入全局 snapshot
        for (SoftBodyVehicle vehicle : vehicles) {
            vehicle.cacheEntityLocation();
            vehicle.updateVoxelSnapshot(mcWorld, voxelSnapshot, mutablePos, dt);
        }

        long t2 = System.nanoTime();
        double mcWorldScanMs = (t2 - t1) / 1_000_000.0;
        double internalForceMs = 0.0, globalSAPMs = 0.0, dyeCollisionMs = 0.0, softCollisionMs = 0.0, mcCollisionMs = 0.0;

        for (int s = 0; s < subSteps; s++) {

            long ti1 = System.nanoTime();

            // 1. 算内力并积分预测坐标【完全独立并行】
            vehicles.parallelStream().forEach(vehicle -> {
                vehicle.solveInternalForces(subDt);
            });

            long ti2 = System.nanoTime();
            internalForceMs += (ti2 - ti1) / 1_000_000.0;

            // 2. 宽阶段：降频更新树与接触缓存
            if (s % broadphaseRate == 0) {
                long tii1 = System.nanoTime();
                globalSap.clear();

                // ★ 动态内存紧凑 (Dynamic Packing) ★
                // 每次宽阶段前，重新为存活的车辆分配紧凑的全局 ID 偏移量。
                // 这样无论玩家怎么删除和生成车辆，内存永远是 0 空隙
                int activeOffset = 0;
                for (SoftBodyVehicle vehicle : vehicles) {
                    vehicle.globalNodeOffset = activeOffset;
                    activeOffset += vehicle.nodes.count;

                    globalSap.insertNodes(vehicle); // 顺便把节点插入 SAP 树
                }

                // 如果当前所有车的节点总数超过了管理器的上限，可以在这里加个警告甚至扩容逻辑
                // if (activeOffset >= SoftBodyCollisionManager.MAX_GLOBAL_NODES) { ... }

                globalSap.updateAndSort();

                long tii2 = System.nanoTime();
                globalSAPMs += (tii2 - tii1) / 1_000_000.0;

                collisionManager.clearContacts();

                // 并行查树
                vehicles.parallelStream().forEach(vehicle -> {
                    vehicle.generateCollisionCandidates(globalSap, collisionManager, subDt * broadphaseRate);
                });

                // 极速图染色分批
                collisionManager.buildAndColorBatches();

                long tii3 = System.nanoTime();
                dyeCollisionMs += (tii3 - tii2) / 1_000_000.0;
            }

            long ti3 = System.nanoTime();

            // 3. 窄阶段：多车/单车智能动态并行求解
            solveCachedContacts(subDt);

            long ti4 = System.nanoTime();
            softCollisionMs += (ti4 - ti3) / 1_000_000.0;

            // 4. 环境方块碰撞【车辆级别完全独立并行！】
            vehicles.parallelStream().forEach(vehicle -> {
                vehicle.solveEnvironmentCollisions(voxelSnapshot, subDt);
            });

            long ti5 = System.nanoTime();
            mcCollisionMs += (ti5 - ti4) / 1_000_000.0;
        }

        long t3 = System.nanoTime();
        vehicles.parallelStream().forEach(vehicle -> {
            vehicle.updateLocalCOMCache();
            vehicle.updateBeamPrecompression(dt);
            vehicle.nodes.writeRenderBuffer();
        });
        long t4 = System.nanoTime();
        double postUpdateMs = (t4 - t3) / 1_000_000.0;

        for (SoftBodyVehicle vehicle : vehicles) {
            vehicle.updateEntityLocation();
            if (vehicle.parentEntity != null && ClientPlayNetworking.canSend(VehicleSyncPayload.ID)) {
                ClientPlayNetworking.send(new VehicleSyncPayload(
                        vehicle.parentEntity.getId(),
                        vehicle.parentEntity.getX(),
                        vehicle.parentEntity.getY(),
                        vehicle.parentEntity.getZ(),
                        vehicle.parentEntity.getYaw()
                ));
            }
        }
        long t5 = System.nanoTime();
        double moveEntityMs = (t5 - t4) / 1_000_000.0;

        double totalMs = (t5 - t1) / 1_000_000.0;
        lastPhycisMsDetail[0] = totalMs;
        lastPhycisMsDetail[1] = mcWorldScanMs;
        lastPhycisMsDetail[2] = internalForceMs;
        lastPhycisMsDetail[3] = globalSAPMs;
        lastPhycisMsDetail[4] = dyeCollisionMs;
        lastPhycisMsDetail[5] = softCollisionMs;
        lastPhycisMsDetail[6] = mcCollisionMs;
        lastPhycisMsDetail[7] = postUpdateMs;
        lastPhycisMsDetail[8] = moveEntityMs;
    }

    /**
     * 智能动态多线程调度器：根据批次大小动态决定是否开启并行
     */
    private void solveCachedContacts(double dt) {
        // 动态阈值：你可以根据测试结果调整。
        final int PARALLEL_THRESHOLD = 1024;

        for (int b = 0; b < collisionManager.activeBatchCount; b++) {
            int currentBatchSize = collisionManager.batchSize[b];
            if (currentBatchSize == 0) continue;

            final int batchIndex = b;

            // 智能分发逻辑
            if (currentBatchSize < PARALLEL_THRESHOLD) {
                // 【单线程快车道】点数太少，唤醒线程池反而亏本，直接主线程纯数学速通！
                for (int idx = 0; idx < currentBatchSize; idx++) {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    resolveSingleContact(contactId, dt);
                }
            } else {
                // 【多核重火力网】点数巨大（严重连环相撞），放开枷锁，多线程火力全开！
                java.util.stream.IntStream.range(0, currentBatchSize).parallel().forEach(idx -> {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    resolveSingleContact(contactId, dt);
                });
            }
        }
    }

    /**
     * 被提取出来的纯数学 PBD 解算器
     */
    private void resolveSingleContact(int contactId, double dt) {
        // 物理参数（可根据车辆材质调整）
        double THICKNESS = 0.01;
        double PBD_RELAXATION = 1;
        double MAX_POS_PUSH = 0.1;
        double RESTITUTION = 0.0;
        double invDt = 1.0 / dt;

        SoftBodyVehicle nVeh = collisionManager.contactNodeVeh[contactId];
        int nHit = collisionManager.contactNodeId[contactId];

        SoftBodyVehicle tVeh = collisionManager.contactTriVeh[contactId];
        int nA = collisionManager.contactTriA[contactId];
        int nB = collisionManager.contactTriB[contactId];
        int nC = collisionManager.contactTriC[contactId];

        // 1. 三角形坐标
        double ax = tVeh.entityX + tVeh.nodes.posX[nA], ay = tVeh.entityY + tVeh.nodes.posY[nA], az = tVeh.entityZ + tVeh.nodes.posZ[nA];
        double bx = tVeh.entityX + tVeh.nodes.posX[nB], by = tVeh.entityY + tVeh.nodes.posY[nB], bz = tVeh.entityZ + tVeh.nodes.posZ[nB];
        double cx = tVeh.entityX + tVeh.nodes.posX[nC], cy = tVeh.entityY + tVeh.nodes.posY[nC], cz = tVeh.entityZ + tVeh.nodes.posZ[nC];

        // 节点当前位置（世界坐标）
        double pX = nVeh.entityX + nVeh.nodes.posX[nHit];
        double pY = nVeh.entityY + nVeh.nodes.posY[nHit];
        double pZ = nVeh.entityZ + nVeh.nodes.posZ[nHit];

        // AABB 快速过滤（带厚度）
        double minX = Math.min(ax, Math.min(bx, cx)) - THICKNESS;
        double maxX = Math.max(ax, Math.max(bx, cx)) + THICKNESS;
        double minY = Math.min(ay, Math.min(by, cy)) - THICKNESS;
        double maxY = Math.max(ay, Math.max(by, cy)) + THICKNESS;
        double minZ = Math.min(az, Math.min(bz, cz)) - THICKNESS;
        double maxZ = Math.max(az, Math.max(bz, cz)) + THICKNESS;
        if (pX < minX || pX > maxX || pY < minY || pY > maxY || pZ < minZ || pZ > maxZ) return;

        // 三角形法线
        double abx = bx - ax, aby = by - ay, abz = bz - az;
        double acx = cx - ax, acy = cy - ay, acz = cz - az;
        double nx = aby * acz - abz * acy;
        double ny = abz * acx - abx * acz;
        double nz = abx * acy - aby * acx;

        double nLenSq = nx*nx + ny*ny + nz*nz;
        if (nLenSq < 1e-8) return;
        double invNLen = 1.0 / Math.sqrt(nLenSq);
        nx *= invNLen; ny *= invNLen; nz *= invNLen;

        // 预计算重心坐标用数据
        double d00 = abx*abx + aby*aby + abz*abz;
        double d01 = abx*acx + aby*acy + abz*acz;
        double d11 = acx*acx + acy*acy + acz*acz;
        double denom = d00 * d11 - d01 * d01;
        if (denom < 1e-8) return;
        double invDenom = 1.0 / denom;

        // 2. CCD: 距离预测
        double apx = pX - ax, apy = pY - ay, apz = pZ - az;
        double distCurr = apx * nx + apy * ny + apz * nz;

        double triVx = (tVeh.nodes.velX[nA] + tVeh.nodes.velX[nB] + tVeh.nodes.velX[nC]) * 0.33333333;
        double triVy = (tVeh.nodes.velY[nA] + tVeh.nodes.velY[nB] + tVeh.nodes.velY[nC]) * 0.33333333;
        double triVz = (tVeh.nodes.velZ[nA] + tVeh.nodes.velZ[nB] + tVeh.nodes.velZ[nC]) * 0.33333333;

        double approxRelV = (nVeh.nodes.velX[nHit] - triVx) * nx +
                (nVeh.nodes.velY[nHit] - triVy) * ny +
                (nVeh.nodes.velZ[nHit] - triVz) * nz;

        double distPrev = distCurr - approxRelV * dt;

        // 3. 确定碰撞侧并计算穿透
        double pushDir = (distPrev > 0) ? 1.0 : -1.0;
        double signedDist = distCurr * pushDir;
        double penetration = THICKNESS - signedDist;
        if (penetration <= 0) return;

        // 4. 重心坐标
        double ppx = apx - distCurr * nx;
        double ppy = apy - distCurr * ny;
        double ppz = apz - distCurr * nz;

        double d20 = ppx*abx + ppy*aby + ppz*abz;
        double d21 = ppx*acx + ppy*acy + ppz*acz;

        double wB = (d11 * d20 - d01 * d21) * invDenom;
        double wC = (d00 * d21 - d01 * d20) * invDenom;
        double wA = 1.0 - wB - wC;

        double TOLERANCE = -0.01;
        if (!(wA >= TOLERANCE && wB >= TOLERANCE && wC >= TOLERANCE)) return;

        double effNx = nx * pushDir, effNy = ny * pushDir, effNz = nz * pushDir;

        double massNode = nVeh.nodes.mass[nHit];
        double massA = tVeh.nodes.mass[nA], massB = tVeh.nodes.mass[nB], massC = tVeh.nodes.mass[nC];

        double wTotal = (1.0 / massNode) + (wA*wA / massA) + (wB*wB / massB) + (wC*wC / massC);
        if (wTotal < 1e-8) return;
        double invWTotal = 1.0 / wTotal;

        // 5. 位置修正 (PBD 硬约束)
        double pushAmount = penetration * PBD_RELAXATION;
        if (pushAmount > MAX_POS_PUSH) pushAmount = MAX_POS_PUSH;
        double posImpulse = pushAmount * invWTotal;
        double dpX = posImpulse * effNx;
        double dpY = posImpulse * effNy;
        double dpZ = posImpulse * effNz;

        // 6. 速度修正（法向弹性 + 切向库仑摩擦）
        double dvX = 0, dvY = 0, dvZ = 0;

        double exactTriVx = wA * tVeh.nodes.velX[nA] + wB * tVeh.nodes.velX[nB] + wC * tVeh.nodes.velX[nC];
        double exactTriVy = wA * tVeh.nodes.velY[nA] + wB * tVeh.nodes.velY[nB] + wC * tVeh.nodes.velY[nC];
        double exactTriVz = wA * tVeh.nodes.velZ[nA] + wB * tVeh.nodes.velZ[nB] + wC * tVeh.nodes.velZ[nC];

        double relVx = nVeh.nodes.velX[nHit] - exactTriVx;
        double relVy = nVeh.nodes.velY[nHit] - exactTriVy;
        double relVz = nVeh.nodes.velZ[nHit] - exactTriVz;

        double approachSpeed = relVx * effNx + relVy * effNy + relVz * effNz;
        double jn = 0; // 法向冲量大小

        // 6.1 法向弹性冲量
        if (approachSpeed < 0) {
            double deltaRelVel = -(1 + RESTITUTION) * approachSpeed;
            jn = deltaRelVel * invWTotal;
            dvX += jn * effNx;
            dvY += jn * effNy;
            dvZ += jn * effNz;
        }

        // 6.2 估算总法向支撑力
        // 即便速度为 0 (静止靠在斜坡上)，如果有位置推挤，说明重力在挤压，也要产生摩擦力支撑！
        double equivalentJn = jn;
        if (penetration > 0 && approachSpeed >= -1e-4) {
            equivalentJn += (pushAmount * invDt) * invWTotal;
        }

        // 6.3 库仑摩擦力模型 (基于重心坐标插值与乘积混合法则)
        if (equivalentJn > 0) {

            // 计算切向相对速度
            double tangentVx = relVx - (approachSpeed * effNx);
            double tangentVy = relVy - (approachSpeed * effNy);
            double tangentVz = relVz - (approachSpeed * effNz);

            double vtLen = Math.sqrt(tangentVx * tangentVx + tangentVy * tangentVy + tangentVz * tangentVz);

            // 1. 🚀 利用现成的重心坐标 (wA, wB, wC) 精准插值出目标三角形表面的基础摩擦力
            double tri_mu_s = wA * tVeh.nodes.friction[nA] + wB * tVeh.nodes.friction[nB] + wC * tVeh.nodes.friction[nC];
            double tri_mu_k = wA * tVeh.nodes.slidingFriction[nA] + wB * tVeh.nodes.slidingFriction[nB] + wC * tVeh.nodes.slidingFriction[nC];

            double mu_s;
            double mu_k;

            // 2. 🚀 O(1) 检查发生碰撞的节点是否为轮胎
            int wIdx = nVeh.nodes.wheelId[nHit];

            if (wIdx != -1) {
                // ==========================================================
                // 🚗 轮胎节点：启用 BeamNG 高级载荷敏感与速度过渡乘子模型
                // ==========================================================
                double staticBase  = nVeh.wheels.frictionCoef[wIdx];
                double slidingBase = nVeh.wheels.slidingFrictionCoef[wIdx];
                double noLoad      = nVeh.wheels.noLoadCoef[wIdx];
                double fullLoad    = nVeh.wheels.fullLoadCoef[wIdx];
                double slope       = nVeh.wheels.loadSensitivitySlope[wIdx];

                // --- A. 将当前累积的等效冲量转化为物理载荷力 (牛顿) ---
                double equivalentLoadN = equivalentJn * invDt;

                // --- B. 计算载荷敏感度衰减 (Load Sensitivity) ---
                double loadFactor = noLoad - (slope * equivalentLoadN);
                if (loadFactor < fullLoad) loadFactor = fullLoad; // 触底保护

                // --- C. 斯特里贝克速度过渡曲线 (Stribeck Curve) ---
                double stribeckVel = nVeh.wheels.stribeckVelMult[wIdx];
                double exponent    = nVeh.wheels.stribeckExponent[wIdx];

                double speedFactor = 1.0;
                if (vtLen > 1e-4 && stribeckVel > 1e-4) {
                    double velRatio = vtLen / stribeckVel;
                    speedFactor = Math.exp(-Math.pow(velRatio, exponent));
                }

                // 动态滑动乘子插值
                double dynamicMuMultiplier = slidingBase + (staticBase - slidingBase) * speedFactor;
                double treadCoef = nVeh.wheels.treadCoef[wIdx];

                // 🚀 核心物理混合：轮胎乘子 × 目标表面插值基础值
                mu_s = (staticBase * loadFactor * treadCoef)  * tri_mu_s;
                mu_k = (dynamicMuMultiplier * loadFactor * treadCoef) * tri_mu_k;

            } else {
                // ==========================================================
                // 🛡️ 普通车身节点：采用标准乘积法则兜底
                // ==========================================================
                // 普通金属/塑料撞击，同样严格遵循乘积法则而不是算术平均
                mu_s = nVeh.nodes.friction[nHit] * tri_mu_s;
                mu_k = nVeh.nodes.slidingFriction[nHit] * tri_mu_k;
            }

            if (vtLen > 1e-8) {
                // 完全抵消切向相对速度所需的冲量
                double jtMax = vtLen * invWTotal;

                double frictionImpulse;
                if (jtMax <= mu_s * equivalentJn) {
                    // 【静摩擦】：所需冲量没有突破极限，彻底消除切向速度（咬死）
                    frictionImpulse = jtMax;
                } else {
                    // 【滑动摩擦】：突破极限发生打滑，提供恒定阻力
                    frictionImpulse = mu_k * equivalentJn;
                }

                // 切向单位向量
                double invVtLen = 1.0 / vtLen;
                double tDirX = tangentVx * invVtLen;
                double tDirY = tangentVy * invVtLen;
                double tDirZ = tangentVz * invVtLen;

                // 施加摩擦冲量
                dvX -= frictionImpulse * tDirX;
                dvY -= frictionImpulse * tDirY;
                dvZ -= frictionImpulse * tDirZ;
            }
        }

        // 7. 应用位置和速度增量
        nVeh.applyPositionAndVelocityDeltaUnSafe(nHit,
                dpX / massNode, dpY / massNode, dpZ / massNode,
                dvX / massNode, dvY / massNode, dvZ / massNode);

        tVeh.applyPositionAndVelocityDeltaUnSafe(nA,
                -dpX * (wA / massA), -dpY * (wA / massA), -dpZ * (wA / massA),
                -dvX * (wA / massA), -dvY * (wA / massA), -dvZ * (wA / massA));

        tVeh.applyPositionAndVelocityDeltaUnSafe(nB,
                -dpX * (wB / massB), -dpY * (wB / massB), -dpZ * (wB / massB),
                -dvX * (wB / massB), -dvY * (wB / massB), -dvZ * (wB / massB));

        tVeh.applyPositionAndVelocityDeltaUnSafe(nC,
                -dpX * (wC / massC), -dpY * (wC / massC), -dpZ * (wC / massC),
                -dvX * (wC / massC), -dvY * (wC / massC), -dvZ * (wC / massC));
    }
}