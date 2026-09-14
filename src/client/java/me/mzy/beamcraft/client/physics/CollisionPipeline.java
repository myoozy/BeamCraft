package me.mzy.beamcraft.client.physics;

import java.util.stream.IntStream;

/**
 * Collision pipeline shared by every vehicle in a {@link PhysicsWorld}.
 *
 * Owns the concrete algorithms that were previously spread across the physics
 * world scheduler and the per-vehicle class:
 * <ul>
 *     <li>soft-body contact candidate generation (triangle vs node, fed by the
 *     shared SAP broad-phase),</li>
 *     <li>the batched resolution of cached soft-body contacts (former
 *     {@code PhysicsWorld.solveCachedContacts}/{@code resolveSingleContact}), and</li>
 *     <li>Minecraft environment collision resolution from a {@link VoxelSnapshot}.</li>
 * </ul>
 *
 * The per-vehicle methods are invoked by {@code PhysicsWorld} inside its
 * per-vehicle parallel phases, so this pipeline must not keep per-vehicle
 * scratch state; the one per-vehicle sweep buffer remains owned by each
 * {@link SoftBodyVehicle}. No Minecraft world access happens here, which keeps
 * these calls safe on the pure-physics thread.
 */
public final class CollisionPipeline {
    private static final float SOFT_CONTACT_THICKNESS = 0.01f;
    private static final float SOFT_CONTACT_BARYCENTRIC_TOLERANCE = 0.01f;

    private final VoxelSnapshot voxelSnapshot;
    private final DynamicAxisSweep sap;
    private final SoftBodyCollisionManager collisionManager;

    public CollisionPipeline(VoxelSnapshot voxelSnapshot,
                             DynamicAxisSweep sap,
                             SoftBodyCollisionManager collisionManager) {
        this.voxelSnapshot = voxelSnapshot;
        this.sap = sap;
        this.collisionManager = collisionManager;
    }

    /**
     * Generates soft-body collision candidates for one vehicle by sweeping the
     * broad-phase over each collidable triangle's swept AABB.
     */
    public void generateCollisionCandidates(SoftBodyVehicle vehicle, double dtPredict) {
        double eX = vehicle.entityX, eY = vehicle.entityY, eZ = vehicle.entityZ;

        int sapHits = 0;
        int stored = 0;
        int dropped = 0;

        double BASE_MARGIN = 0.01;

        for (int i = 0; i < vehicle.triangles.count; i++) {
            if (!vehicle.triangles.collision[i] || vehicle.triangles.broken[i]) continue;

            int nA = vehicle.triangles.node1[i];
            int nB = vehicle.triangles.node2[i];
            int nC = vehicle.triangles.node3[i];

            double ax = eX + vehicle.nodes.posX[nA], ay = eY + vehicle.nodes.posY[nA], az = eZ + vehicle.nodes.posZ[nA];
            double bx = eX + vehicle.nodes.posX[nB], by = eY + vehicle.nodes.posY[nB], bz = eZ + vehicle.nodes.posZ[nB];
            double cx = eX + vehicle.nodes.posX[nC], cy = eY + vehicle.nodes.posY[nC], cz = eZ + vehicle.nodes.posZ[nC];

            // Union the current and predicted position of every vertex. Using
            // average triangle velocity misses rotation and deformation where
            // one vertex sweeps much farther than the centroid.
            double futureAx = ax + vehicle.nodes.velX[nA] * dtPredict;
            double futureAy = ay + vehicle.nodes.velY[nA] * dtPredict;
            double futureAz = az + vehicle.nodes.velZ[nA] * dtPredict;
            double futureBx = bx + vehicle.nodes.velX[nB] * dtPredict;
            double futureBy = by + vehicle.nodes.velY[nB] * dtPredict;
            double futureBz = bz + vehicle.nodes.velZ[nB] * dtPredict;
            double futureCx = cx + vehicle.nodes.velX[nC] * dtPredict;
            double futureCy = cy + vehicle.nodes.velY[nC] * dtPredict;
            double futureCz = cz + vehicle.nodes.velZ[nC] * dtPredict;

            double minX = Math.min(Math.min(ax, futureAx),
                    Math.min(Math.min(bx, futureBx), Math.min(cx, futureCx))) - BASE_MARGIN;
            double maxX = Math.max(Math.max(ax, futureAx),
                    Math.max(Math.max(bx, futureBx), Math.max(cx, futureCx))) + BASE_MARGIN;
            double minY = Math.min(Math.min(ay, futureAy),
                    Math.min(Math.min(by, futureBy), Math.min(cy, futureCy))) - BASE_MARGIN;
            double maxY = Math.max(Math.max(ay, futureAy),
                    Math.max(Math.max(by, futureBy), Math.max(cy, futureCy))) + BASE_MARGIN;
            double minZ = Math.min(Math.min(az, futureAz),
                    Math.min(Math.min(bz, futureBz), Math.min(cz, futureCz))) - BASE_MARGIN;
            double maxZ = Math.max(Math.max(az, futureAz),
                    Math.max(Math.max(bz, futureBz), Math.max(cz, futureCz))) + BASE_MARGIN;

            vehicle.sweepResultBuffer.clear();
            int rawHits = sap.queryCollisionNodesInAABB(
                    minX, minY, minZ, maxX, maxY, maxZ,
                    vehicle, nA, nB, nC, vehicle.triangles.partId[i],
                    vehicle.sweepResultBuffer);
            sapHits += rawHits;

            for (int k = 0; k < vehicle.sweepResultBuffer.count; k++) {
                SoftBodyVehicle hitVeh = vehicle.sweepResultBuffer.vehicles[k];
                int hitNodeId = vehicle.sweepResultBuffer.nodeIds[k];

                if (collisionManager.addContact(hitVeh, hitNodeId, vehicle, nA, nB, nC)) {
                    stored++;
                } else {
                    dropped++;
                }
            }
        }

        vehicle.collisionCandidateSapHits = sapHits;
        vehicle.collisionCandidateStored = stored;
        vehicle.collisionCandidateDropped = dropped;
    }

    /**
     * Resolve cached soft-body contacts batch by batch.
     */
    public long solveSoftBodyContacts(float dt) {
        final int PARALLEL_THRESHOLD = 1024;
        long narrowStats = 0L;

        for (int b = 0; b < collisionManager.activeBatchCount; b++) {
            int currentBatchSize = collisionManager.batchSize[b];
            if (currentBatchSize == 0) continue;

            final int batchIndex = b;

            if (!canSolveBatchInParallel(b, currentBatchSize, PARALLEL_THRESHOLD)) {
                for (int idx = 0; idx < currentBatchSize; idx++) {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    narrowStats += packNarrowResult(resolveSingleContact(contactId, dt));
                }
            } else {
                narrowStats += IntStream.range(0, currentBatchSize).parallel().mapToLong(idx -> {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    return packNarrowResult(resolveSingleContact(contactId, dt));
                }).sum();
            }
        }
        return narrowStats;
    }

    static boolean canSolveBatchInParallel(int batchIndex, int batchSize, int parallelThreshold) {
        return batchIndex != SoftBodyCollisionManager.OVERFLOW_BATCH_INDEX
                && batchSize >= parallelThreshold;
    }

    private static long packNarrowResult(int result) {
        long aabbPassed = result >= 1 ? 1L : 0L;
        long resolved = result >= 2 ? 1L : 0L;
        return (resolved << 32) | aabbPassed;
    }

    /**
     * Resolve one node-vs-triangle soft-body contact in triangle-local coordinates.
     */
    private int resolveSingleContact(int contactId, float dt) {
        final float THICKNESS = SOFT_CONTACT_THICKNESS;
        final float PBD_RELAXATION = 1.0f;
        final float MAX_POS_PUSH = 0.1f;
        final float RESTITUTION = 0.0f;
        final float invDt = 1.0f / dt;

        SoftBodyVehicle nVeh = collisionManager.contactNodeVeh[contactId];
        int nHit = collisionManager.contactNodeId[contactId];

        SoftBodyVehicle tVeh = collisionManager.contactTriVeh[contactId];
        int nA = collisionManager.contactTriA[contactId];
        int nB = collisionManager.contactTriB[contactId];
        int nC = collisionManager.contactTriC[contactId];

        float entityDeltaX = (float) (nVeh.entityX - tVeh.entityX);
        float entityDeltaY = (float) (nVeh.entityY - tVeh.entityY);
        float entityDeltaZ = (float) (nVeh.entityZ - tVeh.entityZ);

        float ax = tVeh.nodes.posX[nA], ay = tVeh.nodes.posY[nA], az = tVeh.nodes.posZ[nA];
        float bx = tVeh.nodes.posX[nB], by = tVeh.nodes.posY[nB], bz = tVeh.nodes.posZ[nB];
        float cx = tVeh.nodes.posX[nC], cy = tVeh.nodes.posY[nC], cz = tVeh.nodes.posZ[nC];

        float pX = entityDeltaX + nVeh.nodes.posX[nHit];
        float pY = entityDeltaY + nVeh.nodes.posY[nHit];
        float pZ = entityDeltaZ + nVeh.nodes.posZ[nHit];

        float minX = Math.min(ax, Math.min(bx, cx)) - THICKNESS;
        float maxX = Math.max(ax, Math.max(bx, cx)) + THICKNESS;
        float minY = Math.min(ay, Math.min(by, cy)) - THICKNESS;
        float maxY = Math.max(ay, Math.max(by, cy)) + THICKNESS;
        float minZ = Math.min(az, Math.min(bz, cz)) - THICKNESS;
        float maxZ = Math.max(az, Math.max(bz, cz)) + THICKNESS;
        if (pX < minX || pX > maxX || pY < minY || pY > maxY || pZ < minZ || pZ > maxZ) return 0;

        float abx = bx - ax, aby = by - ay, abz = bz - az;
        float acx = cx - ax, acy = cy - ay, acz = cz - az;
        float nx = aby * acz - abz * acy;
        float ny = abz * acx - abx * acz;
        float nz = abx * acy - aby * acx;

        float nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < PhysicsWorld.KINDA_SMALL_NUMBER) return 1;
        float invNLen = 1.0f / (float) Math.sqrt(nLenSq);
        nx *= invNLen; ny *= invNLen; nz *= invNLen;

        float apx = pX - ax, apy = pY - ay, apz = pZ - az;
        float distCurr = apx * nx + apy * ny + apz * nz;

        float triVx = (tVeh.nodes.velX[nA] + tVeh.nodes.velX[nB] + tVeh.nodes.velX[nC]) * 0.33333334f;
        float triVy = (tVeh.nodes.velY[nA] + tVeh.nodes.velY[nB] + tVeh.nodes.velY[nC]) * 0.33333334f;
        float triVz = (tVeh.nodes.velZ[nA] + tVeh.nodes.velZ[nB] + tVeh.nodes.velZ[nC]) * 0.33333334f;

        float approxRelV = (nVeh.nodes.velX[nHit] - triVx) * nx +
                (nVeh.nodes.velY[nHit] - triVy) * ny +
                (nVeh.nodes.velZ[nHit] - triVz) * nz;

        float distPrev = distCurr - approxRelV * dt;
        float pushDir = (distPrev > 0.0f) ? 1.0f : -1.0f;
        float signedDist = distCurr * pushDir;
        float penetration = THICKNESS - signedDist;
        if (penetration <= 0.0f) return 1;

        // Most cached candidates fail the plane-distance test above. Delay the
        // barycentric Gram matrix until a contact can actually penetrate.
        float d00 = abx * abx + aby * aby + abz * abz;
        float d01 = abx * acx + aby * acy + abz * acz;
        float d11 = acx * acx + acy * acy + acz * acz;
        float denom = d00 * d11 - d01 * d01;
        if (denom < PhysicsWorld.KINDA_SMALL_NUMBER) return 1;
        float invDenom = 1.0f / denom;

        float ppx = apx - distCurr * nx;
        float ppy = apy - distCurr * ny;
        float ppz = apz - distCurr * nz;

        float d20 = ppx * abx + ppy * aby + ppz * abz;
        float d21 = ppx * acx + ppy * acy + ppz * acz;

        float wB = (d11 * d20 - d01 * d21) * invDenom;
        float wC = (d00 * d21 - d01 * d20) * invDenom;
        float wA = 1.0f - wB - wC;

        final float TOLERANCE = -SOFT_CONTACT_BARYCENTRIC_TOLERANCE;
        if (!(wA >= TOLERANCE && wB >= TOLERANCE && wC >= TOLERANCE)) return 1;

        float effNx = nx * pushDir, effNy = ny * pushDir, effNz = nz * pushDir;

        float massNode = nVeh.nodes.mass[nHit];
        float massA = tVeh.nodes.mass[nA], massB = tVeh.nodes.mass[nB], massC = tVeh.nodes.mass[nC];

        float wTotal = (1.0f / massNode) + (wA * wA / massA) + (wB * wB / massB) + (wC * wC / massC);
        if (wTotal < PhysicsWorld.KINDA_SMALL_NUMBER) return 1;
        float invWTotal = 1.0f / wTotal;

        float pushAmount = penetration * PBD_RELAXATION;
        if (pushAmount > MAX_POS_PUSH) pushAmount = MAX_POS_PUSH;
        float posImpulse = pushAmount * invWTotal;
        float dpX = posImpulse * effNx;
        float dpY = posImpulse * effNy;
        float dpZ = posImpulse * effNz;

        float dvX = 0.0f, dvY = 0.0f, dvZ = 0.0f;

        float exactTriVx = wA * tVeh.nodes.velX[nA] + wB * tVeh.nodes.velX[nB] + wC * tVeh.nodes.velX[nC];
        float exactTriVy = wA * tVeh.nodes.velY[nA] + wB * tVeh.nodes.velY[nB] + wC * tVeh.nodes.velY[nC];
        float exactTriVz = wA * tVeh.nodes.velZ[nA] + wB * tVeh.nodes.velZ[nB] + wC * tVeh.nodes.velZ[nC];

        float relVx = nVeh.nodes.velX[nHit] - exactTriVx;
        float relVy = nVeh.nodes.velY[nHit] - exactTriVy;
        float relVz = nVeh.nodes.velZ[nHit] - exactTriVz;

        float approachSpeed = relVx * effNx + relVy * effNy + relVz * effNz;
        float jn = 0.0f;

        if (approachSpeed < 0.0f) {
            float deltaRelVel = -(1.0f + RESTITUTION) * approachSpeed;
            jn = deltaRelVel * invWTotal;
            dvX += jn * effNx;
            dvY += jn * effNy;
            dvZ += jn * effNz;
        }

        float equivalentJn = jn;
        if (penetration > 0.0f && approachSpeed >= -1e-4f) {
            equivalentJn += (pushAmount * invDt) * invWTotal;
        }

        if (equivalentJn > 0.0f) {
            float tangentVx = relVx - (approachSpeed * effNx);
            float tangentVy = relVy - (approachSpeed * effNy);
            float tangentVz = relVz - (approachSpeed * effNz);

            float vtLen = (float) Math.sqrt(tangentVx * tangentVx + tangentVy * tangentVy + tangentVz * tangentVz);

            float triMuS = wA * tVeh.nodes.friction[nA] + wB * tVeh.nodes.friction[nB] + wC * tVeh.nodes.friction[nC];
            float triMuK = wA * tVeh.nodes.slidingFriction[nA] + wB * tVeh.nodes.slidingFriction[nB] + wC * tVeh.nodes.slidingFriction[nC];

            float muS;
            float muK;

            int wIdx = nVeh.nodes.wheelId[nHit];
            if (wIdx >= 0 && wIdx < nVeh.wheels.count) {
                float staticBase = nVeh.wheels.frictionCoef[wIdx];
                float slidingBase = nVeh.wheels.slidingFrictionCoef[wIdx];
                float noLoad = nVeh.wheels.noLoadCoef[wIdx];
                float fullLoad = nVeh.wheels.fullLoadCoef[wIdx];
                float slope = nVeh.wheels.loadSensitivitySlope[wIdx];

                float equivalentLoadN = equivalentJn * invDt;
                float loadFactor = noLoad - (slope * equivalentLoadN);
                if (loadFactor < fullLoad) loadFactor = fullLoad;

                float stribeckVel = nVeh.wheels.stribeckVelMult[wIdx];
                float exponent = nVeh.wheels.stribeckExponent[wIdx];
                float speedFactor = 1.0f;
                if (vtLen > 1e-4f && stribeckVel > 1e-4f) {
                    float velRatio = vtLen / stribeckVel;
                    speedFactor = (float) Math.exp(-Math.pow(velRatio, exponent));
                }

                float dynamicMuMultiplier = slidingBase + (staticBase - slidingBase) * speedFactor;
                float treadCoef = nVeh.wheels.treadCoef[wIdx];

                muS = (staticBase * loadFactor * treadCoef) * triMuS;
                muK = (dynamicMuMultiplier * loadFactor * treadCoef) * triMuK;
            } else {
                muS = nVeh.nodes.friction[nHit] * triMuS;
                muK = nVeh.nodes.slidingFriction[nHit] * triMuK;
            }

            if (vtLen > 1e-8f) {
                float jtMax = vtLen * invWTotal;
                float frictionImpulse = (jtMax <= muS * equivalentJn) ? jtMax : muK * equivalentJn;

                float invVtLen = 1.0f / vtLen;
                float tDirX = tangentVx * invVtLen;
                float tDirY = tangentVy * invVtLen;
                float tDirZ = tangentVz * invVtLen;

                dvX -= frictionImpulse * tDirX;
                dvY -= frictionImpulse * tDirY;
                dvZ -= frictionImpulse * tDirZ;
            }
        }

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
        return 2;
    }

    /**
     * Resolves Minecraft environment (block) collisions for one vehicle against
     * the prepared voxel snapshot. Only reads the snapshot; never the world.
     */
    public void solveEnvironmentCollisions(SoftBodyVehicle vehicle, float dt) {
        NodeContainer nodes = vehicle.nodes;
        WheelContainer wheels = vehicle.wheels;
        VoxelSnapshot snapshot = voxelSnapshot;
        double entityX = vehicle.entityX, entityY = vehicle.entityY, entityZ = vehicle.entityZ;

        for (int i = 0; i < nodes.count; i++) {
            if (!nodes.collision[i]) continue;

            double worldX = entityX + nodes.posX[i];
            double worldY = entityY + nodes.posY[i];
            double worldZ = entityZ + nodes.posZ[i];

            if (worldY < 320 && worldY > -70 && snapshot.isSolid(worldX, worldY, worldZ)) {

                float oldLocalX = nodes.prevPosX[i];
                float oldLocalY = nodes.prevPosY[i];
                float oldLocalZ = nodes.prevPosZ[i];

                double oldWorldX = entityX + oldLocalX;
                double oldWorldY = entityY + oldLocalY;
                double oldWorldZ = entityZ + oldLocalZ;

                boolean hitX = snapshot.isSolid(worldX, oldWorldY, oldWorldZ);
                boolean hitY = snapshot.isSolid(oldWorldX, worldY, oldWorldZ);
                boolean hitZ = snapshot.isSolid(oldWorldX, oldWorldY, worldZ);

                if (hitY || hitX || hitZ) {
                    double invDt = 1.0 / dt;

                    double reboundCoef = PhysicsWorld.BLOCK_REBOUND;
                    double blockFriction = PhysicsWorld.BLOCK_FRICTION;

                    double gravityImpulse = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY) * dt;

                    double pushX = hitX ? Math.abs(nodes.posX[i] - oldLocalX) : 0.0;
                    double pushY = hitY ? Math.abs(nodes.posY[i] - oldLocalY) : 0.0;
                    double pushZ = hitZ ? Math.abs(nodes.posZ[i] - oldLocalZ) : 0.0;

                    double totalNormalPush = Math.sqrt(pushX*pushX + pushY*pushY + pushZ*pushZ);

                    double equivalentLoadN = (nodes.mass[i] * totalNormalPush) * (invDt * invDt);
                    double minGravityLoad = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY);
                    if (equivalentLoadN < minGravityLoad) equivalentLoadN = minGravityLoad;

                    double mu_s = nodes.friction[i] * blockFriction;
                    double mu_k = nodes.slidingFriction[i] * blockFriction;

                    int wIdx = nodes.wheelId[i];
                    if (0 <= wIdx && wIdx < wheels.count) {
                        double staticBase  = wheels.frictionCoef[wIdx];
                        double slidingBase = wheels.slidingFrictionCoef[wIdx];
                        double noLoad      = wheels.noLoadCoef[wIdx];
                        double fullLoad    = wheels.fullLoadCoef[wIdx];
                        double slope       = wheels.loadSensitivitySlope[wIdx];
                        double treadCoef   = wheels.treadCoef[wIdx];

                        double loadFactor = noLoad - (slope * equivalentLoadN);
                        if (loadFactor < fullLoad) loadFactor = fullLoad;

                        double vx = nodes.velX[i], vy = nodes.velY[i], vz = nodes.velZ[i];
                        double tVelSq = (hitX ? 0 : vx*vx) + (hitY ? 0 : vy*vy) + (hitZ ? 0 : vz*vz);
                        double vtLen = Math.sqrt(tVelSq);

                        double stribeckVel = wheels.stribeckVelMult[wIdx];
                        double exponent    = wheels.stribeckExponent[wIdx];
                        double speedFactor = 1.0;
                        if (vtLen > 1e-4 && stribeckVel > 1e-4) {
                            double velRatio = vtLen / stribeckVel;
                            speedFactor = Math.exp(-Math.pow(velRatio, exponent));
                        }

                        double dynamicMuMultiplier = slidingBase + (staticBase - slidingBase) * speedFactor;
                        mu_s = (staticBase * loadFactor * treadCoef)  * blockFriction;
                        mu_k = (dynamicMuMultiplier * loadFactor * treadCoef) * blockFriction;
                    }

                    if (hitY) {
                        // J = m * |v| * (1 + e) + m * |g| * dt
                        double jn = nodes.mass[i] * Math.abs(nodes.velY[i]) * (1.0 + reboundCoef) + gravityImpulse;

                        nodes.velY[i] *= -reboundCoef;
                        nodes.posY[i] = oldLocalY;

                        double vx = nodes.velX[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vx*vx + vz*vz);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0f; nodes.velZ[i] = 0.0f;
                            } else {
                                double frictionImpulse = mu_k * jn;
                                velKeepRatio = Math.max(0.0, 1.0 - (frictionImpulse / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        double creepX = nodes.posX[i] - oldLocalX, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepX*creepX + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posX[i] = oldLocalX; nodes.posZ[i] = oldLocalZ;
                        } else {
                            nodes.posX[i] = (float) (oldLocalX + creepX * velKeepRatio);
                            nodes.posZ[i] = (float) (oldLocalZ + creepZ * velKeepRatio);
                        }
                    }

                    if (hitX) {
                        double jn = nodes.mass[i] * Math.abs(nodes.velX[i]) * (1.0 + reboundCoef) + gravityImpulse;
                        nodes.velX[i] *= -reboundCoef;
                        nodes.posX[i] = oldLocalX;

                        double vy = nodes.velY[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vy*vy + vz*vz);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velY[i] = 0.0f; nodes.velZ[i] = 0.0f;
                            } else {
                                velKeepRatio = Math.max(0.0, 1.0 - ((mu_k * jn) / jtReq));
                                nodes.velY[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        double creepY = nodes.posY[i] - oldLocalY, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepY*creepY + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posY[i] = oldLocalY; nodes.posZ[i] = oldLocalZ;
                        } else {
                            nodes.posY[i] = (float) (oldLocalY + creepY * velKeepRatio);
                            nodes.posZ[i] = (float) (oldLocalZ + creepZ * velKeepRatio);
                        }
                    }

                    if (hitZ) {
                        double jn = nodes.mass[i] * Math.abs(nodes.velZ[i]) * (1.0 + reboundCoef) + gravityImpulse;
                        nodes.velZ[i] *= -reboundCoef;
                        nodes.posZ[i] = oldLocalZ;

                        double vx = nodes.velX[i], vy = nodes.velY[i];
                        double vtLen = Math.sqrt(vx*vx + vy*vy);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0f; nodes.velY[i] = 0.0f;
                            } else {
                                velKeepRatio = Math.max(0.0, 1.0 - ((mu_k * jn) / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velY[i] *= velKeepRatio;
                            }
                        }

                        double creepX = nodes.posX[i] - oldLocalX, creepY = nodes.posY[i] - oldLocalY;
                        double creepLen = Math.sqrt(creepX*creepX + creepY*creepY);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posX[i] = oldLocalX; nodes.posY[i] = oldLocalY;
                        } else {
                            nodes.posX[i] = (float) (oldLocalX + creepX * velKeepRatio);
                            nodes.posY[i] = (float) (oldLocalY + creepY * velKeepRatio);
                        }
                    }
                }
            }
        }
    }
}
