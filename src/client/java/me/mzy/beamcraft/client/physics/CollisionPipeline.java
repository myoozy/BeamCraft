package me.mzy.beamcraft.client.physics;

import java.util.List;
import java.util.stream.IntStream;

/**
 * Collision pipeline shared by every vehicle in a {@link PhysicsWorld}.
 *
 * Owns the concrete algorithms that were previously spread across the physics
 * world scheduler and the per-vehicle class:
 * <ul>
 *     <li>soft-body contact candidate generation (triangle vs node, fed by the
 *     active chunk broad-phase or the retained SAP implementation),</li>
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
    static final double SOFT_BROADPHASE_MARGIN = 0.01;
    private static final float SOFT_CONTACT_THICKNESS = 0.01f;
    private static final float SOFT_CONTACT_BARYCENTRIC_TOLERANCE = 0.01f;
    static final int NARROW_STAT_BITS = 21;
    static final long NARROW_STAT_MASK = (1L << NARROW_STAT_BITS) - 1L;

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

        for (int i = 0; i < vehicle.triangles.count; i++) {
            if (!vehicle.triangles.collision[i] || vehicle.triangles.broken[i]) continue;

            int nA = vehicle.triangles.node1[i];
            int nB = vehicle.triangles.node2[i];
            int nC = vehicle.triangles.node3[i];

            double ax = eX + vehicle.nodes.posX[nA], ay = eY + vehicle.nodes.posY[nA], az = eZ + vehicle.nodes.posZ[nA];
            double bx = eX + vehicle.nodes.posX[nB], by = eY + vehicle.nodes.posY[nB], bz = eZ + vehicle.nodes.posZ[nB];
            double cx = eX + vehicle.nodes.posX[nC], cy = eY + vehicle.nodes.posY[nC], cz = eZ + vehicle.nodes.posZ[nC];
            double previousAx = eX + vehicle.nodes.prevPosX[nA];
            double previousAy = eY + vehicle.nodes.prevPosY[nA];
            double previousAz = eZ + vehicle.nodes.prevPosZ[nA];
            double previousBx = eX + vehicle.nodes.prevPosX[nB];
            double previousBy = eY + vehicle.nodes.prevPosY[nB];
            double previousBz = eZ + vehicle.nodes.prevPosZ[nB];
            double previousCx = eX + vehicle.nodes.prevPosX[nC];
            double previousCy = eY + vehicle.nodes.prevPosY[nC];
            double previousCz = eZ + vehicle.nodes.prevPosZ[nC];

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

            double minX = Math.min(Math.min(previousAx, Math.min(previousBx, previousCx)), Math.min(Math.min(ax, futureAx),
                    Math.min(Math.min(bx, futureBx), Math.min(cx, futureCx)))) - SOFT_BROADPHASE_MARGIN;
            double maxX = Math.max(Math.max(previousAx, Math.max(previousBx, previousCx)), Math.max(Math.max(ax, futureAx),
                    Math.max(Math.max(bx, futureBx), Math.max(cx, futureCx)))) + SOFT_BROADPHASE_MARGIN;
            double minY = Math.min(Math.min(previousAy, Math.min(previousBy, previousCy)), Math.min(Math.min(ay, futureAy),
                    Math.min(Math.min(by, futureBy), Math.min(cy, futureCy)))) - SOFT_BROADPHASE_MARGIN;
            double maxY = Math.max(Math.max(previousAy, Math.max(previousBy, previousCy)), Math.max(Math.max(ay, futureAy),
                    Math.max(Math.max(by, futureBy), Math.max(cy, futureCy)))) + SOFT_BROADPHASE_MARGIN;
            double minZ = Math.min(Math.min(previousAz, Math.min(previousBz, previousCz)), Math.min(Math.min(az, futureAz),
                    Math.min(Math.min(bz, futureBz), Math.min(cz, futureCz)))) - SOFT_BROADPHASE_MARGIN;
            double maxZ = Math.max(Math.max(previousAz, Math.max(previousBz, previousCz)), Math.max(Math.max(az, futureAz),
                    Math.max(Math.max(bz, futureBz), Math.max(cz, futureCz)))) + SOFT_BROADPHASE_MARGIN;

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
        long aabbPassed = result == 1 || result == 2 ? 1L : 0L;
        long resolved = result == 2 ? 1L : 0L;
        long certificateSkipped = result == 3 ? 1L : 0L;
        return (certificateSkipped << (NARROW_STAT_BITS * 2))
                | (resolved << NARROW_STAT_BITS)
                | aabbPassed;
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

        float pax = pX - ax, pay = pY - ay, paz = pZ - az;
        float bax = bx - ax, bay = by - ay, baz = bz - az;
        float cax = cx - ax, cay = cy - ay, caz = cz - az;
        if (collisionManager.separationCertificateStillValid(contactId,
                pax, pay, paz, bax, bay, baz, cax, cay, caz)) {
            return 3;
        }

        float minX = Math.min(ax, Math.min(bx, cx)) - THICKNESS;
        float maxX = Math.max(ax, Math.max(bx, cx)) + THICKNESS;
        float minY = Math.min(ay, Math.min(by, cy)) - THICKNESS;
        float maxY = Math.max(ay, Math.max(by, cy)) + THICKNESS;
        float minZ = Math.min(az, Math.min(bz, cz)) - THICKNESS;
        float maxZ = Math.max(az, Math.max(bz, cz)) + THICKNESS;
        boolean currentAabbPassed = pX >= minX && pX <= maxX
                && pY >= minY && pY <= maxY && pZ >= minZ && pZ <= maxZ;
        float sweptHitTime = -1.0f;
        if (!currentAabbPassed) {
            sweptHitTime = sweptPointTriangleHitTime(
                    entityDeltaX + nVeh.nodes.prevPosX[nHit],
                    entityDeltaY + nVeh.nodes.prevPosY[nHit],
                    entityDeltaZ + nVeh.nodes.prevPosZ[nHit],
                    tVeh.nodes.prevPosX[nA], tVeh.nodes.prevPosY[nA], tVeh.nodes.prevPosZ[nA],
                    tVeh.nodes.prevPosX[nB], tVeh.nodes.prevPosY[nB], tVeh.nodes.prevPosZ[nB],
                    tVeh.nodes.prevPosX[nC], tVeh.nodes.prevPosY[nC], tVeh.nodes.prevPosZ[nC],
                    pX, pY, pZ, ax, ay, az, bx, by, bz, cx, cy, cz);
        }
        if (!currentAabbPassed && sweptHitTime < 0.0f) {
            float aabbSlack = Math.max(
                    Math.max(Math.max(minX - pX, pX - maxX), Math.max(minY - pY, pY - maxY)),
                    Math.max(minZ - pZ, pZ - maxZ));
            collisionManager.recordSeparationCertificate(contactId, aabbSlack,
                    pax, pay, paz, bax, bay, baz, cax, cay, caz);
            return 0;
        }

        float abx = bax, aby = bay, abz = baz;
        float acx = cax, acy = cay, acz = caz;
        float nx = aby * acz - abz * acy;
        float ny = abz * acx - abx * acz;
        float nz = abx * acy - aby * acx;

        float nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < PhysicsWorld.KINDA_SMALL_NUMBER) {
            collisionManager.invalidateSeparationCertificate(contactId);
            return 1;
        }
        float invNLen = 1.0f / (float) Math.sqrt(nLenSq);
        nx *= invNLen; ny *= invNLen; nz *= invNLen;

        float apx = pax, apy = pay, apz = paz;
        float distCurr = apx * nx + apy * ny + apz * nz;

        float triVx = (tVeh.nodes.velX[nA] + tVeh.nodes.velX[nB] + tVeh.nodes.velX[nC]) * 0.33333334f;
        float triVy = (tVeh.nodes.velY[nA] + tVeh.nodes.velY[nB] + tVeh.nodes.velY[nC]) * 0.33333334f;
        float triVz = (tVeh.nodes.velZ[nA] + tVeh.nodes.velZ[nB] + tVeh.nodes.velZ[nC]) * 0.33333334f;

        float approxRelV = (nVeh.nodes.velX[nHit] - triVx) * nx +
                (nVeh.nodes.velY[nHit] - triVy) * ny +
                (nVeh.nodes.velZ[nHit] - triVz) * nz;

        float prevAx = tVeh.nodes.prevPosX[nA], prevAy = tVeh.nodes.prevPosY[nA], prevAz = tVeh.nodes.prevPosZ[nA];
        float prevAbx = tVeh.nodes.prevPosX[nB] - prevAx;
        float prevAby = tVeh.nodes.prevPosY[nB] - prevAy;
        float prevAbz = tVeh.nodes.prevPosZ[nB] - prevAz;
        float prevAcx = tVeh.nodes.prevPosX[nC] - prevAx;
        float prevAcy = tVeh.nodes.prevPosY[nC] - prevAy;
        float prevAcz = tVeh.nodes.prevPosZ[nC] - prevAz;
        float prevNx = prevAby * prevAcz - prevAbz * prevAcy;
        float prevNy = prevAbz * prevAcx - prevAbx * prevAcz;
        float prevNz = prevAbx * prevAcy - prevAby * prevAcx;
        float prevApx = entityDeltaX + nVeh.nodes.prevPosX[nHit] - prevAx;
        float prevApy = entityDeltaY + nVeh.nodes.prevPosY[nHit] - prevAy;
        float prevApz = entityDeltaZ + nVeh.nodes.prevPosZ[nHit] - prevAz;
        float prevSignedVolume = prevApx * prevNx + prevApy * prevNy + prevApz * prevNz;
        float pushDir = prevSignedVolume != 0.0f
                ? (prevSignedVolume > 0.0f ? 1.0f : -1.0f)
                : (distCurr - approxRelV * dt > 0.0f ? 1.0f : -1.0f);
        float signedDist = distCurr * pushDir;
        float penetration = THICKNESS - signedDist;
        if (penetration <= 0.0f) {
            recordGeometricSeparation(contactId, apx, apy, apz, abx, aby, abz, acx, acy, acz);
            return 1;
        }

        // Most cached candidates fail the plane-distance test above. Delay the
        // barycentric Gram matrix until a contact can actually penetrate.
        float d00 = abx * abx + aby * aby + abz * abz;
        float d01 = abx * acx + aby * acy + abz * acz;
        float d11 = acx * acx + acy * acy + acz * acz;
        float denom = d00 * d11 - d01 * d01;
        if (denom < PhysicsWorld.KINDA_SMALL_NUMBER) {
            collisionManager.invalidateSeparationCertificate(contactId);
            return 1;
        }
        float invDenom = 1.0f / denom;

        float ppx = apx - distCurr * nx;
        float ppy = apy - distCurr * ny;
        float ppz = apz - distCurr * nz;

        float d20 = ppx * abx + ppy * aby + ppz * abz;
        float d21 = ppx * acx + ppy * acy + ppz * acz;

        float wB = (d11 * d20 - d01 * d21) * invDenom;
        float wC = (d00 * d21 - d01 * d20) * invDenom;
        float wA = 1.0f - wB - wC;

        if (sweptHitTime >= 0.0f) {
            float hitAx = lerp(prevAx, ax, sweptHitTime);
            float hitAy = lerp(prevAy, ay, sweptHitTime);
            float hitAz = lerp(prevAz, az, sweptHitTime);
            float hitBx = lerp(tVeh.nodes.prevPosX[nB], bx, sweptHitTime);
            float hitBy = lerp(tVeh.nodes.prevPosY[nB], by, sweptHitTime);
            float hitBz = lerp(tVeh.nodes.prevPosZ[nB], bz, sweptHitTime);
            float hitCx = lerp(tVeh.nodes.prevPosX[nC], cx, sweptHitTime);
            float hitCy = lerp(tVeh.nodes.prevPosY[nC], cy, sweptHitTime);
            float hitCz = lerp(tVeh.nodes.prevPosZ[nC], cz, sweptHitTime);
            float hitPx = lerp(entityDeltaX + nVeh.nodes.prevPosX[nHit], pX, sweptHitTime);
            float hitPy = lerp(entityDeltaY + nVeh.nodes.prevPosY[nHit], pY, sweptHitTime);
            float hitPz = lerp(entityDeltaZ + nVeh.nodes.prevPosZ[nHit], pZ, sweptHitTime);
            float hitAbx = hitBx - hitAx, hitAby = hitBy - hitAy, hitAbz = hitBz - hitAz;
            float hitAcx = hitCx - hitAx, hitAcy = hitCy - hitAy, hitAcz = hitCz - hitAz;
            float hitApx = hitPx - hitAx, hitApy = hitPy - hitAy, hitApz = hitPz - hitAz;
            float hitD00 = hitAbx * hitAbx + hitAby * hitAby + hitAbz * hitAbz;
            float hitD01 = hitAbx * hitAcx + hitAby * hitAcy + hitAbz * hitAcz;
            float hitD11 = hitAcx * hitAcx + hitAcy * hitAcy + hitAcz * hitAcz;
            float hitD20 = hitApx * hitAbx + hitApy * hitAby + hitApz * hitAbz;
            float hitD21 = hitApx * hitAcx + hitApy * hitAcy + hitApz * hitAcz;
            float hitDenom = hitD00 * hitD11 - hitD01 * hitD01;
            if (hitDenom >= PhysicsWorld.KINDA_SMALL_NUMBER) {
                float hitInvDenom = 1.0f / hitDenom;
                wB = (hitD11 * hitD20 - hitD01 * hitD21) * hitInvDenom;
                wC = (hitD00 * hitD21 - hitD01 * hitD20) * hitInvDenom;
                wA = 1.0f - wB - wC;
            }
        }

        final float TOLERANCE = -SOFT_CONTACT_BARYCENTRIC_TOLERANCE;
        if (!(wA >= TOLERANCE && wB >= TOLERANCE && wC >= TOLERANCE)) {
            recordGeometricSeparation(contactId, apx, apy, apz, abx, aby, abz, acx, acy, acz);
            return 1;
        }

        float effNx = nx * pushDir, effNy = ny * pushDir, effNz = nz * pushDir;

        float massNode = nVeh.nodes.mass[nHit];
        float massA = tVeh.nodes.mass[nA], massB = tVeh.nodes.mass[nB], massC = tVeh.nodes.mass[nC];

        float wTotal = (1.0f / massNode) + (wA * wA / massA) + (wB * wB / massB) + (wC * wC / massC);
        if (wTotal < PhysicsWorld.KINDA_SMALL_NUMBER) {
            collisionManager.invalidateSeparationCertificate(contactId);
            return 1;
        }
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
        collisionManager.invalidateSeparationCertificate(contactId);
        if (sweptHitTime >= 0.0f) collisionManager.sweptResolvedCount.incrementAndGet();
        return 2;
    }

    /**
     * Generates candidates through a node-chunk SAP followed by a small node
     * SAP inside each overlapping triangle-meshlet/node-chunk pair. Chunk
     * membership is unique, so the same node-triangle pair cannot be emitted
     * by two different chunk pairs.
     */
    public void generateChunkCollisionCandidates(SoftBodyVehicle triangleVehicle,
                                                 List<SoftBodyVehicle> activeVehicles) {
        CollisionCandidateBuffer buffer = new CollisionCandidateBuffer();
        generateChunkCollisionCandidates(
                triangleVehicle, activeVehicles, 0,
                triangleVehicle.collisionChunks.triangleMeshletCount(), buffer);
        int stored = collisionManager.appendContacts(buffer);
        clearCandidateStats(triangleVehicle);
        applyCandidateStats(buffer, stored, buffer.dropped + buffer.count - stored);
    }

    void generateChunkCollisionCandidates(SoftBodyVehicle triangleVehicle,
                                          List<SoftBodyVehicle> activeVehicles,
                                          int meshletStart, int meshletEnd,
                                          CollisionCandidateBuffer buffer) {
        CollisionChunkIndex triangleChunks = triangleVehicle.collisionChunks;
        TriangleContainer triangles = triangleVehicle.triangles;
        buffer.reset(triangleVehicle);

        for (int meshlet = meshletStart; meshlet < meshletEnd; meshlet++) {
            if (!triangleChunks.meshletActive(meshlet)) continue;
            for (SoftBodyVehicle nodeVehicle : activeVehicles) {
                CollisionChunkIndex nodeChunks = nodeVehicle.collisionChunks;
                boolean self = nodeVehicle == triangleVehicle;
                int firstNodeChunk = triangleChunks.firstNodeChunkCandidate(meshlet, nodeChunks);
                for (int sortedChunk = firstNodeChunk;
                     sortedChunk < nodeChunks.nodeChunkCount(); sortedChunk++) {
                    if (triangleChunks.nodeChunkStartsAfterMeshlet(meshlet, nodeChunks, sortedChunk)) break;
                    int nodeChunk = triangleChunks.sortedNodeChunkAt(nodeChunks, sortedChunk);
                    if (self && !nodeChunks.nodeChunkHasSelfCollision(nodeChunk)) continue;
                    if (self && triangleChunks.sameKnownPart(meshlet, nodeChunks, nodeChunk)) continue;
                    buffer.chunkPairTests++;
                    if (!triangleChunks.chunksOverlap(meshlet, nodeChunks, nodeChunk)) continue;
                    buffer.chunkPairOverlaps++;
                    int localSweepAxis = triangleChunks.localSweepAxis(meshlet, nodeChunk, nodeChunks);
                    boolean productivePair = false;

                    for (int triangleMember = triangleChunks.triangleMeshletStart(meshlet);
                         triangleMember < triangleChunks.triangleMeshletEnd(meshlet); triangleMember++) {
                        int triangle = triangleChunks.triangleAt(triangleMember);
                        if (triangles.broken[triangle]) continue;
                        int nA = triangles.node1[triangle];
                        int nB = triangles.node2[triangle];
                        int nC = triangles.node3[triangle];
                        int trianglePart = triangles.partId[triangle];

                        int firstNode = triangleChunks.firstNodeCandidate(
                                triangle, nodeChunk, nodeChunks, localSweepAxis);
                        for (int sortedNode = firstNode;
                             sortedNode < triangleChunks.sortedNodeEnd(
                                     nodeChunk, nodeChunks, localSweepAxis); sortedNode++) {
                            if (triangleChunks.nodeStartsAfterTriangle(
                                    triangle, nodeChunks, sortedNode, localSweepAxis)) break;
                            int node = triangleChunks.sortedNodeAt(nodeChunks, sortedNode);
                            if (self && !nodeVehicle.nodes.selfCollision[node]) continue;
                            buffer.finePairTests++;
                            if (!triangleChunks.triangleOverlapsNode(triangle, nodeChunks, node)) continue;
                            productivePair = true;
                            buffer.rawHits++;

                            if (self) {
                                if (node == nA || node == nB || node == nC) continue;
                                if (trianglePart >= 0 && trianglePart < triangleVehicle.matrixPartStride
                                        && triangleVehicle.nodeInPartMatrix != null
                                        && triangleVehicle.nodeInPartMatrix[
                                        node * triangleVehicle.matrixPartStride + trianglePart]) continue;
                            }
                            buffer.add(nodeVehicle, node, triangleVehicle, nA, nB, nC);
                        }
                    }
                    if (productivePair) buffer.chunkPairProductive++;
                }
            }
        }

    }

    static void clearCandidateStats(SoftBodyVehicle vehicle) {
        vehicle.collisionCandidateSapHits = 0;
        vehicle.collisionCandidateStored = 0;
        vehicle.collisionCandidateDropped = 0;
        vehicle.collisionChunkPairTests = 0;
        vehicle.collisionChunkPairOverlaps = 0;
        vehicle.collisionChunkPairProductive = 0;
        vehicle.collisionFinePairTests = 0L;
    }

    static void applyCandidateStats(CollisionCandidateBuffer buffer, int stored, int dropped) {
        SoftBodyVehicle vehicle = buffer.triangleVehicle;
        vehicle.collisionCandidateSapHits += buffer.rawHits;
        vehicle.collisionCandidateStored += stored;
        vehicle.collisionCandidateDropped += dropped;
        vehicle.collisionChunkPairTests += buffer.chunkPairTests;
        vehicle.collisionChunkPairOverlaps += buffer.chunkPairOverlaps;
        vehicle.collisionChunkPairProductive += buffer.chunkPairProductive;
        vehicle.collisionFinePairTests += buffer.finePairTests;
    }

    private void recordGeometricSeparation(int contactId,
                                           float px, float py, float pz,
                                           float bx, float by, float bz,
                                           float cx, float cy, float cz) {
        float distanceSq = pointTriangleDistanceSquared(px, py, pz, bx, by, bz, cx, cy, cz);
        float abLength = (float) Math.sqrt(bx * bx + by * by + bz * bz);
        float acLength = (float) Math.sqrt(cx * cx + cy * cy + cz * cz);
        float contactEnvelope = SOFT_CONTACT_THICKNESS
                + SOFT_CONTACT_BARYCENTRIC_TOLERANCE * (abLength + acLength);
        float slack = (float) Math.sqrt(distanceSq) - contactEnvelope;
        collisionManager.recordSeparationCertificate(contactId, slack,
                px, py, pz, bx, by, bz, cx, cy, cz);
    }

    /** Squared distance from P to triangle (0, B, C). */
    static float pointTriangleDistanceSquared(float px, float py, float pz,
                                              float bx, float by, float bz,
                                              float cx, float cy, float cz) {
        float d1 = bx * px + by * py + bz * pz;
        float d2 = cx * px + cy * py + cz * pz;
        if (d1 <= 0.0f && d2 <= 0.0f) return px * px + py * py + pz * pz;

        float bpx = px - bx, bpy = py - by, bpz = pz - bz;
        float d3 = bx * bpx + by * bpy + bz * bpz;
        float d4 = cx * bpx + cy * bpy + cz * bpz;
        if (d3 >= 0.0f && d4 <= d3) return bpx * bpx + bpy * bpy + bpz * bpz;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            float v = d1 / (d1 - d3);
            float qx = px - v * bx, qy = py - v * by, qz = pz - v * bz;
            return qx * qx + qy * qy + qz * qz;
        }

        float cpx = px - cx, cpy = py - cy, cpz = pz - cz;
        float d5 = bx * cpx + by * cpy + bz * cpz;
        float d6 = cx * cpx + cy * cpy + cz * cpz;
        if (d6 >= 0.0f && d5 <= d6) return cpx * cpx + cpy * cpy + cpz * cpz;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            float w = d2 / (d2 - d6);
            float qx = px - w * cx, qy = py - w * cy, qz = pz - w * cz;
            return qx * qx + qy * qy + qz * qz;
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f) {
            float edgeX = cx - bx, edgeY = cy - by, edgeZ = cz - bz;
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            float qx = bpx - w * edgeX, qy = bpy - w * edgeY, qz = bpz - w * edgeZ;
            return qx * qx + qy * qy + qz * qz;
        }

        float denom = 1.0f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        float qx = px - bx * v - cx * w;
        float qy = py - by * v - cy * w;
        float qz = pz - bz * v - cz * w;
        return qx * qx + qy * qy + qz * qz;
    }

    static float sweptPointTriangleHitTime(
            float p0x, float p0y, float p0z,
            float a0x, float a0y, float a0z,
            float b0x, float b0y, float b0z,
            float c0x, float c0y, float c0z,
            float p1x, float p1y, float p1z,
            float a1x, float a1y, float a1z,
            float b1x, float b1y, float b1z,
            float c1x, float c1y, float c1z) {
        float f0 = orientedVolume(p0x, p0y, p0z, a0x, a0y, a0z, b0x, b0y, b0z, c0x, c0y, c0z);
        float f1 = orientedVolume(p1x, p1y, p1z, a1x, a1y, a1z, b1x, b1y, b1z, c1x, c1y, c1z);
        if (!Float.isFinite(f0) || !Float.isFinite(f1) || f0 * f1 > 0.0f) return -1.0f;

        float low = 0.0f, high = 1.0f, lowValue = f0;
        for (int iteration = 0; iteration < 12; iteration++) {
            float middle = (low + high) * 0.5f;
            float value = orientedVolumeAt(middle,
                    p0x, p0y, p0z, a0x, a0y, a0z, b0x, b0y, b0z, c0x, c0y, c0z,
                    p1x, p1y, p1z, a1x, a1y, a1z, b1x, b1y, b1z, c1x, c1y, c1z);
            if (Math.abs(value) <= 1e-7f) {
                low = high = middle;
                break;
            }
            if ((lowValue <= 0.0f && value <= 0.0f) || (lowValue >= 0.0f && value >= 0.0f)) {
                low = middle;
                lowValue = value;
            } else {
                high = middle;
            }
        }

        float hitTime = (low + high) * 0.5f;
        float ax = lerp(a0x, a1x, hitTime), ay = lerp(a0y, a1y, hitTime), az = lerp(a0z, a1z, hitTime);
        float bx = lerp(b0x, b1x, hitTime), by = lerp(b0y, b1y, hitTime), bz = lerp(b0z, b1z, hitTime);
        float cx = lerp(c0x, c1x, hitTime), cy = lerp(c0y, c1y, hitTime), cz = lerp(c0z, c1z, hitTime);
        float px = lerp(p0x, p1x, hitTime) - ax;
        float py = lerp(p0y, p1y, hitTime) - ay;
        float pz = lerp(p0z, p1z, hitTime) - az;
        float abx = bx - ax, aby = by - ay, abz = bz - az;
        float acx = cx - ax, acy = cy - ay, acz = cz - az;
        float d00 = abx * abx + aby * aby + abz * abz;
        float d01 = abx * acx + aby * acy + abz * acz;
        float d11 = acx * acx + acy * acy + acz * acz;
        float d20 = px * abx + py * aby + pz * abz;
        float d21 = px * acx + py * acy + pz * acz;
        float denominator = d00 * d11 - d01 * d01;
        if (denominator < PhysicsWorld.KINDA_SMALL_NUMBER) return -1.0f;
        float inverse = 1.0f / denominator;
        float wB = (d11 * d20 - d01 * d21) * inverse;
        float wC = (d00 * d21 - d01 * d20) * inverse;
        float wA = 1.0f - wB - wC;
        float tolerance = -SOFT_CONTACT_BARYCENTRIC_TOLERANCE;
        return wA >= tolerance && wB >= tolerance && wC >= tolerance ? hitTime : -1.0f;
    }

    private static float orientedVolumeAt(float t,
                                          float p0x, float p0y, float p0z,
                                          float a0x, float a0y, float a0z,
                                          float b0x, float b0y, float b0z,
                                          float c0x, float c0y, float c0z,
                                          float p1x, float p1y, float p1z,
                                          float a1x, float a1y, float a1z,
                                          float b1x, float b1y, float b1z,
                                          float c1x, float c1y, float c1z) {
        return orientedVolume(
                lerp(p0x, p1x, t), lerp(p0y, p1y, t), lerp(p0z, p1z, t),
                lerp(a0x, a1x, t), lerp(a0y, a1y, t), lerp(a0z, a1z, t),
                lerp(b0x, b1x, t), lerp(b0y, b1y, t), lerp(b0z, b1z, t),
                lerp(c0x, c1x, t), lerp(c0y, c1y, t), lerp(c0z, c1z, t));
    }

    private static float orientedVolume(float px, float py, float pz,
                                        float ax, float ay, float az,
                                        float bx, float by, float bz,
                                        float cx, float cy, float cz) {
        float abx = bx - ax, aby = by - ay, abz = bz - az;
        float acx = cx - ax, acy = cy - ay, acz = cz - az;
        float apx = px - ax, apy = py - ay, apz = pz - az;
        float nx = aby * acz - abz * acy;
        float ny = abz * acx - abx * acz;
        float nz = abx * acy - aby * acx;
        return apx * nx + apy * ny + apz * nz;
    }

    private static float lerp(float from, float to, float t) {
        return from + (to - from) * t;
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
