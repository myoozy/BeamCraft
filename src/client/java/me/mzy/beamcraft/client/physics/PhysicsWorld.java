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
    public static final float GRAVITY = -9.81f;
    public static final float SOUND_SPEED = 340.0f;
    public static final float BLOCK_REBOUND = 0.0f;
    public static final float BLOCK_FRICTION = 1.0f;
    public static final float METAL_PLASTIC_FLOW_RATE = 10.0f;
    public static final float KINDA_SMALL_NUMBER = 1e-8f;
    public static final float KINDA_BIG_NUMBER = 1e8f;
    public static final int MAX_AABB_SIZE = 10;
    public static final float invPhysicsDT = 2000.0f;

    // 鐢ㄤ簬澶勭悊鍜宮c涓栫晫鐨勭鎾?
    public final VoxelSnapshot voxelSnapshot = new VoxelSnapshot();
    BlockPos.Mutable mutablePos = new BlockPos.Mutable();

    // 澶勭悊杞綋涔嬮棿鐨勭鎾?
    public final DynamicAxisSweep globalSap = new DynamicAxisSweep();
    public final SoftBodyCollisionManager collisionManager = new SoftBodyCollisionManager();

    private int nextVehicleId = 0;     // 姘歌繙閫掑鐨勫叏灞€杞︾墝鍙?(缁濅笉閲嶅)

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
     * 瀹夊叏鍦板皢杞﹁締浠庣墿鐞嗕笘鐣屼腑绉婚櫎骞堕攢姣?
     */
    public void removeVehicle(SoftBodyVehicle vehicle) {
        if (vehicle == null || !vehicles.contains(vehicle)) return;

        // 1. 浠庢椿璺冨垪琛ㄤ腑绉婚櫎
        // 鐢变簬浣犵殑 vehicles 鏄?CopyOnWriteArrayList锛岃繖涓€姝ュぉ鐢熺嚎绋嬪畨鍏紒
        // 鍝€曠墿鐞嗙嚎绋嬫鍦?foreach 閬嶅巻鎵€鏈夎溅锛岀Щ闄ゆ搷浣滀篃涓嶄細瀵艰嚧宕╂簝銆?
        vehicles.remove(vehicle);

        // 2. 閲婃斁璇ヨ溅杈嗗崰鐢ㄧ殑搴曞眰 SoA 鏁扮粍鍐呭瓨
        vehicle.clear();

        System.out.println("馃殫 Vehicle removed safely. ID: " + vehicle.vehicleId);

        // 娉ㄦ剰锛氭垜浠笉闇€瑕佸幓娓呯悊 SAP 鏍戞垨鑰?SoftBody)CollisionManager锛?
        // 鍥犱负 SAP 鏍戝湪瀹介樁娈典細璋冪敤 globalSap.clear() 骞跺畬鍏ㄦ牴鎹綋鍓嶇殑 vehicles 鍒楄〃閲嶅缓銆?
        // CollisionManager涔熶細鍦ㄥ闃舵鑷姩 clearContacts()銆?
        // 杩欏氨鏄瘡娆￠噸寤哄闃舵鏋舵瀯鐨勬渶澶т紭鍔匡細娌℃湁鐘舵€佹畫鐣欙紒
    }

    public void clear() {
        vehicles.clear();
        collisionManager.clearContacts(); // 娓呯悊纰版挒绠＄悊鍣?
        System.out.println("馃Ч Physics world data cleared");
    }

    /**
     * Main physics update loop
     */
    public void step(World mcWorld, double dt, double[] lastPhycisMsDetail) {
        int subSteps = (int)Math.ceil(dt * invPhysicsDT);
        float subDt = (float) (dt / subSteps);
        int broadphaseRate = 10;

        long t1 = System.nanoTime();

        voxelSnapshot.clear();
        // 璁╂瘡杈嗚溅鑷繁鍘绘壂鎻忓懆鍥寸殑鏂瑰潡骞跺啓鍏ュ叏灞€ snapshot
        for (SoftBodyVehicle vehicle : vehicles) {
            vehicle.cacheEntityLocation();
            vehicle.updateVoxelSnapshot(mcWorld, voxelSnapshot, mutablePos, dt);
        }

        long t2 = System.nanoTime();
        double mcWorldScanMs = (t2 - t1) / 1_000_000.0;
        double internalForceMs = 0.0, globalSAPMs = 0.0, dyeCollisionMs = 0.0, softCollisionMs = 0.0, mcCollisionMs = 0.0;

        for (int s = 0; s < subSteps; s++) {

            long ti1 = System.nanoTime();

            // 1. 绠楀唴鍔涘苟绉垎棰勬祴鍧愭爣銆愬畬鍏ㄧ嫭绔嬪苟琛屻€?
            vehicles.parallelStream().forEach(vehicle -> {
                vehicle.solveInternalForces(subDt);
            });

            long ti2 = System.nanoTime();
            internalForceMs += (ti2 - ti1) / 1_000_000.0;

            // 2. 瀹介樁娈碉細闄嶉鏇存柊鏍戜笌鎺ヨЕ缂撳瓨
            if (s % broadphaseRate == 0) {
                long tii1 = System.nanoTime();
                globalSap.clear();

                // 鈽?鍔ㄦ€佸唴瀛樼揣鍑?(Dynamic Packing) 鈽?
                // 姣忔瀹介樁娈靛墠锛岄噸鏂颁负瀛樻椿鐨勮溅杈嗗垎閰嶇揣鍑戠殑鍏ㄥ眬 ID 鍋忕Щ閲忋€?
                // 杩欐牱鏃犺鐜╁鎬庝箞鍒犻櫎鍜岀敓鎴愯溅杈嗭紝鍐呭瓨姘歌繙鏄?0 绌洪殭
                int activeOffset = 0;
                for (SoftBodyVehicle vehicle : vehicles) {
                    vehicle.globalNodeOffset = activeOffset;
                    activeOffset += vehicle.nodes.count;

                    globalSap.insertNodes(vehicle); // 椤轰究鎶婅妭鐐规彃鍏?SAP 鏍?
                }

                // 濡傛灉褰撳墠鎵€鏈夎溅鐨勮妭鐐规€绘暟瓒呰繃浜嗙鐞嗗櫒鐨勪笂闄愶紝鍙互鍦ㄨ繖閲屽姞涓鍛婄敋鑷虫墿瀹归€昏緫
                // if (activeOffset >= SoftBodyCollisionManager.MAX_GLOBAL_NODES) { ... }

                globalSap.updateAndSort();

                long tii2 = System.nanoTime();
                globalSAPMs += (tii2 - tii1) / 1_000_000.0;

                collisionManager.clearContacts();

                // 骞惰鏌ユ爲
                vehicles.parallelStream().forEach(vehicle -> {
                    vehicle.generateCollisionCandidates(globalSap, collisionManager, subDt * broadphaseRate);
                });

                // 鏋侀€熷浘鏌撹壊鍒嗘壒
                collisionManager.buildAndColorBatches();

                long tii3 = System.nanoTime();
                dyeCollisionMs += (tii3 - tii2) / 1_000_000.0;
            }

            long ti3 = System.nanoTime();

            // 3. 绐勯樁娈碉細澶氳溅/鍗曡溅鏅鸿兘鍔ㄦ€佸苟琛屾眰瑙?
            solveCachedContacts(subDt);

            long ti4 = System.nanoTime();
            softCollisionMs += (ti4 - ti3) / 1_000_000.0;

            // 4. 鐜鏂瑰潡纰版挒銆愯溅杈嗙骇鍒畬鍏ㄧ嫭绔嬪苟琛岋紒銆?
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
     * 鏅鸿兘鍔ㄦ€佸绾跨▼璋冨害鍣細鏍规嵁鎵规澶у皬鍔ㄦ€佸喅瀹氭槸鍚﹀紑鍚苟琛?
     */
    private void solveCachedContacts(float dt) {
        // 鍔ㄦ€侀槇鍊硷細浣犲彲浠ユ牴鎹祴璇曠粨鏋滆皟鏁淬€?
        final int PARALLEL_THRESHOLD = 1024;

        for (int b = 0; b < collisionManager.activeBatchCount; b++) {
            int currentBatchSize = collisionManager.batchSize[b];
            if (currentBatchSize == 0) continue;

            final int batchIndex = b;

            // 鏅鸿兘鍒嗗彂閫昏緫
            if (currentBatchSize < PARALLEL_THRESHOLD) {
                // 銆愬崟绾跨▼蹇溅閬撱€戠偣鏁板お灏戯紝鍞ら啋绾跨▼姹犲弽鑰屼簭鏈紝鐩存帴涓荤嚎绋嬬函鏁板閫熼€氾紒
                for (int idx = 0; idx < currentBatchSize; idx++) {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    resolveSingleContact(contactId, dt);
                }
            } else {
                // 銆愬鏍搁噸鐏姏缃戙€戠偣鏁板法澶э紙涓ラ噸杩炵幆鐩告挒锛夛紝鏀惧紑鏋烽攣锛屽绾跨▼鐏姏鍏ㄥ紑锛?
                java.util.stream.IntStream.range(0, currentBatchSize).parallel().forEach(idx -> {
                    int contactId = collisionManager.batches[batchIndex][idx];
                    resolveSingleContact(contactId, dt);
                });
            }
        }
    }

    /**
     * 琚彁鍙栧嚭鏉ョ殑绾暟瀛?PBD 瑙ｇ畻鍣?
     */
    private void resolveSingleContact(int contactId, float dt) {
        final float THICKNESS = 0.01f;
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
        if (pX < minX || pX > maxX || pY < minY || pY > maxY || pZ < minZ || pZ > maxZ) return;

        float abx = bx - ax, aby = by - ay, abz = bz - az;
        float acx = cx - ax, acy = cy - ay, acz = cz - az;
        float nx = aby * acz - abz * acy;
        float ny = abz * acx - abx * acz;
        float nz = abx * acy - aby * acx;

        float nLenSq = nx * nx + ny * ny + nz * nz;
        if (nLenSq < PhysicsWorld.KINDA_SMALL_NUMBER) return;
        float invNLen = 1.0f / (float) Math.sqrt(nLenSq);
        nx *= invNLen; ny *= invNLen; nz *= invNLen;

        float d00 = abx * abx + aby * aby + abz * abz;
        float d01 = abx * acx + aby * acy + abz * acz;
        float d11 = acx * acx + acy * acy + acz * acz;
        float denom = d00 * d11 - d01 * d01;
        if (denom < PhysicsWorld.KINDA_SMALL_NUMBER) return;
        float invDenom = 1.0f / denom;

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
        if (penetration <= 0.0f) return;

        float ppx = apx - distCurr * nx;
        float ppy = apy - distCurr * ny;
        float ppz = apz - distCurr * nz;

        float d20 = ppx * abx + ppy * aby + ppz * abz;
        float d21 = ppx * acx + ppy * acy + ppz * acz;

        float wB = (d11 * d20 - d01 * d21) * invDenom;
        float wC = (d00 * d21 - d01 * d20) * invDenom;
        float wA = 1.0f - wB - wC;

        final float TOLERANCE = -0.01f;
        if (!(wA >= TOLERANCE && wB >= TOLERANCE && wC >= TOLERANCE)) return;

        float effNx = nx * pushDir, effNy = ny * pushDir, effNz = nz * pushDir;

        float massNode = nVeh.nodes.mass[nHit];
        float massA = tVeh.nodes.mass[nA], massB = tVeh.nodes.mass[nB], massC = tVeh.nodes.mass[nC];

        float wTotal = (1.0f / massNode) + (wA * wA / massA) + (wB * wB / massB) + (wC * wC / massC);
        if (wTotal < PhysicsWorld.KINDA_SMALL_NUMBER) return;
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
    }
}
