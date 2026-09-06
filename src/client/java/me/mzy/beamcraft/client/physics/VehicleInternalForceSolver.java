package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;

/**
 * Per-vehicle internal-force solver.
 *
 * Holds the concrete implementation of one physics sub-step's internal force
 * phase for a single {@link SoftBodyVehicle}: tire pressure, every beam
 * constraint family (normal / support / bounded / LBeam / anisotropic),
 * torsion bars, slide nodes, plastic deformation with hardening, node
 * integration under gravity, the node velocity sanitizer, and the fixed
 * call order of the hydro updates, powertrain, brakes and coupler velocity
 * constraints. The owning {@code SoftBodyVehicle} keeps every container, all
 * add/reset/clear/finalize lifecycle methods, break-group coordination and the
 * public {@code solveInternalForces} entry points, which simply delegate here.
 *
 * The solver instance is owned by one vehicle and only mutates that vehicle's
 * own containers, so it stays safe under the world's per-vehicle parallelism.
 * It performs no per-sub-step allocation.
 */
public final class VehicleInternalForceSolver {
    private static final float KINDA_SMALL_NUMBER = SoftBodyVehicle.KINDA_SMALL_NUMBER;
    private static final float MAX_NODE_SPEED_SQ =
            SoftBodyVehicle.MAX_NODE_SPEED * SoftBodyVehicle.MAX_NODE_SPEED;

    private final SoftBodyVehicle v;

    public VehicleInternalForceSolver(SoftBodyVehicle vehicle) {
        this.v = vehicle;
    }

    /**
     * Runs the full internal-force sub-step exactly as the former
     * {@code SoftBodyVehicle.solveInternalForces} body did.
     */
    public void solve(float dt, float plasticRelaxation, ElectricSnapshot electricSnapshot) {
        NodeContainer nodes = v.nodes;
        float invDt = 1.0f / dt;

        v.hydros.update(dt, v.normalBeams,
                electricSnapshot == null ? ElectricSnapshot.EMPTY : electricSnapshot);
        v.torsionHydros.update(dt, v.torsionbars,
                electricSnapshot == null ? ElectricSnapshot.EMPTY : electricSnapshot);

        for (int i = 0; i < nodes.count; i++) {
            nodes.forceX[i] = 0.0f;
            nodes.forceY[i] = 0.0f;
            nodes.forceZ[i] = 0.0f;

            nodes.prevPosX[i] = nodes.posX[i];
            nodes.prevPosY[i] = nodes.posY[i];
            nodes.prevPosZ[i] = nodes.posZ[i];
        }

        // ==========================================
        // ==========================================
        solveTirePressure();

        // ==========================================
        // ==========================================
        solveNormalBeams(plasticRelaxation, invDt);

        solveSupportBeams(plasticRelaxation, invDt);

        solveBoundedBeams(plasticRelaxation, invDt);

        // ========== 4. LBeams ==========
        solveLBeams(plasticRelaxation, invDt);

        solveAnisotropicBeams(plasticRelaxation, invDt);

        // ==========================================
        // ==========================================
        solveTorsionBars(plasticRelaxation, invDt);

        // ==========================================
        // ==========================================
        solveSlideNodes(dt, invDt);

        // Powertrain only touches primitive physics state and is safe on the
        // dedicated worker. Run it before node integration so driven-wheel
        // torque participates in this substep.
        v.powertrain.solve(dt, electricSnapshot);
        v.wheels.applyBrakes(
                (float) electricSnapshot.get(v.brakeInputSignalId),
                (float) electricSnapshot.get(v.parkingBrakeInputSignalId),
                dt);

        // ==========================================
        // ==========================================
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.mass[i] < PhysicsWorld.KINDA_SMALL_NUMBER) continue;
            nodes.forceY[i] += PhysicsWorld.GRAVITY * nodes.mass[i];

            float invMass = 1.0f / nodes.mass[i];
            nodes.velX[i] += (nodes.forceX[i] * invMass) * dt;
            nodes.velY[i] += (nodes.forceY[i] * invMass) * dt;
            nodes.velZ[i] += (nodes.forceZ[i] * invMass) * dt;

            sanitizeNodeVelocity(i);
        }

        v.couplers.solveVelocityConstraints(nodes, dt);

        for (int i = 0; i < nodes.count; i++) {
            if (nodes.mass[i] < PhysicsWorld.KINDA_SMALL_NUMBER) continue;
            sanitizeNodeVelocity(i);

            nodes.posX[i] += nodes.velX[i] * dt;
            nodes.posY[i] += nodes.velY[i] * dt;
            nodes.posZ[i] += nodes.velZ[i] * dt;
        }
    }

    private void solveTirePressure() {
        NodeContainer nodes = v.nodes;
        WheelContainer wheels = v.wheels;
        TriangleContainer triangles = v.triangles;

        for (int w = 0; w < wheels.count; w++) {
            if (wheels.isDeflated[w]) continue;

            int start = wheels.tireTriangleIdxStart[w];
            int end = wheels.tireTriangleIdxEnd[w];
            if (start >= end || start == 0) continue;

            double currentVolume = wheels.prevVolume[w];
            if (currentVolume < KINDA_SMALL_NUMBER) continue;

            double p0_Pa = wheels.pressurePSI[w] * 6894.76;
            double absP0_Pa = p0_Pa + 101325.0;
            double currentAbsPressurePa = absP0_Pa * (wheels.initialVolume[w] / currentVolume);
            double pressureDiffPa = currentAbsPressurePa - 101325.0;

            double forceMultiplier = (pressureDiffPa * wheels.normalSign[w]) / 6.0;

            double nextVolumeSum = 0.0;

            for (int i = start; i <= end; i++) {
                int nA = triangles.node1[i];
                int nB = triangles.node2[i];
                int nC = triangles.node3[i];

                double ax = nodes.posX[nA], ay = nodes.posY[nA], az = nodes.posZ[nA];
                double bx = nodes.posX[nB], by = nodes.posY[nB], bz = nodes.posZ[nB];
                double cx = nodes.posX[nC], cy = nodes.posY[nC], cz = nodes.posZ[nC];

                double abx = bx - ax, aby = by - ay, abz = bz - az;
                double acx = cx - ax, acy = cy - ay, acz = cz - az;

                double nx = aby * acz - abz * acy;
                double ny = abz * acx - abx * acz;
                double nz = abx * acy - aby * acx;

                double fx = nx * forceMultiplier;
                double fy = ny * forceMultiplier;
                double fz = nz * forceMultiplier;

                nodes.forceX[nA] += fx; nodes.forceY[nA] += fy; nodes.forceZ[nA] += fz;
                nodes.forceX[nB] += fx; nodes.forceY[nB] += fy; nodes.forceZ[nB] += fz;
                nodes.forceX[nC] += fx; nodes.forceY[nC] += fy; nodes.forceZ[nC] += fz;

                nextVolumeSum += (ax * nx + ay * ny + az * nz);
            }

            wheels.normalSign[w] = (nextVolumeSum < 0.0) ? -1.0f : 1.0f;
            wheels.prevVolume[w] = (float) Math.abs(nextVolumeSum / 6.0);
        }
    }

    /**
     * Returns the permanent rest-state change for an elastic load beyond its yield load.
     * The exponential relaxation is timestep independent and never overshoots the
     * current hardening surface for a single solve.
     */
    private static float plasticDeltaFromExcess(float excessLoad, float yieldLoad, float maxYieldLoad,
                                                float stiffness, float relaxation) {
        float invStiffness = 1.0f / stiffness;
        float remainingHardening = Math.max(0.0f, maxYieldLoad - yieldLoad);
        float targetDelta;
        if (remainingHardening > 0.0f) {
            if (excessLoad <= 2.0f * remainingHardening) {
                targetDelta = 0.5f * excessLoad * invStiffness;
            } else {
                targetDelta = (excessLoad - remainingHardening) * invStiffness;
            }
        } else {
            targetDelta = excessLoad * invStiffness;
        }
        return relaxation * targetDelta;
    }

    private static double plasticDeltaFromExcess(double excessLoad, double yieldLoad, double maxYieldLoad,
                                                 double stiffness, double relaxation) {
        double invStiffness = 1.0 / stiffness;
        double remainingHardening = Math.max(0.0, maxYieldLoad - yieldLoad);
        double targetDelta;
        if (remainingHardening > 0.0) {
            if (excessLoad <= 2.0 * remainingHardening) {
                targetDelta = 0.5 * excessLoad * invStiffness;
            } else {
                targetDelta = (excessLoad - remainingHardening) * invStiffness;
            }
        } else {
            targetDelta = excessLoad * invStiffness;
        }
        return relaxation * targetDelta;
    }

    private static float hardenDeform(float currentDeform, float maxDeform, float stiffness, float permanentDelta) {
        float remainingHardening = Math.max(0.0f, maxDeform - currentDeform);
        return currentDeform + Math.min(remainingHardening, stiffness * permanentDelta);
    }

    private static double hardenDeform(double currentDeform, double maxDeform,
                                       double stiffness, double permanentDelta) {
        double remainingHardening = Math.max(0.0, maxDeform - currentDeform);
        return currentDeform + Math.min(remainingHardening, stiffness * permanentDelta);
    }

    private void solveNormalBeams(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        BeamContainer normalBeams = v.normalBeams;

        for (int i = 0; i < normalBeams.count; i++) {
            if (normalBeams.broken[i]) continue;

            int n1 = normalBeams.node1[i];
            int n2 = normalBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float actuationRatio = Math.max(KINDA_SMALL_NUMBER, normalBeams.actuationRatio[i]);
            float restL = normalBeams.effectiveRestLength(i);
            float activeSpring = normalBeams.spring[i];
            float springForce = normalBeams.spring[i] * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = normalBeams.damp[i];
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > normalBeams.strength[i]) {
                v.breakBeamAt(normalBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > normalBeams.deform[i]) {
                float maxDeform = normalBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - normalBeams.deform[i],
                        normalBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float neutralRestLength = normalBeams.restLength[i];
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            neutralRestLength + Math.signum(springForce) * permanentDelta / actuationRatio);
                    float appliedDelta = Math.abs(newRestLength - neutralRestLength) * actuationRatio;
                    normalBeams.restLength[i] = newRestLength;
                    normalBeams.deform[i] = hardenDeform(normalBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveSupportBeams(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        BeamContainer supportBeams = v.supportBeams;

        for (int i = 0; i < supportBeams.count; i++) {
            if (supportBeams.broken[i]) continue;

            int n1 = supportBeams.node1[i];
            int n2 = supportBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = supportBeams.restLength[i];

            if (dist > restL) continue;

            float activeSpring = supportBeams.spring[i];
            float springForce = activeSpring * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = supportBeams.damp[i];
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > supportBeams.strength[i]) {
                v.breakBeamAt(supportBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > supportBeams.deform[i]) {
                float maxDeform = supportBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - supportBeams.deform[i],
                        supportBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    supportBeams.restLength[i] = newRestLength;
                    supportBeams.deform[i] = hardenDeform(supportBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveBoundedBeams(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        BoundedBeamContainer boundedBeams = v.boundedBeams;

        for (int i = 0; i < boundedBeams.count; i++) {
            if (boundedBeams.broken[i]) continue;

            int n1 = boundedBeams.node1[i];
            int n2 = boundedBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = boundedBeams.restLength[i];
            float activeSpring = boundedBeams.spring[i];
            float springForce = activeSpring * (dist - restL);
            float plasticStiffness = activeSpring;

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = boundedBeams.damp[i];
            float split = boundedBeams.dampVelocitySplit[i];
            boolean isRebound = relVel > 0;
            boolean isFast = Math.abs(relVel) > split;
            if (isRebound) {
                activeDamp = isFast ? boundedBeams.dampReboundFast[i] : boundedBeams.dampRebound[i];
            } else {
                if (isFast) activeDamp = boundedBeams.dampFast[i];
            }

            float shortBoundary, longBoundary;

            if (boundedBeams.shortBoundRange[i] >= 0) {
                shortBoundary = restL - boundedBeams.shortBoundRange[i];
            } else {
                shortBoundary = restL * (1.0f - boundedBeams.shortBound[i]);
            }

            if (boundedBeams.longBoundRange[i] >= 0) {
                longBoundary = restL + boundedBeams.longBoundRange[i];
            } else {
                longBoundary = restL * (1.0f + boundedBeams.longBound[i]);
            }

            float limitSpring = boundedBeams.limitSpring[i];

            if (dist < shortBoundary) {
                springForce += limitSpring * (dist - shortBoundary);
                plasticStiffness += limitSpring * (boundedBeams.shortBoundRange[i] >= 0
                        ? 1.0f : 1.0f - boundedBeams.shortBound[i]);
                activeDamp = boundedBeams.limitDamp[i];
            } else if (dist > longBoundary) {
                springForce += limitSpring * (dist - longBoundary);
                plasticStiffness += limitSpring * (boundedBeams.longBoundRange[i] >= 0
                        ? 1.0f : 1.0f + boundedBeams.longBound[i]);
                activeDamp = boundedBeams.limitDamp[i];
            }

            float totalForce = springForce + (relVel * activeDamp);
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > boundedBeams.strength[i]) {
                v.breakBeamAt(boundedBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (plasticStiffness > KINDA_SMALL_NUMBER && absSpringForce > boundedBeams.deform[i]) {
                float maxDeform = boundedBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - boundedBeams.deform[i],
                        boundedBeams.deform[i], maxDeform,
                        plasticStiffness, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    boundedBeams.restLength[i] = newRestLength;
                    boundedBeams.deform[i] = hardenDeform(boundedBeams.deform[i], maxDeform,
                            plasticStiffness, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveLBeams(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        LBeamContainer lBeams = v.lBeams;

        for (int i = 0; i < lBeams.count; i++) {
            if (lBeams.broken[i]) continue;

            int n1 = lBeams.node1[i];
            int n2 = lBeams.node2[i];
            int n3 = lBeams.node3[i];

            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];

            double dx13 = x1 - x3, dy13 = y1 - y3, dz13 = z1 - z3;
            double l1Sq = dx13*dx13 + dy13*dy13 + dz13*dz13;

            double dx23 = x2 - x3, dy23 = y2 - y3, dz23 = z2 - z3;
            double l2Sq = dx23*dx23 + dy23*dy23 + dz23*dz23;

            double dx12 = x2 - x1, dy12 = y2 - y1, dz12 = z2 - z1;
            double distSq = dx12*dx12 + dy12*dy12 + dz12*dz12;

            if (l1Sq < KINDA_SMALL_NUMBER || l2Sq < KINDA_SMALL_NUMBER || distSq < KINDA_SMALL_NUMBER) continue;

            double l1 = Math.sqrt(l1Sq);
            double l2 = Math.sqrt(l2Sq);
            double dist = Math.sqrt(distSq);

            double invL1 = 1.0 / l1;
            double invL2 = 1.0 / l2;
            double invDist = 1.0 / dist;

            double cosTheta0 = lBeams.restCosTheta[i];
            double targetDistSq = l1Sq + l2Sq - 2.0 * l1 * l2 * cosTheta0;
            if (targetDistSq < KINDA_SMALL_NUMBER) continue;
            double targetDist = Math.sqrt(targetDistSq);
            double invTargetDist = 1.0 / targetDist;

            double g1 = (l1 - l2 * cosTheta0) * invTargetDist;
            double g2 = (l2 - l1 * cosTheta0) * invTargetDist;

            double vx1 = nodes.velX[n1], vy1 = nodes.velY[n1], vz1 = nodes.velZ[n1];
            double vx2 = nodes.velX[n2], vy2 = nodes.velY[n2], vz2 = nodes.velZ[n2];
            double vx3 = nodes.velX[n3], vy3 = nodes.velY[n3], vz3 = nodes.velZ[n3];

            double v13x = vx1 - vx3, v13y = vy1 - vy3, v13z = vz1 - vz3;
            double l1Dot = (v13x*dx13 + v13y*dy13 + v13z*dz13) * invL1;

            double v23x = vx2 - vx3, v23y = vy2 - vy3, v23z = vz2 - vz3;
            double l2Dot = (v23x*dx23 + v23y*dy23 + v23z*dz23) * invL2;

            double targetDistDot = g1 * l1Dot + g2 * l2Dot;

            double v12x = vx2 - vx1, v12y = vy2 - vy1, v12z = vz2 - vz1;
            double distDot = (v12x*dx12 + v12y*dy12 + v12z*dz12) * invDist;

            double dampVel = distDot - targetDistDot;

            double activeSpring = lBeams.spring[i];
            double springForce = activeSpring * (dist - targetDist);
            double dampForce = lBeams.damp[i] * dampVel;
            double totalForce = springForce + dampForce;

            double absTotalForce = Math.abs(totalForce);
            if (absTotalForce > lBeams.strength[i]) {
                v.breakBeamAt(lBeams, i);
                continue;
            }

            double absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > lBeams.deform[i]) {
                float maxDeform = lBeams.maxDeform[i];
                double permanentDelta = plasticDeltaFromExcess(absSpringForce - lBeams.deform[i],
                        lBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0) {
                    double newTargetDist = targetDist + Math.signum(springForce) * permanentDelta;
                    newTargetDist = Math.clamp(newTargetDist, Math.abs(l1 - l2), l1 + l2);
                    double appliedDelta = Math.abs(newTargetDist - targetDist);
                    double newRestCos = (l1Sq + l2Sq - newTargetDist * newTargetDist) / (2.0 * l1 * l2);
                    lBeams.restCosTheta[i] = (float) Math.clamp(newRestCos, -1.0, 1.0);
                    lBeams.deform[i] = (float) hardenDeform(lBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            double u13x = dx13 * invL1,   u13y = dy13 * invL1,   u13z = dz13 * invL1;
            double u23x = dx23 * invL2,   u23y = dy23 * invL2,   u23z = dz23 * invL2;
            double u12x = dx12 * invDist, u12y = dy12 * invDist, u12z = dz12 * invDist;

            double f1x = totalForce * (u12x + g1 * u13x);
            double f1y = totalForce * (u12y + g1 * u13y);
            double f1z = totalForce * (u12z + g1 * u13z);

            double f2x = totalForce * (-u12x + g2 * u23x);
            double f2y = totalForce * (-u12y + g2 * u23y);
            double f2z = totalForce * (-u12z + g2 * u23z);

            double f3x = totalForce * (-g1 * u13x - g2 * u23x);
            double f3y = totalForce * (-g1 * u13y - g2 * u23y);
            double f3z = totalForce * (-g1 * u13z - g2 * u23z);

            nodes.forceX[n1] += f1x; nodes.forceY[n1] += f1y; nodes.forceZ[n1] += f1z;
            nodes.forceX[n2] += f2x; nodes.forceY[n2] += f2y; nodes.forceZ[n2] += f2z;
            nodes.forceX[n3] += f3x; nodes.forceY[n3] += f3y; nodes.forceZ[n3] += f3z;
        }
    }

    private void solveAnisotropicBeams(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        AnisotropicBeamContainer anisotropicBeams = v.anisotropicBeams;

        for (int i = 0; i < anisotropicBeams.count; i++) {
            if (anisotropicBeams.broken[i]) continue;

            int n1 = anisotropicBeams.node1[i];
            int n2 = anisotropicBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = anisotropicBeams.restLength[i];

            float activeSpring = anisotropicBeams.spring[i];
            float activeDamp   = anisotropicBeams.damp[i];

            if (dist > restL) {
                float expSpring = anisotropicBeams.springExpansion[i];
                float expDamp   = anisotropicBeams.dampExpansion[i];
                float tZoneRatio = anisotropicBeams.transitionZone[i];

                if (tZoneRatio > KINDA_SMALL_NUMBER) {
                    float absoluteTZone = tZoneRatio * restL;
                    float stretch = dist - restL;

                    if (stretch >= absoluteTZone) {
                        activeSpring = expSpring;
                        activeDamp   = expDamp;
                    } else {
                        float factor = stretch / absoluteTZone;
                        activeSpring += (expSpring - activeSpring) * factor;
                        activeDamp   += (expDamp   - activeDamp)   * factor;
                    }
                } else {
                    activeSpring = expSpring;
                    activeDamp   = expDamp;
                }
            }

            float springForce = activeSpring * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > anisotropicBeams.strength[i]) {
                v.breakBeamAt(anisotropicBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > anisotropicBeams.deform[i]) {
                float maxDeform = anisotropicBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - anisotropicBeams.deform[i],
                        anisotropicBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    anisotropicBeams.restLength[i] = newRestLength;
                    anisotropicBeams.deform[i] = hardenDeform(anisotropicBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveTorsionBars(float plasticRelaxation, float invDt) {
        NodeContainer nodes = v.nodes;
        TorsionBarContainer torsionbars = v.torsionbars;

        for (int i = 0; i < torsionbars.count; i++) {
            if (torsionbars.broken[i]) continue;

            int n1 = torsionbars.node1[i], n2 = torsionbars.node2[i], n3 = torsionbars.node3[i], n4 = torsionbars.node4[i];

            if (nodes.mass[n1] < KINDA_SMALL_NUMBER ||
                    nodes.mass[n2] < KINDA_SMALL_NUMBER ||
                    nodes.mass[n3] < KINDA_SMALL_NUMBER ||
                    nodes.mass[n4] < KINDA_SMALL_NUMBER) {
                torsionbars.broken[i] = true;
                continue;
            }

            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];
            double x4 = nodes.posX[n4], y4 = nodes.posY[n4], z4 = nodes.posZ[n4];

            double b1x = x2 - x1, b1y = y2 - y1, b1z = z2 - z1;
            double b2x = x3 - x2, b2y = y3 - y2, b2z = z3 - z2;
            double b3x = x4 - x3, b3y = y4 - y3, b3z = z4 - z3;

            double c1x = b1y * b2z - b1z * b2y;
            double c1y = b1z * b2x - b1x * b2z;
            double c1z = b1x * b2y - b1y * b2x;

            double c2x = b2y * b3z - b2z * b3y;
            double c2y = b2z * b3x - b2x * b3z;
            double c2z = b2x * b3y - b2y * b3x;

            double c1_sq = c1x*c1x + c1y*c1y + c1z*c1z;
            double c2_sq = c2x*c2x + c2y*c2y + c2z*c2z;
            double b2_sq = b2x*b2x + b2y*b2y + b2z*b2z;

            if (Double.isNaN(c1_sq) || Double.isNaN(c2_sq) ||  Double.isNaN(b2_sq)) {
                torsionbars.broken[i] = true;
                continue;
            }

            double b2_mag = Math.sqrt(b2_sq);

            double c1Xc2_x = c1y * c2z - c1z * c2y;
            double c1Xc2_y = c1z * c2x - c1x * c2z;
            double c1Xc2_z = c1x * c2y - c1y * c2x;

            double dot1 = (c1Xc2_x * b2x + c1Xc2_y * b2y + c1Xc2_z * b2z) / b2_mag;
            double dot2 = c1x * c2x + c1y * c2y + c1z * c2z;
            double currentAngle = Math.atan2(dot1, dot2);

            float precompressionTarget = torsionbars.precompressionAngle[i];
            float precompressionState = torsionbars.precompressionState[i];
            if (precompressionState != precompressionTarget) {
                float time = torsionbars.precompressionTime[i];
                float maxDelta = time <= KINDA_SMALL_NUMBER
                        ? Float.POSITIVE_INFINITY
                        : Math.abs(precompressionTarget) / (time * invDt);
                if (precompressionState < precompressionTarget) {
                    precompressionState = Math.min(precompressionState + maxDelta, precompressionTarget);
                } else {
                    precompressionState = Math.max(precompressionState - maxDelta, precompressionTarget);
                }
                torsionbars.precompressionState[i] = precompressionState;
            }

            double targetAngle = torsionbars.restAngle[i]
                    + precompressionState + torsionbars.actuationAngle[i];
            double deltaAngle = currentAngle - targetAngle;
            while (deltaAngle > Math.PI) deltaAngle -= Math.PI * 2;
            while (deltaAngle < -Math.PI) deltaAngle += Math.PI * 2;

            double g1_factor = b2_mag / c1_sq;
            double g4_factor = -b2_mag / c2_sq;

            double g1x = g1_factor * c1x, g1y = g1_factor * c1y, g1z = g1_factor * c1z;
            double g4x = g4_factor * c2x, g4y = g4_factor * c2y, g4z = g4_factor * c2z;

            double b1_dot_b2_div_sq = (b1x*b2x + b1y*b2y + b1z*b2z) / b2_sq;
            double b3_dot_b2_div_sq = (b3x*b2x + b3y*b2y + b3z*b2z) / b2_sq;

            double g2x = -g1x * b1_dot_b2_div_sq + g4x * b3_dot_b2_div_sq - g1x;
            double g2y = -g1y * b1_dot_b2_div_sq + g4y * b3_dot_b2_div_sq - g1y;
            double g2z = -g1z * b1_dot_b2_div_sq + g4z * b3_dot_b2_div_sq - g1z;

            double g3x = -g1x - g2x - g4x;
            double g3y = -g1y - g2y - g4y;
            double g3z = -g1z - g2z - g4z;

            double g1_sq_val = g1x*g1x + g1y*g1y + g1z*g1z;
            double g2_sq_val = g2x*g2x + g2y*g2y + g2z*g2z;
            double g3_sq_val = g3x*g3x + g3y*g3y + g3z*g3z;
            double g4_sq_val = g4x*g4x + g4y*g4y + g4z*g4z;

            double invGenMass = (g1_sq_val / nodes.mass[n1]) + (g2_sq_val / nodes.mass[n2]) +
                    (g3_sq_val / nodes.mass[n3]) + (g4_sq_val / nodes.mass[n4]);

            double genMass = 1.0 / invGenMass;

            double maxSafeSpring = genMass * invDt * invDt;
            double maxSafeDamp = genMass * invDt;

            float configuredSpring = deltaAngle >= 0.0
                    ? torsionbars.spring[i] : torsionbars.spring2[i];
            float configuredDamp = deltaAngle >= 0.0
                    ? torsionbars.damp[i] : torsionbars.damp2[i];
            double activeSpring = Math.min(configuredSpring, maxSafeSpring);
            double activeDamp = Math.min(configuredDamp, maxSafeDamp);

            double omega = (g1x*nodes.velX[n1] + g1y*nodes.velY[n1] + g1z*nodes.velZ[n1]) +
                    (g2x*nodes.velX[n2] + g2y*nodes.velY[n2] + g2z*nodes.velZ[n2]) +
                    (g3x*nodes.velX[n3] + g3y*nodes.velY[n3] + g3z*nodes.velZ[n3]) +
                    (g4x*nodes.velX[n4] + g4y*nodes.velY[n4] + g4z*nodes.velZ[n4]);

            double elasticTorque = activeSpring * deltaAngle;
            double torque = elasticTorque - (activeDamp * omega);

            double absTorque = Math.abs(torque);
            if (Double.isNaN(torque) || absTorque > torsionbars.strength[i]) {
                torsionbars.broken[i] = true;
                continue;
            }
            double absElasticTorque = Math.abs(elasticTorque);
            if (activeSpring > KINDA_SMALL_NUMBER && absElasticTorque > torsionbars.deform[i]) {
                double maxDeform = torsionbars.strength[i];
                double permanentDelta = plasticDeltaFromExcess(absElasticTorque - torsionbars.deform[i],
                        torsionbars.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0) {
                    torsionbars.restAngle[i] += Math.signum(elasticTorque) * permanentDelta;
                    torsionbars.deform[i] = (float) hardenDeform(torsionbars.deform[i], maxDeform,
                            activeSpring, permanentDelta);
                    while (torsionbars.restAngle[i] > Math.PI) torsionbars.restAngle[i] -= Math.PI * 2;
                    while (torsionbars.restAngle[i] < -Math.PI) torsionbars.restAngle[i] += Math.PI * 2;
                }
            }

            nodes.forceX[n1] += torque * g1x; nodes.forceY[n1] += torque * g1y; nodes.forceZ[n1] += torque * g1z;
            nodes.forceX[n2] += torque * g2x; nodes.forceY[n2] += torque * g2y; nodes.forceZ[n2] += torque * g2z;
            nodes.forceX[n3] += torque * g3x; nodes.forceY[n3] += torque * g3y; nodes.forceZ[n3] += torque * g3z;
            nodes.forceX[n4] += torque * g4x; nodes.forceY[n4] += torque * g4y; nodes.forceZ[n4] += torque * g4z;
        }
    }

    private void solveSlideNodes(float dt, float invDt) {
        NodeContainer nodes = v.nodes;
        SlideNodeContainer slidenodes = v.slidenodes;

        for (int i = 0; i < slidenodes.count; i++) {
            int nId = slidenodes.nodeId[i];
            int aId = slidenodes.railA[i];
            int bId = slidenodes.railB[i];

            double nx = nodes.posX[nId], ny = nodes.posY[nId], nz = nodes.posZ[nId];
            double ax = nodes.posX[aId], ay = nodes.posY[aId], az = nodes.posZ[aId];
            double bx = nodes.posX[bId], by = nodes.posY[bId], bz = nodes.posZ[bId];

            double abx = bx - ax, aby = by - ay, abz = bz - az;
            double anx = nx - ax, any = ny - ay, anz = nz - az;

            double ab_sq = abx*abx + aby*aby + abz*abz;
            if (ab_sq < KINDA_SMALL_NUMBER) continue;

            double t = (anx*abx + any*aby + anz*abz) / ab_sq;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;

            double px = ax + t * abx;
            double py = ay + t * aby;
            double pz = az + t * abz;

            double pnx = nx - px, pny = ny - py, pnz = nz - pz;
            double dist = Math.sqrt(pnx*pnx + pny*pny + pnz*pnz);

            // Anti zero-divide protection
            if (dist < KINDA_SMALL_NUMBER) {
                dist = KINDA_SMALL_NUMBER;
            }

            double invDist = 1.0 / dist;
            double nDirX = pnx * invDist, nDirY = pny * invDist, nDirZ = pnz * invDist;

            double mN = nodes.mass[nId];
            double mRail = nodes.mass[aId] + nodes.mass[bId];
            if (mN < KINDA_SMALL_NUMBER ||  mRail < KINDA_SMALL_NUMBER) continue;

            double reducedMass = (mN * mRail) / (mN + mRail);
            double maxSafeSpring = reducedMass * invDt * invDt;
            double activeSpring = Math.min(slidenodes.spring[i], maxSafeSpring);

            // Keep original rest offset, no forced snap
            double springForce = activeSpring * (dist - slidenodes.restDist[i]);

            // Rail point velocity interpolation
            double vpx = nodes.velX[aId] * (1 - t) + nodes.velX[bId] * t;
            double vpy = nodes.velY[aId] * (1 - t) + nodes.velY[bId] * t;
            double vpz = nodes.velZ[aId] * (1 - t) + nodes.velZ[bId] * t;

            double relVel = (nodes.velX[nId] - vpx) * nDirX + (nodes.velY[nId] - vpy) * nDirY + (nodes.velZ[nId] - vpz) * nDirZ;

            double activeDamp = Math.min(slidenodes.damp[i], reducedMass * invDt);
            double dampForce = activeDamp * relVel;

            // Apply slide constraint force
            double fx = (springForce + dampForce) * nDirX;
            double fy = (springForce + dampForce) * nDirY;
            double fz = (springForce + dampForce) * nDirZ;

            nodes.forceX[nId] -= fx; nodes.forceY[nId] -= fy; nodes.forceZ[nId] -= fz;
            nodes.forceX[aId] += fx * (1 - t); nodes.forceY[aId] += fy * (1 - t); nodes.forceZ[aId] += fz * (1 - t);
            nodes.forceX[bId] += fx * t;       nodes.forceY[bId] += fy * t;       nodes.forceZ[bId] += fz * t;
        }
    }

    private void sanitizeNodeVelocity(int i) {
        NodeContainer nodes = v.nodes;
        if (!Float.isFinite(nodes.velX[i]) || !Float.isFinite(nodes.velY[i]) || !Float.isFinite(nodes.velZ[i])) {
            nodes.velX[i] = 0.0f;
            nodes.velY[i] = 0.0f;
            nodes.velZ[i] = 0.0f;
            return;
        }
        float speedSq = nodes.velX[i] * nodes.velX[i]
                + nodes.velY[i] * nodes.velY[i]
                + nodes.velZ[i] * nodes.velZ[i];
        if (speedSq > MAX_NODE_SPEED_SQ) {
            float scale = SoftBodyVehicle.MAX_NODE_SPEED / (float) Math.sqrt(speedSq);
            nodes.velX[i] *= scale;
            nodes.velY[i] *= scale;
            nodes.velZ[i] *= scale;
        }
    }
}
