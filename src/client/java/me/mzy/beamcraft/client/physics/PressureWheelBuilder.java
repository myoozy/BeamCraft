/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see
 * LICENSES/bCDDL-1.1.txt.
 *
 * Pressure-wheel construction is adapted from BeamNG.drive
 * lua/common/jbeam/sections/wheels.lua. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors.
 */
package me.mzy.beamcraft.client.physics;

import java.util.List;

/** Builds the authored node, beam and triangle topology for pressure wheels. */
final class PressureWheelBuilder {
    private final WheelContainer wheels;
    private final SoftBodyVehicle vehicle;

    PressureWheelBuilder(WheelContainer wheels, SoftBodyVehicle vehicle) {
        this.wheels = wheels;
        this.vehicle = vehicle;
    }

    public void generateHub(PhysicsSpecs.WheelHubSpec spec) {
        buildHub(spec);
    }

    private void buildHub(PhysicsSpecs.WheelHubSpec spec) {
        String wheelName = spec.wheelName();
        int n1 = spec.node1();
        int n2 = spec.node2();
        Integer nodeS = spec.nodeS();
        int wheelDir = spec.wheelDir();
        int rays = spec.rays();
        double radius = spec.radius();
        double width = spec.width();
        double offset = spec.offset();
        double nodeWeight = spec.nodeWeight();
        double frictionCoef = spec.frictionCoef();
        double hubBeamDeform = spec.hubBeamDeform();
        double hubBeamStrength = spec.hubBeamStrength();
        double hubTreadSpring = spec.hubTreadSpring();
        double hubTreadDamp = spec.hubTreadDamp();
        double hubPeriphSpring = spec.hubPeriphSpring();
        double hubPeriphDamp = spec.hubPeriphDamp();
        double hubSideSpring = spec.hubSideSpring();
        double hubSideDamp = spec.hubSideDamp();
        double hubReinfSpring = spec.hubReinfSpring();
        double hubReinfDamp = spec.hubReinfDamp();
        String hubGroup = spec.hubGroup();
        wheels.ensureWheelCapacity();
        int wIdx = wheels.count;
        wheels.nameToIndex.put(wheelName, wIdx);

        wheels.name[wIdx] = wheelName;
        wheels.node1[wIdx] = n1;
        wheels.node2[wIdx] = n2;
        wheels.wheelDir[wIdx] = wheelDir >= 0 ? 1 : -1;
        wheels.numRays[wIdx] = rays > 0 ? Math.min(rays, WheelContainer.MAX_RAYS) : WheelContainer.MAX_RAYS;
        wheels.hubRadius[wIdx] = (float) radius;
        wheels.isDeflated[wIdx] = false;

        // Default every pressure-wheel reaction to "undefined"; the parser fills the
        // drivetrain nodes via setReactionNodes() when the wheel defines them. The nodeArm
        // header column is the braking lever arm and arrives through the hub spec.
        wheels.torqueCouplingNode[wIdx] = -1;
        wheels.torqueArmNode[wIdx] = -1;
        wheels.torqueArm2Node[wIdx] = -1;
        wheels.nodeCouplingNode[wIdx] = -1;
        wheels.nodeArmNode[wIdx] = spec.nodeArm() != null ? spec.nodeArm() : -1;

        int partId = vehicle.nodes.partId[n1];
        int baseOffset = wIdx * WheelContainer.MAX_RAYS;

        // 1. 基底向量计算
        double[] uX = {0}, uY = {0}, uZ = {0};
        double[] vX = {0}, vY = {0}, vZ = {0};
        double[] axisX = {0}, axisY = {0}, axisZ = {0};
        calculateWheelBasis(n1, n2, wheelDir, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        // 1. 算出 n1 和 n2 的物理中点
        double midX = (vehicle.nodes.posX[n1] + vehicle.nodes.posX[n2]) * 0.5;
        double midY = (vehicle.nodes.posY[n1] + vehicle.nodes.posY[n2]) * 0.5;
        double midZ = (vehicle.nodes.posZ[n1] + vehicle.nodes.posZ[n2]) * 0.5;

        // 2. 基于中点施加 Offset 偏距 (减号保持不变：axisX 指向外侧，减去 offset 即向外)
        double centerX = midX - axisX[0] * offset;
        double centerY = midY - axisY[0] * offset;
        double centerZ = midZ - axisZ[0] * offset;

        // 2. 生成 Hub 节点
        //
        // The two rings are offset by half a ray, matching addPressureWheel: it
        // advances its ray vector by 2*pi/(numRays*2) between the two nodes of each
        // iteration, so the node2-side ring sits on the ray angles and the
        // node1-side ring sits half a step ahead. Aligning the rings instead changes
        // the length and direction of every across-width beam.
        double hubStep = (2.0 * Math.PI) / rays;
        double hubHalfStep = hubStep * 0.5;
        for (int i = 0; i < rays; i++) {
            double inAngle = i * hubStep;
            double outAngle = inAngle + hubHalfStep;
            double inCos = Math.cos(inAngle);
            double inSin = Math.sin(inAngle);
            double outCos = Math.cos(outAngle);
            double outSin = Math.sin(outAngle);

            // 内外圈各占 width 的一半。in = n2 侧（角度未偏移），out = n1 侧（偏移半格）
            double inX = centerX + (uX[0] * inCos + vX[0] * inSin) * radius - axisX[0] * (width * 0.5);
            double inY = centerY + (uY[0] * inCos + vY[0] * inSin) * radius - axisY[0] * (width * 0.5);
            double inZ = centerZ + (uZ[0] * inCos + vZ[0] * inSin) * radius - axisZ[0] * (width * 0.5);

            double outX = centerX + (uX[0] * outCos + vX[0] * outSin) * radius + axisX[0] * (width * 0.5);
            double outY = centerY + (uY[0] * outCos + vY[0] * outSin) * radius + axisY[0] * (width * 0.5);
            double outZ = centerZ + (uZ[0] * outCos + vZ[0] * outSin) * radius + axisZ[0] * (width * 0.5);

            // 生成物理节点
            wheels.hubInnerNodes[baseOffset + i] = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_hub_in_" + i, (float) inX, (float) inY, (float) inZ, (float) nodeWeight,
                    (float) frictionCoef, 0.0f, partId,
                    true, false, List.of(hubGroup)));

            wheels.hubOuterNodes[baseOffset + i]  = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_hub_out_" + i, (float) outX, (float) outY, (float) outZ, (float) nodeWeight,
                    (float) frictionCoef, 0.0f, partId,
                    true, false, List.of(hubGroup)));
        }

        // 3. 生成物理拓扑 (Beams)
        //
        // Family layout follows addPressureWheel, translated into this naming where
        // hubInner is the n2 side and hubOuter the n1 side:
        //   hubTread      : hubInner_i->hubOuter_i and hubOuter_i->hubInner_{i+1}
        //   hubPeriphery  : the two rings
        //   hubSide       : each ring to its own axle node
        //   hubReinf      : each ring to the opposite axle node
        //   hubStabilizer : hubInner_i -> nodeS (node2 side only, when present)
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;
            int hInCur = wheels.hubInnerNodes[baseOffset + i], hInNext = wheels.hubInnerNodes[baseOffset + next];
            int hOutCur = wheels.hubOuterNodes[baseOffset + i], hOutNext = wheels.hubOuterNodes[baseOffset + next];

            // ================= 1. 轮辋胎面 (Tread) =================
            // 同射线的跨宽度支撑，加一格斜撑，两者构成 BeamNG 的胎面
            addFastBeam(hInCur, hOutCur, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hOutCur, hInNext, hubTreadSpring, hubTreadDamp, hubBeamDeform, hubBeamStrength);

            // ================= 2. 圆周环 (Periphery) =================
            int periInIdx = addFastBeam(hInCur, hInNext, hubPeriphSpring, hubPeriphDamp, hubBeamDeform, hubBeamStrength);
            int periOutIdx = addFastBeam(hOutCur, hOutNext, hubPeriphSpring, hubPeriphDamp, hubBeamDeform, hubBeamStrength);
            vehicle.normalBeams.bindToTire(periInIdx, wIdx);
            vehicle.normalBeams.bindToTire(periOutIdx, wIdx);

            // ================= 3. 辐条 (Spokes) =================
            // 直连辐条：每圈连自己那一侧的轴节点
            addFastBeam(hInCur, n2, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hOutCur, n1, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);

            // 交叉辐条：每圈连对侧的轴节点
            addFastBeam(hInCur, n1, hubReinfSpring, hubReinfDamp, hubBeamDeform, hubBeamStrength);
            addFastBeam(hOutCur, n2, hubReinfSpring, hubReinfDamp, hubBeamDeform, hubBeamStrength);

            // ================= 4. 稳定节点支撑 (nodeS) =================
            // BeamNG 只把 node2 侧那一圈连到稳定节点
            if (nodeS != null) {
                addFastBeam(hInCur, nodeS, hubSideSpring, hubSideDamp, hubBeamDeform, hubBeamStrength);
            }
        }

        wheels.count++;
    }

    /**
     * 生成轮胎 (Tire)
     */
    public void generateTire(PhysicsSpecs.WheelTireSpec spec) {
        buildTire(spec);
    }

    private void buildTire(PhysicsSpecs.WheelTireSpec spec) {
        String wheelName = spec.wheelName();
        int n1 = spec.node1();
        int n2 = spec.node2();
        int wheelDir = spec.wheelDir();
        int rays = spec.rays();
        double radius = spec.radius();
        double width = spec.width();
        double offset = spec.offset();
        double nodeWeight = spec.nodeWeight();
        double frictionCoef = spec.frictionCoef();
        double pressurePSI = spec.pressurePSI();
        double slidingFrictionCoef = spec.slidingFrictionCoef();
        double stribeckVelMult = spec.stribeckVelMult();
        double stribeckExponent = spec.stribeckExponent();
        double treadCoef = spec.treadCoef();
        double noLoadCoef = spec.noLoadCoef();
        double loadSensitivitySlope = spec.loadSensitivitySlope();
        double fullLoadCoef = spec.fullLoadCoef();
        double softnessCoef = spec.softnessCoef();
        double treadSpring = spec.treadSpring();
        double treadDamp = spec.treadDamp();
        double treadDeform = spec.treadDeform();
        double treadStrength = spec.treadStrength();
        double periSpring = spec.periSpring();
        double periDamp = spec.periDamp();
        double periDeform = spec.periDeform();
        double periStrength = spec.periStrength();
        double sideSpring = spec.sideSpring();
        double sideDamp = spec.sideDamp();
        double sideSpringExp = spec.sideSpringExp();
        double sideDampExp = spec.sideDampExp();
        double sideTransZone = spec.sideTransZone();
        double sideDeform = spec.sideDeform();
        double sideStrength = spec.sideStrength();
        double reinfSpring = spec.reinfSpring();
        double reinfDamp = spec.reinfDamp();
        double reinfDeform = spec.reinfDeform();
        double reinfStrength = spec.reinfStrength();
        double treadReinfSpring = spec.treadReinfSpring();
        double treadReinfDamp = spec.treadReinfDamp();
        double periReinfSpring = spec.periReinfSpring();
        double periReinfDamp = spec.periReinfDamp();
        double sideReinfSpring = spec.sideReinfSpring();
        double sideReinfDamp = spec.sideReinfDamp();
        double sideReinfSpringExp = spec.sideReinfSpringExp();
        double sideReinfDampExp = spec.sideReinfDampExp();
        boolean enableTireLBeams = spec.enableTireLBeams();
        boolean enableTireReinfBeams = spec.enableTireReinfBeams();
        boolean enableTireSideReinfBeams = spec.enableTireSideReinfBeams();
        boolean enableTreadReinfBeams = spec.enableTreadReinfBeams();
        boolean enableTirePeripheryReinfBeams = spec.enableTirePeripheryReinfBeams();
        boolean enableTireSupportBeams = spec.enableTireSupportBeams();
        String group = spec.group();

        if (!wheels.nameToIndex.containsKey(wheelName)) return;

        wheels.ensureWheelCapacity();

        int wIdx = wheels.nameToIndex.get(wheelName);
        int baseOffset = wIdx * WheelContainer.MAX_RAYS;
        int partId = vehicle.nodes.partId[n1];

        wheels.tireRadius[wIdx] = (float) radius;
        wheels.tireWidth[wIdx] = (float) width;
        wheels.pressurePSI[wIdx] = (float) pressurePSI;
        wheels.isDeflated[wIdx] = false;

        wheels.frictionCoef[wIdx] = (float) frictionCoef;
        wheels.slidingFrictionCoef[wIdx] = (float) slidingFrictionCoef;
        wheels.stribeckVelMult[wIdx] = (float) stribeckVelMult;
        wheels.stribeckExponent[wIdx] = (float) stribeckExponent;
        wheels.treadCoef[wIdx] = (float) treadCoef;
        wheels.noLoadCoef[wIdx] = (float) noLoadCoef;
        wheels.loadSensitivitySlope[wIdx] = (float) loadSensitivitySlope;
        wheels.fullLoadCoef[wIdx] = (float) fullLoadCoef;
        wheels.softnessCoef[wIdx] = (float) softnessCoef;
        wheels.brakeTorque[wIdx] = Math.max(0.0f, (float) spec.brakeTorque());
        wheels.parkingTorque[wIdx] = Math.max(0.0f, (float) spec.parkingTorque());
        wheels.brakeSpring[wIdx] = Math.max(0.0f, (float) spec.brakeSpring());
        wheels.brakeInputSplit[wIdx] = Math.clamp((float) spec.brakeInputSplit(), 0.0f, 1.0f);
        wheels.brakeSplitCoef[wIdx] = Math.clamp((float) spec.brakeSplitCoef(), 0.0f, 1.0f);
        wheels.brakePressureInDelay[wIdx] = Math.max(0.0f, (float) spec.brakePressureInDelay());
        wheels.brakePressureOutDelay[wIdx] = Math.max(0.0f, (float) spec.brakePressureOutDelay());
        wheels.serviceBrakeTorque[wIdx] = 0.0f;
        wheels.brakeAngle[wIdx] = 0.0f;

        double[] uX = {0}, uY = {0}, uZ = {0};
        double[] vX = {0}, vY = {0}, vZ = {0};
        double[] axisX = {0}, axisY = {0}, axisZ = {0};
        calculateWheelBasis(n1, n2, wheelDir, axisX, axisY, axisZ, uX, uY, uZ, vX, vY, vZ);

        // 1. 算出 n1 和 n2 的物理中点
        double midX = (vehicle.nodes.posX[n1] + vehicle.nodes.posX[n2]) * 0.5;
        double midY = (vehicle.nodes.posY[n1] + vehicle.nodes.posY[n2]) * 0.5;
        double midZ = (vehicle.nodes.posZ[n1] + vehicle.nodes.posZ[n2]) * 0.5;

        // 2. 基于中点施加 Offset 偏距 (减号保持不变：axisX 指向外侧，减去 offset 即向外)
        double centerX = midX - axisX[0] * offset;
        double centerY = midY - axisY[0] * offset;
        double centerZ = midZ - axisZ[0] * offset;

        // 1. 生成轮胎外圈节点
        //
        // Same half-ray offset as the hub, matching addPressureWheel. Note the
        // parity is opposite to the hub: here the n1-side ring sits on the ray angles
        // and the n2-side ring is half a step ahead, which is what makes the tread
        // and sidewall diagonals fold the way BeamNG's do.
        double tireStep = (2.0 * Math.PI) / rays;
        double tireHalfStep = tireStep * 0.5;
        for (int i = 0; i < rays; i++) {
            double inAngle = i * tireStep + tireHalfStep;
            double outAngle = i * tireStep;
            double inCos = Math.cos(inAngle);
            double inSin = Math.sin(inAngle);
            double outCos = Math.cos(outAngle);
            double outSin = Math.sin(outAngle);

            double inX = centerX + (uX[0] * inCos + vX[0] * inSin) * radius - axisX[0] * (width * 0.5);
            double inY = centerY + (uY[0] * inCos + vY[0] * inSin) * radius - axisY[0] * (width * 0.5);
            double inZ = centerZ + (uZ[0] * inCos + vZ[0] * inSin) * radius - axisZ[0] * (width * 0.5);

            double outX = centerX + (uX[0] * outCos + vX[0] * outSin) * radius + axisX[0] * (width * 0.5);
            double outY = centerY + (uY[0] * outCos + vY[0] * outSin) * radius + axisY[0] * (width * 0.5);
            double outZ = centerZ + (uZ[0] * outCos + vZ[0] * outSin) * radius + axisZ[0] * (width * 0.5);

            int idxIn = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_tire_in_" + i, (float) inX, (float) inY, (float) inZ, (float) nodeWeight,
                    (float) frictionCoef, (float) slidingFrictionCoef, partId,
                    true, false, List.of(group)));
            wheels.tireInnerNodes[baseOffset + i] = idxIn;
            vehicle.nodes.bindToTire(idxIn, wIdx);

            int idxOut = vehicle.nodes.addNode(new PhysicsSpecs.NodeSpec(
                    wheelName + "_tire_out_" + i, (float) outX, (float) outY, (float) outZ, (float) nodeWeight,
                    (float) frictionCoef, (float) slidingFrictionCoef, partId,
                    true, false, List.of(group)));
            wheels.tireOuterNodes[baseOffset + i] = idxOut;
            vehicle.nodes.bindToTire(idxOut, wIdx);
        }

        boolean COLLISION = false;

        // 2. 缝合轮胎三角形与梁
        wheels.tireTriangleIdxStart[wIdx] = vehicle.triangles.count;
        for (int i = 0; i < rays; i++) {
            int next = (i + 1) % rays;

            int hInCur = wheels.hubInnerNodes[baseOffset + i], hInNext = wheels.hubInnerNodes[baseOffset + next];
            int hOutCur = wheels.hubOuterNodes[baseOffset + i], hOutNext = wheels.hubOuterNodes[baseOffset + next];
            int tInCur = wheels.tireInnerNodes[baseOffset + i], tInNext = wheels.tireInnerNodes[baseOffset + next];
            int tOutCur = wheels.tireOuterNodes[baseOffset + i], tOutNext = wheels.tireOuterNodes[baseOffset + next];

            // 侧壁面：内侧环带 (Hub Inner -> Tire Inner)
            // Keep BeamNG's tIn_i -> hIn_{i+1} diagonal. After the two rings were
            // staggered by half a ray, the old opposite diagonal no longer had a
            // matching sidewall beam and let pressure shear the tread cyclically.
            addTriangle(tInCur, hInCur, hInNext, partId, COLLISION);
            addTriangle(tInCur, hInNext, tInNext, partId, COLLISION);

            // 侧壁面：外侧环带 (Hub Outer -> Tire Outer)
            addTriangle(hOutCur, tOutCur, tOutNext, partId, COLLISION);
            addTriangle(hOutCur, tOutNext, hOutNext, partId, COLLISION);

            // 胎面 (Tire Inner -> Tire Outer)
            addTriangle(tInCur, tInNext, tOutNext, partId, COLLISION);
            addTriangle(tInCur, tOutNext, tOutCur, partId, COLLISION);

            // 轮胎与轮辋接触面（仅用于闭合散度体积，关闭碰撞）
            // Use the same hOut_i -> hIn_{i+1} diagonal as hubTread. The former
            // triangulation was left over from the aligned-ring topology.
            addTriangle(hOutCur, hInNext, hInCur, partId, false);
            addTriangle(hOutCur, hOutNext, hInNext, partId, false);

            // 胎面 加强筋 (i 连 i+2)
            int next2 = (i + 2) % rays;
            int tInNext2 = wheels.tireInnerNodes[baseOffset + next2];
            int tOutNext2 = wheels.tireOuterNodes[baseOffset + next2];
            // ========================================================
            // 1. 圆周梁 (Periphery Beams) —— 维持周长，主导纵向抓地力
            // ========================================================
            // 普通圆周梁 (沿 i 连 i+1)
            int periInIdx = addFastBeam(tInCur,  tInNext,  periSpring, periDamp, periDeform, periStrength);
            int periOutIdx = addFastBeam(tOutCur, tOutNext, periSpring, periDamp, periDeform, periStrength);
            vehicle.normalBeams.bindToTire(periInIdx, wIdx);
            vehicle.normalBeams.bindToTire(periOutIdx, wIdx);

            // 圆周加强筋 (沿 i 连 i+2，文档中的 circumference +-2 nodes)
            if (enableTirePeripheryReinfBeams) {
                addFastBeam(tInCur,  tInNext2,  periReinfSpring, periReinfDamp, periDeform, periStrength);
                addFastBeam(tOutCur, tOutNext2, periReinfSpring, periReinfDamp, periDeform, periStrength);
            }

            // ========================================================
            // 2. 胎面横向梁 (Tread Beams) —— 跨宽度，主导过弯侧向支撑
            // ========================================================
            // BeamNG keeps exactly two per ray: the within-ray cross-width beam and one
            // half-step-folded diagonal. The previous third beam (tOut_c->tIn_next) is
            // BeamNG's tread-reinforcement beam, so it moved into that family below.
            addFastBeam(tOutCur, tInCur,  treadSpring, treadDamp, treadDeform, treadStrength);
            addFastBeam(tInCur,  tOutNext, treadSpring, treadDamp, treadDeform, treadStrength);

            // 胎面加强筋 (跨宽度 且 跨圆周的大交叉，文档中的 across +-2 nodes)
            if (enableTreadReinfBeams) {
                addFastBeam(tOutCur, tInNext,  treadReinfSpring, treadReinfDamp, treadDeform, treadStrength);
                addFastBeam(tInCur,  tOutNext2, treadReinfSpring, treadReinfDamp, treadDeform, treadStrength);
            }

            // ========================================================
            // 3. 侧壁梁 (Sidewall Beams) —— 连 Hub 和 Tire，由气压主导
            // ========================================================
            // BeamNG braces each sidewall band with a V per ray: hub ring i to tire ring
            // i, and tire ring i to hub ring i+1. Two straight radial beams per side (the
            // previous layout) is half the bracing.
            int sideOutA = addFastAnisotropicBeam(hOutCur, tOutCur, sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);
            int sideOutB = addFastAnisotropicBeam(hOutCur, tOutNext, sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);
            vehicle.anisotropicBeams.bindToTire(sideOutA, wIdx);
            vehicle.anisotropicBeams.bindToTire(sideOutB, wIdx);
            addFastAnisotropicBeam(hInCur, tInCur, sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);
            addFastAnisotropicBeam(tInCur, hInNext, sideSpring, sideDamp, sideDeform, sideStrength,
                    sideSpringExp, sideDampExp, sideTransZone);

            // 侧壁加强筋 (同侧长斜撑，跨 i+1 与 i+2)
            if (enableTireSideReinfBeams) {
                addFastAnisotropicBeam(hInCur,  tInNext,  sideReinfSpring, sideReinfDamp, sideDeform, sideStrength,
                        sideReinfSpringExp, sideReinfDampExp, sideTransZone);
                addFastAnisotropicBeam(tInCur,  wheels.hubInnerNodes[baseOffset + next2], sideReinfSpring, sideReinfDamp,
                        sideDeform, sideStrength, sideReinfSpringExp, sideReinfDampExp, sideTransZone);
                addFastAnisotropicBeam(tOutCur, hOutNext, sideReinfSpring, sideReinfDamp, sideDeform, sideStrength,
                        sideReinfSpringExp, sideReinfDampExp, sideTransZone);
                addFastAnisotropicBeam(hOutCur, wheels.tireOuterNodes[baseOffset + next2], sideReinfSpring, sideReinfDamp,
                        sideDeform, sideStrength, sideReinfSpringExp, sideReinfDampExp, sideTransZone);
            }

            // ========================================================
            // 4. 内部截面大支撑 (wheelReinfBeam / L-Beam)
            // ========================================================
            // 穿过空气腔，连接内侧 Hub 和 外侧 Tire，防止轮胎截面横向塌陷。
            // BeamNG picks exactly one of the two forms; L-beams are the default and the
            // straight beams are only used when reinforcement beams are explicitly on.
            if (enableTireReinfBeams) {
                addFastBeam(tOutCur, hInCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
                addFastBeam(hOutCur, tInCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
            } else if (enableTireLBeams) {
                // 交叉对角线 1：连接 tOut 和 hIn，以对侧轮辋节点 hOut 为支点
                addFastLBeam(tOutCur, hInCur, hOutCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);

                // 交叉对角线 2：连接 hOut 和 tIn，以对侧轮辋节点 hIn 为支点
                addFastLBeam(hOutCur, tInCur, hInCur, reinfSpring, reinfDamp, reinfDeform, reinfStrength);
            }

            // ========================================================
            // 5. 防瘪兜底梁 (Tire Support Beams) —— 仅做物理限位缓冲
            // ========================================================
            if (enableTireSupportBeams) {
                // TODO: 优先级不高
                // 这里的梁应当存入 supportBeams 容器，并设置 beamPrecompression（如 0.85），
                // 使其平时处于松弛状态，只在轮胎接近压死时提供较大的推力
                // vehicle.supportBeams.addBeam(...);
            }
        }
        wheels.tireTriangleIdxEnd[wIdx] = vehicle.triangles.count - 1;

        // 用离散网格积分求体积，不用圆柱公式，这样初始内外压强比恰为 1.0
        double volSum = 0.0;
        for (int i = wheels.tireTriangleIdxStart[wIdx]; i <= wheels.tireTriangleIdxEnd[wIdx]; i++) {
            int nA = vehicle.triangles.node1[i];
            int nB = vehicle.triangles.node2[i];
            int nC = vehicle.triangles.node3[i];

            double ax = vehicle.nodes.posX[nA], ay = vehicle.nodes.posY[nA], az = vehicle.nodes.posZ[nA];
            double bx = vehicle.nodes.posX[nB], by = vehicle.nodes.posY[nB], bz = vehicle.nodes.posZ[nB];
            double cx = vehicle.nodes.posX[nC], cy = vehicle.nodes.posY[nC], cz = vehicle.nodes.posZ[nC];

            double crossX = by * cz - bz * cy;
            double crossY = bz * cx - bx * cz;
            double crossZ = bx * cy - by * cx;

            volSum += (ax * crossX + ay * crossY + az * crossZ);
        }
        // 记录静止体积
        wheels.initialVolume[wIdx] = (float) Math.abs(volSum / 6.0);

        // 初始化 prevVolume 与 normalSign
        wheels.prevVolume[wIdx] = wheels.initialVolume[wIdx];
        wheels.normalSign[wIdx] = (volSum < 0.0) ? -1.0f : 1.0f;
    }

    private int addFastBeam(int id1, int id2, double spring, double damp, double deform, double strength) {
        double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return vehicle.normalBeams.addBeam(
                beamSpec(BeamContainer.BEAM_NORMAL, spring, damp, deform, strength),
                id1, id2, (float) dist
        );
    }

    private int addFastLBeam(int id1, int id2, int id3, double spring, double damp, double deform, double strength) {
        double dx;
        double dy;
        double dz;
        dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist12 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        dx = vehicle.nodes.posX[id3] - vehicle.nodes.posX[id1];
        dy = vehicle.nodes.posY[id3] - vehicle.nodes.posY[id1];
        dz = vehicle.nodes.posZ[id3] - vehicle.nodes.posZ[id1];
        double dist13 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        dx = vehicle.nodes.posX[id3] - vehicle.nodes.posX[id2];
        dy = vehicle.nodes.posY[id3] - vehicle.nodes.posY[id2];
        dz = vehicle.nodes.posZ[id3] - vehicle.nodes.posZ[id2];
        double dist23 = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return vehicle.lBeams.addBeam(
                beamSpec(BeamContainer.BEAM_LBEAM, spring, damp, deform, strength),
                id1, id2, id3, (float) dist12, (float) dist13, (float) dist23
        );
    }

    private int addFastAnisotropicBeam(int id1, int id2, double spring, double damp, double deform, double strength, double springExp, double dampExp, double transitionZone) {
        double dx = vehicle.nodes.posX[id2] - vehicle.nodes.posX[id1];
        double dy = vehicle.nodes.posY[id2] - vehicle.nodes.posY[id1];
        double dz = vehicle.nodes.posZ[id2] - vehicle.nodes.posZ[id1];
        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        return vehicle.anisotropicBeams.addBeam(
                beamSpec(BeamContainer.BEAM_ANISOTROPIC, spring, damp, deform, strength,
                        springExp, dampExp, transitionZone),
                id1, id2, (float) dist
        );
    }

    private PhysicsSpecs.BeamSpec beamSpec(int type, double spring, double damp, double deform, double strength) {
        return beamSpec(type, spring, damp, deform, strength, spring, damp, 0.0);
    }

    private PhysicsSpecs.BeamSpec beamSpec(int type, double spring, double damp, double deform, double strength,
                                           double springExpansion, double dampExpansion, double transitionZone) {
        return new PhysicsSpecs.BeamSpec(
                type, null, null, null,
                List.of(), Float.POSITIVE_INFINITY,
                null, 0, false,
                (float) spring, (float) damp, -1.0f, (float) deform, (float) strength,
                1.0f, 0.0f, false, 0.0f,
                1.0f, 1.0f, -1.0f, -1.0f,
                1.0f,
                (float) spring, (float) damp, -1.0f,
                -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
                (float) springExpansion, (float) dampExpansion, (float) transitionZone,
                PhysicsWorld.KINDA_BIG_NUMBER,
                null
        );
    }

    private void addTriangle(int n1, int n2, int n3, int partId, boolean collision) {
        vehicle.triangles.addTriangle(
                new PhysicsSpecs.TriangleSpec(null, null, null, List.of(), partId, collision),
                n1, n2, n3);
    }

    private void calculateWheelBasis(int n1, int n2, int wheelDir, double[] ax, double[] ay, double[] az, double[] ux, double[] uy, double[] uz, double[] vx, double[] vy, double[] vz) {
        double n1x = vehicle.nodes.posX[n1], n1y = vehicle.nodes.posY[n1], n1z = vehicle.nodes.posZ[n1];
        double n2x = vehicle.nodes.posX[n2], n2y = vehicle.nodes.posY[n2], n2z = vehicle.nodes.posZ[n2];

        // n1 是外侧，n2 是内侧，因此 n1 - n2 指向车外
        ax[0] = n1x - n2x; ay[0] = n1y - n2y; az[0] = n1z - n2z;
        double len = Math.sqrt(ax[0]*ax[0] + ay[0]*ay[0] + az[0]*az[0]);
        if (len > 0) { ax[0]/=len; ay[0]/=len; az[0]/=len; }

        if (Math.abs(ay[0]) > 0.9) { ux[0] = 1; uy[0] = 0; uz[0] = 0; }
        else { ux[0] = 0; uy[0] = 1; uz[0] = 0; }

        vx[0] = ay[0] * uz[0] - az[0] * uy[0];
        vy[0] = az[0] * ux[0] - ax[0] * uz[0];
        vz[0] = ax[0] * uy[0] - ay[0] * ux[0];
        double vLen = Math.sqrt(vx[0]*vx[0] + vy[0]*vy[0] + vz[0]*vz[0]);
        if (vLen > 0) { vx[0]/=vLen; vy[0]/=vLen; vz[0]/=vLen; }

        ux[0] = vy[0] * az[0] - vz[0] * ay[0];
        uy[0] = vz[0] * ax[0] - vx[0] * az[0];
        uz[0] = vx[0] * ay[0] - vy[0] * ax[0];
    }


}
