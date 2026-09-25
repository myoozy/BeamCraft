/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, You can obtain one at
 * https://beamng.com/bCDDL-1.1.txt
 */
package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.debug.LoadTiming;
import me.mzy.beamcraft.client.physics.BeamGraph;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.PropContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.WheelContainer;

import java.util.ArrayList;
import java.util.List;

public class FlexbodyBindingUtil {

    /** Number of locally-nearest nodes considered when building a vertex basis. */
    static final int MAX_BASIS_CANDIDATES = 16;
    /** BeamNG describes VX/VY as roughly perpendicular; this is the preferred lower bound. */
    static final double PREFERRED_BASIS_ANGLE_DEGREES = 45.0;
    /** Below this angle the planar solve is too ill-conditioned to use safely. */
    static final double MIN_USABLE_BASIS_ANGLE_DEGREES = 20.0;
    /** Bounds used by BeamNG's Flexbody Debug to flag potentially spiking locator coordinates. */
    static final double PREFERRED_LOCATOR_MIN = -0.5;
    static final double PREFERRED_LOCATOR_MAX = 1.5;
    /**
     * Maximum L1 norm of the affine node coefficients used by an explicit
     * four-node deform basis.
     * A value of 1 is interpolation inside the chosen simplex; 2 permits
     * moderate extrapolation without letting ordinary node separation turn
     * into a much larger render-mesh spike.
     */
    static final double MAX_AFFINE_NODE_GAIN = 2.0;
    /** Four-node binding must beat the three-node normal basis by a useful margin. */
    static final double EXPLICIT_Z_SENSITIVITY_MARGIN = 0.05;
    /**
     * Meshes below this confident-VZ fraction use one continuous three-node model.
     * This separates ETK's volumetric wheel/brake meshes from its mostly planar
     * bumper/duct meshes without inspecting mesh or part names.
     */
    static final double MIN_EXPLICIT_Z_MESH_COVERAGE = 0.75;
    /** A VZ candidate must reach every planar anchor within this local beam radius. */
    static final int MAX_COHESIVE_CAGE_HOPS = 2;
    /** Last-resort finite guard. Values this large are never a defensible local locator. */
    static final double MAX_SAFE_LOCATOR_MAGNITUDE = 15.0;
    static final double MIN_AXIS_LENGTH_SQUARED = 1.0e-6;
    static final double MIN_BASIS_NORMAL_LENGTH_SQUARED = 1.0e-14;
    static final double MIN_INPUT_NORMAL_LENGTH = 1.0e-5;
    static final double MIN_INVERSE_SCALE_MAGNITUDE = 1.0e-12;

    private enum ExplicitZMode {
        /** Conservative per-vertex vote used only to classify the whole mesh. */
        CONFIDENCE_PROBE,
        /** Topology and affine safety decide VZ after the mesh has passed the vote. */
        STRUCTURAL,
        /** Force the continuous three-node representation. */
        DISABLED
    }

    public static void performBinding(FlexbodyContainer flex, SoftBodyVehicle vehicle) {
        NodeContainer nodes = vehicle.nodes;
        if (flex.isSkinningBound || flex.meshCount == 0) return;

        long totalStart = LoadTiming.start();
        int totalVerts = 0;

        long scanStart = LoadTiming.start();
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

            // An unresolvable mesh gets its name cleared. Its vertices are not counted
            // here, and the renderer later skips it because the name resolves to nothing.
            if (!valid) {
                flex.meshName[m] = "";
            } else {
                DaeMeshLoader.RawGeometry geom = DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[m]);
                if (geom != null) totalVerts += geom.vertexCount;
            }
        }

        LoadTiming.log("[flex] mesh scan (" + flex.meshCount + " meshes)", scanStart);

        long allocStart = LoadTiming.start();
        flex.allocateSkinningBuffers(totalVerts);
        LoadTiming.log("[flex] buffer allocation (" + totalVerts + " verts)", allocStart);
        if (totalVerts == 0) {
            flex.isSkinningBound = true;
            return;
        }

        int ptr = 0;
        BeamGraph beamGraph = vehicle.beamGraph();
        boolean[] wheelAxisNodes = collectWheelAxisNodes(vehicle.wheels, nodes.count);
        boolean[] generatedWheelNodes = collectGeneratedWheelNodes(vehicle.wheels, nodes.count);

        long bindStart = LoadTiming.start();
        for (int m = 0; m < flex.meshCount; m++) {
            // Skip meshes cleared above.
            if (flex.meshName[m].isEmpty()) continue;

            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.resolveMesh(flex.vehicleNamespace, flex.meshName[m]);
            if (geom == null) continue;

            int propIndex = flex.propIndex[m];
            if (propIndex >= 0) {
                ptr = bindPropMesh(flex, vehicle, nodes, m, propIndex, geom, ptr);
                continue;
            }

            List<Integer> primaryPool = new ArrayList<>();
            List<Integer> structuralPool = new ArrayList<>();
            boolean[] addedToPool = new boolean[nodes.count];
            boolean[] addedToStructuralPool = new boolean[nodes.count];
            boolean hasAxisOnlyGroup = false;
            boolean hasStructuralGroup = false;
            boolean hasGeneratedWheelGroup = false;
            if (flex.targetGroups[m] != null && !flex.targetGroups[m].isEmpty()) {
                for (String gName : flex.targetGroups[m]) {
                    Integer gId = flex.groupNameToId.get(gName);
                    if (gId != null) {
                        int start = flex.groupNodeOffsets[gId];
                        int count = flex.groupNodeCounts[gId];
                        boolean axisOnly = count > 0;
                        boolean containsGeneratedWheelNode = false;
                        for (int i = 0; i < count; i++) {
                            int node = flex.flatGroupNodes[start + i];
                            axisOnly &= wheelAxisNodes[node];
                            containsGeneratedWheelNode |= generatedWheelNodes[node];
                        }
                        hasAxisOnlyGroup |= axisOnly;
                        hasGeneratedWheelGroup |= containsGeneratedWheelNode;
                        boolean structural = !axisOnly && !containsGeneratedWheelNode;
                        hasStructuralGroup |= structural;
                        for (int i = 0; i < count; i++) {
                            int node = flex.flatGroupNodes[start + i];
                            if (!addedToPool[node]) {
                                addedToPool[node] = true;
                                primaryPool.add(node);
                            }
                            if (structural && !addedToStructuralPool[node]) {
                                addedToStructuralPool[node] = true;
                                structuralPool.add(node);
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < nodes.count; i++) primaryPool.add(i);
            }

            // A non-rotating hub-side flexbody (for example a brake caliper) may
            // intentionally list both its suspension structure and the two wheel-axis
            // nodes. Those axis nodes are useful while attached, but become a destructive
            // locator when the wheel breaks away. Generated wheel/tire groups identify
            // genuinely rotating meshes, which must retain the full authored pool.
            List<Integer> bindingPool = hasAxisOnlyGroup && hasStructuralGroup && !hasGeneratedWheelGroup
                    ? structuralPool : primaryPool;

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
            int meshVertexStart = ptr;
            int explicitZVertices = 0;

            for (int v = 0; v < geom.vertexCount; v++) {

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
                    // A normal is transformed by the inverse transpose of the
                    // position scale, not by the position scale itself.
                    double rawNx = inverseScaleNormal(norms[v * 3], sX);
                    double rawNy = inverseScaleNormal(norms[v * 3 + 1], sY);
                    double rawNz = inverseScaleNormal(norms[v * 3 + 2], sZ);
                    double nx1 = rawNx * cosZ - rawNy * sinZ, ny1 = rawNx * sinZ + rawNy * cosZ, nz1 = rawNz;
                    double nx2 = nx1, ny2 = ny1 * cosX - nz1 * sinX, nz2 = ny1 * sinX + nz1 * cosX;
                    double nx3 = nx2 * cosY + nz2 * sinY, ny3 = ny2, nz3 = -nx2 * sinY + nz2 * cosY;

                    double[] gNorm = slotCtx != null ? slotCtx.transformNode(nx3, ny3, nz3) : new double[]{nx3, ny3, nz3};
                    double[] gOrigin = slotCtx != null ? slotCtx.transformNode(0, 0, 0) : new double[]{0, 0, 0};

                    nOrigX = +(gNorm[0] - gOrigin[0]);
                    nOrigY = +(gNorm[2] - gOrigin[2]);
                    nOrigZ = -(gNorm[1] - gOrigin[1]);
                }

                // There is no global fallback pool. If no usable basis exists inside the
                // mesh's own groups, fall back to a rigid binding at the vertex's own
                // position rather than reaching into nodes owned by other parts.
                boolean success = calculateDecoupledWeights(flex, nodes, ptr,
                        staticMcX, staticMcY, staticMcZ, nOrigX, nOrigY, nOrigZ,
                        beamGraph, ExplicitZMode.CONFIDENCE_PROBE, bindingPool);
                if (!success) {
                    applyFallbackRigidBinding(flex, nodes, ptr, staticMcX, staticMcY, staticMcZ, nOrigX, nOrigY, nOrigZ, bindingPool);
                }
                if (!flex.vUseCrossZ[ptr] && flex.vVzNode[ptr] >= 0) explicitZVertices++;

                if (uvs != null && v * 2 + 1 < uvs.length) {
                    flex.uvU[ptr] = uvs[v * 2]; flex.uvV[ptr] = uvs[v * 2 + 1];
                } else {
                    flex.uvU[ptr] = 0.0f; flex.uvV[ptr] = 0.0f;
                }
                ptr++;
            }

            boolean useStructuralExplicitZ = retainExplicitZForMesh(explicitZVertices, geom.vertexCount);
            for (int vertex = meshVertexStart; vertex < ptr; vertex++) {
                boolean deformBasis = flex.vUseCrossZ[vertex] || flex.vVzNode[vertex] >= 0;
                if (!deformBasis) continue;
                if (!useStructuralExplicitZ && flex.vVzNode[vertex] < 0) continue;
                double[] restNormal = reconstructRestNormal(flex, nodes, vertex);
                boolean rebound = calculateDecoupledWeights(flex, nodes, vertex,
                        flex.skinnedPosX[vertex], flex.skinnedPosY[vertex], flex.skinnedPosZ[vertex],
                        restNormal[0], restNormal[1], restNormal[2],
                        beamGraph,
                        useStructuralExplicitZ ? ExplicitZMode.STRUCTURAL : ExplicitZMode.DISABLED,
                        bindingPool);
                if (!rebound) {
                    applyFallbackRigidBinding(flex, nodes, vertex,
                            flex.skinnedPosX[vertex], flex.skinnedPosY[vertex], flex.skinnedPosZ[vertex],
                            restNormal[0], restNormal[1], restNormal[2], bindingPool);
                }
            }
        }
        LoadTiming.log("[flex] per-vertex binding (" + ptr + " verts)", bindStart);

        flex.isSkinningBound = true;
        System.out.println("Flexbody binding complete, total render vertices: " + flex.totalVertexCount);
        LoadTiming.log("[flex] binding total", totalStart);
    }

    /** Bind one non-deforming prop to three render-only frame nodes. */
    private static int bindPropMesh(FlexbodyContainer flex, SoftBodyVehicle vehicle, NodeContainer nodes,
                                    int mesh, int prop, DaeMeshLoader.RawGeometry geom, int ptr) {
        PropContainer props = vehicle.props;
        float[] basis = new float[9];
        if (!PropContainer.buildReferenceBasis(
                nodes.baseX, nodes.baseY, nodes.baseZ,
                props.refNode[prop], props.xNode[prop], props.yNode[prop], basis)) {
            flex.meshName[mesh] = "";
            return ptr;
        }

        JBeamAssembler.TransformContext slotContext = flex.slotContext[mesh];
        double[] daePivot = transformDaePoint(geom.originX, geom.originY, geom.originZ, slotContext);
        float pivotX = (float) daePivot[0];
        float pivotY = (float) daePivot[2];
        float pivotZ = (float) -daePivot[1];

        int ref = props.refNode[prop];
        float refX = nodes.baseX[ref], refY = nodes.baseY[ref], refZ = nodes.baseZ[ref];
        float bx0 = basis[0], bx1 = basis[3], bx2 = basis[6];
        float by0 = basis[1], by1 = basis[4], by2 = basis[7];
        float bz0 = basis[2], bz1 = basis[5], bz2 = basis[8];

        // Assimp gives us vertices with the complete DAE scene transform baked in.
        // BeamNG props, however, animate the named object's local mesh and let the
        // prop reference frame perform the placement. Remove the baked DAE/root
        // orientation here; retaining it would rotate every prop twice.
        double[] objectAxisX = transformDaeDirection(geom.axisXX, geom.axisXY, geom.axisXZ, slotContext);
        double[] objectAxisY = transformDaeDirection(geom.axisYX, geom.axisYY, geom.axisYZ, slotContext);
        float[] objectBasis = {
                (float) objectAxisX[0], (float) objectAxisY[0], 0.0f,
                (float) objectAxisX[2], (float) objectAxisY[2], 0.0f,
                (float) -objectAxisX[1], (float) -objectAxisY[1], 0.0f
        };
        if (!orthonormalizeBasisColumns(objectBasis)) {
            flex.meshName[mesh] = "";
            return ptr;
        }
        float mx0 = objectBasis[0], mx1 = objectBasis[3], mx2 = objectBasis[6];
        float my0 = objectBasis[1], my1 = objectBasis[4], my2 = objectBasis[7];
        float mz0 = objectBasis[2], mz1 = objectBasis[5], mz2 = objectBasis[8];

        float[] orientedBasis = new float[9];
        boolean orientationReady = props.hasBaseRotationGlobal[prop]
                ? PropContainer.buildGlobalPropOrientation(
                nodes.baseX, nodes.baseY, nodes.baseZ, vehicle.cameras.refNodes(),
                props.baseRotationGlobalX[prop], props.baseRotationGlobalY[prop], props.baseRotationGlobalZ[prop],
                0.0f, 0.0f, 0.0f, orientedBasis, new float[9])
                : PropContainer.buildPropOrientation(
                nodes.baseX, nodes.baseY, nodes.baseZ,
                props.refNode[prop], props.xNode[prop], props.yNode[prop],
                props.baseRotationX[prop], props.baseRotationY[prop], props.baseRotationZ[prop],
                0.0f, 0.0f, 0.0f, orientedBasis);
        if (!orientationReady) {
            flex.meshName[mesh] = "";
            return ptr;
        }

        float targetOriginX = pivotX, targetOriginY = pivotY, targetOriginZ = pivotZ;
        if (props.hasBaseTranslationGlobal[prop]) {
            double[] translated = transformDaePoint(
                    props.baseTranslationGlobalX[prop],
                    props.baseTranslationGlobalY[prop],
                    props.baseTranslationGlobalZ[prop],
                    slotContext);
            targetOriginX = (float) translated[0];
            targetOriginY = (float) translated[2];
            targetOriginZ = (float) -translated[1];
        } else if (props.hasBaseTranslation[prop]) {
            targetOriginX = refX
                    + bx0 * props.baseTranslationX[prop]
                    + by0 * props.baseTranslationY[prop]
                    + bz0 * props.baseTranslationZ[prop];
            targetOriginY = refY
                    + bx1 * props.baseTranslationX[prop]
                    + by1 * props.baseTranslationY[prop]
                    + bz1 * props.baseTranslationZ[prop];
            targetOriginZ = refZ
                    + bx2 * props.baseTranslationX[prop]
                    + by2 * props.baseTranslationY[prop]
                    + bz2 * props.baseTranslationZ[prop];
        }

        float originDx = targetOriginX - refX;
        float originDy = targetOriginY - refY;
        float originDz = targetOriginZ - refZ;
        props.originLocalX[prop] = dot(originDx, originDy, originDz, bx0, bx1, bx2);
        props.originLocalY[prop] = dot(originDx, originDy, originDz, by0, by1, by2);
        props.originLocalZ[prop] = dot(originDx, originDy, originDz, bz0, bz1, bz2);
        props.originBound[prop] = true;

        int syntheticBase = props.syntheticNodeBase(nodes.count, prop);

        for (int vertex = 0; vertex < geom.vertexCount; vertex++, ptr++) {
            double[] transformed = transformDaePoint(
                    geom.positions[vertex * 3], geom.positions[vertex * 3 + 1], geom.positions[vertex * 3 + 2],
                    slotContext);
            float vx = (float) transformed[0];
            float vy = (float) transformed[2];
            float vz = (float) -transformed[1];
            float dx = vx - pivotX, dy = vy - pivotY, dz = vz - pivotZ;
            float localX = dot(dx, dy, dz, mx0, mx1, mx2);
            float localY = dot(dx, dy, dz, my0, my1, my2);
            float localZ = dot(dx, dy, dz, mz0, mz1, mz2);

            flex.vCenterNode[ptr] = syntheticBase;
            flex.vVxNode[ptr] = syntheticBase + 1;
            flex.vVyNode[ptr] = syntheticBase + 2;
            flex.vVzNode[ptr] = -1;
            flex.vWeightX[ptr] = localX;
            flex.vWeightY[ptr] = localY;
            flex.vWeightZ[ptr] = localZ;
            flex.vUseCrossZ[ptr] = true;
            flex.vRestCrossLength[ptr] = 1.0f;

            float normalX = 0.0f, normalY = 0.0f, normalZ = 1.0f;
            if (geom.normals != null && vertex * 3 + 2 < geom.normals.length) {
                double[] transformedNormal = transformDaeDirection(
                        geom.normals[vertex * 3], geom.normals[vertex * 3 + 1], geom.normals[vertex * 3 + 2],
                        slotContext);
                normalX = (float) transformedNormal[0];
                normalY = (float) transformedNormal[2];
                normalZ = (float) -transformedNormal[1];
            }
            flex.vNormWeightX[ptr] = dot(normalX, normalY, normalZ, mx0, mx1, mx2);
            flex.vNormWeightY[ptr] = dot(normalX, normalY, normalZ, my0, my1, my2);
            flex.vNormWeightZ[ptr] = dot(normalX, normalY, normalZ, mz0, mz1, mz2);

            flex.skinnedPosX[ptr] = targetOriginX
                    + orientedBasis[0] * localX + orientedBasis[1] * localY + orientedBasis[2] * localZ;
            flex.skinnedPosY[ptr] = targetOriginY
                    + orientedBasis[3] * localX + orientedBasis[4] * localY + orientedBasis[5] * localZ;
            flex.skinnedPosZ[ptr] = targetOriginZ
                    + orientedBasis[6] * localX + orientedBasis[7] * localY + orientedBasis[8] * localZ;

            if (geom.uvs != null && vertex * 2 + 1 < geom.uvs.length) {
                flex.uvU[ptr] = geom.uvs[vertex * 2];
                flex.uvV[ptr] = geom.uvs[vertex * 2 + 1];
            }
        }
        return ptr;
    }

    private static double[] transformDaePoint(double x, double y, double z,
                                              JBeamAssembler.TransformContext context) {
        return context == null ? new double[]{x, y, z} : context.transformNode(x, y, z);
    }

    private static double[] transformDaeDirection(double x, double y, double z,
                                                  JBeamAssembler.TransformContext context) {
        if (context == null) return new double[]{x, y, z};
        double[] direction = context.transformNode(x, y, z);
        double[] origin = context.transformNode(0.0, 0.0, 0.0);
        return new double[]{
                direction[0] - origin[0],
                direction[1] - origin[1],
                direction[2] - origin[2]
        };
    }

    private static float dot(float ax, float ay, float az, float bx, float by, float bz) {
        return ax * bx + ay * by + az * bz;
    }

    private static boolean orthonormalizeBasisColumns(float[] basis) {
        float xx = basis[0], xy = basis[3], xz = basis[6];
        float xLengthSquared = xx * xx + xy * xy + xz * xz;
        if (xLengthSquared <= 1.0e-10f) return false;
        float inverseXLength = 1.0f / (float) Math.sqrt(xLengthSquared);
        xx *= inverseXLength; xy *= inverseXLength; xz *= inverseXLength;

        float yx = basis[1], yy = basis[4], yz = basis[7];
        float projection = xx * yx + xy * yy + xz * yz;
        yx -= projection * xx; yy -= projection * xy; yz -= projection * xz;
        float yLengthSquared = yx * yx + yy * yy + yz * yz;
        if (yLengthSquared <= 1.0e-10f) return false;
        float inverseYLength = 1.0f / (float) Math.sqrt(yLengthSquared);
        yx *= inverseYLength; yy *= inverseYLength; yz *= inverseYLength;

        basis[0] = xx; basis[3] = xy; basis[6] = xz;
        basis[1] = yx; basis[4] = yy; basis[7] = yz;
        basis[2] = xy * yz - xz * yy;
        basis[5] = xz * yx - xx * yz;
        basis[8] = xx * yy - xy * yx;
        return true;
    }

    private static boolean[] collectWheelAxisNodes(WheelContainer wheels, int nodeCount) {
        boolean[] result = new boolean[nodeCount];
        for (int wheel = 0; wheel < wheels.count; wheel++) {
            markNode(result, wheels.node1[wheel]);
            markNode(result, wheels.node2[wheel]);
        }
        return result;
    }

    private static boolean[] collectGeneratedWheelNodes(WheelContainer wheels, int nodeCount) {
        boolean[] result = new boolean[nodeCount];
        for (int wheel = 0; wheel < wheels.count; wheel++) {
            int start = wheel * WheelContainer.MAX_RAYS;
            for (int ray = 0; ray < wheels.numRays[wheel]; ray++) {
                int index = start + ray;
                markNode(result, wheels.hubInnerNodes[index]);
                markNode(result, wheels.hubOuterNodes[index]);
                markNode(result, wheels.tireInnerNodes[index]);
                markNode(result, wheels.tireOuterNodes[index]);
            }
        }
        return result;
    }

    private static void markNode(boolean[] nodes, int node) {
        if (node >= 0 && node < nodes.length) nodes[node] = true;
    }

    private static double inverseScaleNormal(double component, double scale) {
        return Math.abs(scale) > MIN_INVERSE_SCALE_MAGNITUDE ? component / scale : 0.0;
    }

    static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                              double vx, double vy, double vz,
                                              double normX, double normY, double normZ, List<Integer> pool) {
        return calculateDecoupledWeights(flex, nodes, ptr, vx, vy, vz,
                normX, normY, normZ, null, ExplicitZMode.STRUCTURAL, pool);
    }

    static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                              double vx, double vy, double vz,
                                              double normX, double normY, double normZ,
                                              BeamGraph beamGraph, List<Integer> pool) {
        return calculateDecoupledWeights(flex, nodes, ptr, vx, vy, vz,
                normX, normY, normZ, beamGraph, ExplicitZMode.CONFIDENCE_PROBE, pool);
    }

    private static boolean calculateDecoupledWeights(FlexbodyContainer flex, NodeContainer nodes, int ptr,
                                              double vx, double vy, double vz,
                                              double normX, double normY, double normZ,
                                              BeamGraph beamGraph, ExplicitZMode explicitZMode, List<Integer> pool) {
        if (pool.size() < 3) return false;

        int centerNode = pool.getFirst();
        double nearestDistanceSquared = Double.POSITIVE_INFINITY;
        for (int node : pool) {
            double dx = vx - nodes.baseX[node], dy = vy - nodes.baseY[node], dz = vz - nodes.baseZ[node];
            double distanceSquared = dx * dx + dy * dy + dz * dz;
            if (distanceSquared < nearestDistanceSquared) {
                nearestDistanceSquared = distanceSquared;
                centerNode = node;
            }
        }

        int capacity = Math.min(MAX_BASIS_CANDIDATES, pool.size() - 1);
        int[] candidates = new int[capacity];
        double[] distances = new double[capacity];
        java.util.Arrays.fill(distances, Double.POSITIVE_INFINITY);
        int candidateCount = 0;
        for (int node : pool) {
            if (node == centerNode) continue;
            double dx = vx - nodes.baseX[node], dy = vy - nodes.baseY[node], dz = vz - nodes.baseZ[node];
            double distanceSquared = dx * dx + dy * dy + dz * dz;
            if (candidateCount == capacity && distanceSquared >= distances[capacity - 1]) continue;
            int insert = Math.min(candidateCount, capacity - 1);
            while (insert > 0 && distanceSquared < distances[insert - 1]) insert--;
            if (insert >= capacity) continue;
            int moveCount = Math.min(candidateCount, capacity - 1) - insert;
            if (moveCount > 0) {
                System.arraycopy(candidates, insert, candidates, insert + 1, moveCount);
                System.arraycopy(distances, insert, distances, insert + 1, moveCount);
            }
            candidates[insert] = node;
            distances[insert] = distanceSquared;
            if (candidateCount < capacity) candidateCount++;
        }
        if (candidateCount < 2) return false;

        BasisSolution best = new BasisSolution();
        boolean foundBasis = false;
        for (int first = 0; first < candidateCount - 1; first++) {
            for (int second = first + 1; second < candidateCount; second++) {
                foundBasis |= considerBasis(nodes, centerNode, candidates[first], candidates[second],
                        vx, vy, vz, distances[first] + distances[second], best);
            }
        }
        if (!foundBasis) return false;

        ExplicitZSolution explicitZ = explicitZMode != ExplicitZMode.DISABLED
                ? findExplicitZ(nodes, beamGraph, centerNode, best,
                        candidates, distances, candidateCount, vx, vy, vz, explicitZMode)
                : new ExplicitZSolution();

        if (explicitZ.found) {
            double normalLength = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            double normalScale = normalLength > MIN_INPUT_NORMAL_LENGTH ? 1.0 / normalLength : 0.0;
            double[] normalWeights = solveBasisCoordinates(
                    normX * normalScale, normY * normalScale, normZ * normalScale,
                    best.uX, best.uY, best.uZ,
                    best.vX, best.vY, best.vZ,
                    explicitZ.zX, explicitZ.zY, explicitZ.zZ,
                    explicitZ.determinant);

            flex.vCenterNode[ptr] = centerNode;
            flex.vVxNode[ptr] = best.vxNode;
            flex.vVyNode[ptr] = best.vyNode;
            flex.vVzNode[ptr] = explicitZ.vzNode;
            flex.vWeightX[ptr] = explicitZ.weightX;
            flex.vWeightY[ptr] = explicitZ.weightY;
            flex.vWeightZ[ptr] = explicitZ.weightZ;
            flex.vUseCrossZ[ptr] = false;
            flex.vRestCrossLength[ptr] = restCrossLength(best);
            if (flex.vNormWeightX != null) {
                flex.vNormWeightX[ptr] = (float) normalWeights[0];
                flex.vNormWeightY[ptr] = (float) normalWeights[1];
                flex.vNormWeightZ[ptr] = (float) normalWeights[2];
            }
            return true;
        }

        float normalWeightX = 0, normalWeightY = 0, normalWeightZ = 1;
        if (flex.vNormWeightX != null) {
            double normalLength = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            if (normalLength > MIN_INPUT_NORMAL_LENGTH) {
                double inX = normX / normalLength, inY = normY / normalLength, inZ = normZ / normalLength;
                double normalZ = inX * best.nX + inY * best.nY + inZ * best.nZ;
                double planarX = inX - normalZ * best.nX;
                double planarY = inY - normalZ * best.nY;
                double planarZ = inZ - normalZ * best.nZ;
                double dotU = planarX * best.uX + planarY * best.uY + planarZ * best.uZ;
                double dotV = planarX * best.vX + planarY * best.vY + planarZ * best.vZ;
                normalWeightX = (float) ((dotU * best.vLengthSquared - dotV * best.axisDot) * best.inverseDeterminant);
                normalWeightY = (float) ((dotV * best.uLengthSquared - dotU * best.axisDot) * best.inverseDeterminant);
                normalWeightZ = (float) normalZ;
            }
            flex.vNormWeightX[ptr] = normalWeightX;
            flex.vNormWeightY[ptr] = normalWeightY;
            flex.vNormWeightZ[ptr] = normalWeightZ;
        }

        flex.vCenterNode[ptr] = centerNode;
        flex.vVxNode[ptr] = best.vxNode;
        flex.vVyNode[ptr] = best.vyNode;
        flex.vVzNode[ptr] = -1;
        flex.vWeightX[ptr] = best.weightX;
        flex.vWeightY[ptr] = best.weightY;
        flex.vWeightZ[ptr] = (float) best.weightZ;
        flex.vUseCrossZ[ptr] = true;
        flex.vRestCrossLength[ptr] = restCrossLength(best);
        return true;
    }

    private static float restCrossLength(BasisSolution basis) {
        return (float) Math.sqrt(basis.uLengthSquared * basis.vLengthSquared * basis.sineSquared);
    }

    static boolean retainExplicitZForMesh(int explicitVertices, int totalVertices) {
        return totalVertices > 0
                && explicitVertices >= Math.ceil(totalVertices * MIN_EXPLICIT_Z_MESH_COVERAGE);
    }

    private static double[] reconstructRestNormal(FlexbodyContainer flex, NodeContainer nodes, int vertex) {
        int center = flex.vCenterNode[vertex];
        int vx = flex.vVxNode[vertex];
        int vy = flex.vVyNode[vertex];
        double ux = nodes.baseX[vx] - nodes.baseX[center];
        double uy = nodes.baseY[vx] - nodes.baseY[center];
        double uz = nodes.baseZ[vx] - nodes.baseZ[center];
        double vxAxis = nodes.baseX[vy] - nodes.baseX[center];
        double vyAxis = nodes.baseY[vy] - nodes.baseY[center];
        double vzAxis = nodes.baseZ[vy] - nodes.baseZ[center];
        double zx, zy, zz;
        int vz = flex.vVzNode[vertex];
        if (vz >= 0) {
            zx = nodes.baseX[vz] - nodes.baseX[center];
            zy = nodes.baseY[vz] - nodes.baseY[center];
            zz = nodes.baseZ[vz] - nodes.baseZ[center];
        } else {
            zx = uy * vzAxis - uz * vyAxis;
            zy = uz * vxAxis - ux * vzAxis;
            zz = ux * vyAxis - uy * vxAxis;
            double length = Math.sqrt(zx * zx + zy * zy + zz * zz);
            if (length > 0.0) {
                zx /= length;
                zy /= length;
                zz /= length;
            }
        }
        return new double[]{
                flex.vNormWeightX[vertex] * ux + flex.vNormWeightY[vertex] * vxAxis + flex.vNormWeightZ[vertex] * zx,
                flex.vNormWeightX[vertex] * uy + flex.vNormWeightY[vertex] * vyAxis + flex.vNormWeightZ[vertex] * zy,
                flex.vNormWeightX[vertex] * uz + flex.vNormWeightY[vertex] * vzAxis + flex.vNormWeightZ[vertex] * zz
        };
    }

    private static ExplicitZSolution findExplicitZ(NodeContainer nodes, BeamGraph beamGraph, int centerNode,
                                                    BasisSolution planarBasis,
                                                    int[] candidates, double[] distances,
                                                    int candidateCount,
                                                    double px, double py, double pz,
                                                    ExplicitZMode mode) {
        ExplicitZSolution best = new ExplicitZSolution();
        double cx = nodes.baseX[centerNode], cy = nodes.baseY[centerNode], cz = nodes.baseZ[centerNode];
        double dX = px - cx, dY = py - cy, dZ = pz - cz;
        double crossX = planarBasis.uY * planarBasis.vZ - planarBasis.uZ * planarBasis.vY;
        double crossY = planarBasis.uZ * planarBasis.vX - planarBasis.uX * planarBasis.vZ;
        double crossZ = planarBasis.uX * planarBasis.vY - planarBasis.uY * planarBasis.vX;
        double crossLengthSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;

        for (int i = 0; i < candidateCount; i++) {
            int vzNode = candidates[i];
            if (vzNode == planarBasis.vxNode || vzNode == planarBasis.vyNode) continue;
            int topologyScore = localCageScore(beamGraph, vzNode, centerNode,
                    planarBasis.vxNode, planarBasis.vyNode);
            if (topologyScore == Integer.MAX_VALUE) continue;
            double zX = nodes.baseX[vzNode] - cx;
            double zY = nodes.baseY[vzNode] - cy;
            double zZ = nodes.baseZ[vzNode] - cz;
            double zLengthSquared = zX * zX + zY * zY + zZ * zZ;
            if (zLengthSquared < MIN_AXIS_LENGTH_SQUARED) continue;

            double determinant = crossX * zX + crossY * zY + crossZ * zZ;
            double outOfPlaneSineSquared = determinant * determinant / (crossLengthSquared * zLengthSquared);
            if (outOfPlaneSineSquared < squaredSine(MIN_USABLE_BASIS_ANGLE_DEGREES)) continue;

            double[] weights = solveBasisCoordinates(dX, dY, dZ,
                    planarBasis.uX, planarBasis.uY, planarBasis.uZ,
                    planarBasis.vX, planarBasis.vY, planarBasis.vZ,
                    zX, zY, zZ, determinant);
            if (!finiteAndSafe(weights)) continue;
            double affineGain = affineNodeGain(weights[0], weights[1], weights[2]);
            if (affineGain > MAX_AFFINE_NODE_GAIN) continue;
            if (mode == ExplicitZMode.CONFIDENCE_PROBE
                    && affineGain + EXPLICIT_Z_SENSITIVITY_MARGIN >= crossBasisSensitivity(planarBasis)) continue;

            boolean preferredAngle = outOfPlaneSineSquared >= squaredSine(PREFERRED_BASIS_ANGLE_DEGREES);
            boolean preferredCoordinates = withinPreferredLocatorBounds(weights[0])
                    && withinPreferredLocatorBounds(weights[1])
                    && withinPreferredLocatorBounds(weights[2]);
            int tier = BasisSolution.preferenceTier(preferredAngle, preferredCoordinates);
            if (best.accepts(topologyScore, tier, distances[i], outOfPlaneSineSquared)) {
                best.set(vzNode, zX, zY, zZ, determinant,
                        (float) weights[0], (float) weights[1], (float) weights[2],
                        topologyScore, tier, distances[i], outOfPlaneSineSquared);
            }
        }
        return best;
    }

    /**
     * A fourth locator node is useful only when it belongs to the same compact
     * structural cage as the planar basis. Spatial proximity alone can select a
     * node from a bumper, lamp or liner that later detaches independently.
     */
    static int localCageScore(BeamGraph graph, int vzNode, int centerNode, int vxNode, int vyNode) {
        if (graph == null) return 0;
        int[] anchors = {centerNode, vxNode, vyNode};
        int directConnections = 0;
        for (int anchor : anchors) {
            if (graph.cohesivelyConnected(vzNode, anchor)) directConnections++;
            int hops = graph.hopDistance(vzNode, anchor, MAX_COHESIVE_CAGE_HOPS, true);
            if (hops < 0) return Integer.MAX_VALUE;
        }
        if (directConnections == 0) return Integer.MAX_VALUE;

        // Fewer missing direct links means a more cohesive local cage. The hop
        // checks above are a validity gate, not another weighted heuristic.
        return anchors.length - directConnections;
    }

    private static double[] solveBasisCoordinates(double dX, double dY, double dZ,
                                                  double uX, double uY, double uZ,
                                                  double vX, double vY, double vZ,
                                                  double zX, double zY, double zZ,
                                                  double determinant) {
        double vCrossZX = vY * zZ - vZ * zY;
        double vCrossZY = vZ * zX - vX * zZ;
        double vCrossZZ = vX * zY - vY * zX;
        double dCrossZX = dY * zZ - dZ * zY;
        double dCrossZY = dZ * zX - dX * zZ;
        double dCrossZZ = dX * zY - dY * zX;
        double uCrossVX = uY * vZ - uZ * vY;
        double uCrossVY = uZ * vX - uX * vZ;
        double uCrossVZ = uX * vY - uY * vX;
        return new double[]{
                (dX * vCrossZX + dY * vCrossZY + dZ * vCrossZZ) / determinant,
                (uX * dCrossZX + uY * dCrossZY + uZ * dCrossZZ) / determinant,
                (uCrossVX * dX + uCrossVY * dY + uCrossVZ * dZ) / determinant
        };
    }

    private static boolean finiteAndSafe(double[] coordinates) {
        for (double coordinate : coordinates) {
            if (!Double.isFinite(coordinate) || Math.abs(coordinate) > MAX_SAFE_LOCATOR_MAGNITUDE) return false;
        }
        return true;
    }

    private static boolean considerBasis(NodeContainer nodes, int centerNode, int vxNode, int vyNode,
                                         double px, double py, double pz, double pairDistanceSquared,
                                         BasisSolution best) {
        double cx = nodes.baseX[centerNode], cy = nodes.baseY[centerNode], cz = nodes.baseZ[centerNode];
        double uX = nodes.baseX[vxNode] - cx, uY = nodes.baseY[vxNode] - cy, uZ = nodes.baseZ[vxNode] - cz;
        double vX = nodes.baseX[vyNode] - cx, vY = nodes.baseY[vyNode] - cy, vZ = nodes.baseZ[vyNode] - cz;
        double uLengthSquared = uX * uX + uY * uY + uZ * uZ;
        double vLengthSquared = vX * vX + vY * vY + vZ * vZ;
        if (uLengthSquared < MIN_AXIS_LENGTH_SQUARED || vLengthSquared < MIN_AXIS_LENGTH_SQUARED) return false;

        double axisDot = uX * vX + uY * vY + uZ * vZ;
        double cosineSquared = axisDot * axisDot / (uLengthSquared * vLengthSquared);
        double sineSquared = 1.0 - Math.min(1.0, Math.max(0.0, cosineSquared));
        if (sineSquared < squaredSine(MIN_USABLE_BASIS_ANGLE_DEGREES)) return false;

        double crossX = uY * vZ - uZ * vY;
        double crossY = uZ * vX - uX * vZ;
        double crossZ = uX * vY - uY * vX;
        double crossLengthSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
        if (crossLengthSquared < MIN_BASIS_NORMAL_LENGTH_SQUARED) return false;
        double inverseCrossLength = 1.0 / Math.sqrt(crossLengthSquared);
        double nX = crossX * inverseCrossLength, nY = crossY * inverseCrossLength, nZ = crossZ * inverseCrossLength;

        double dX = px - cx, dY = py - cy, dZ = pz - cz;
        double weightZ = dX * nX + dY * nY + dZ * nZ;
        double planarX = dX - weightZ * nX, planarY = dY - weightZ * nY, planarZ = dZ - weightZ * nZ;
        double determinant = uLengthSquared * vLengthSquared - axisDot * axisDot;
        if (determinant <= MIN_BASIS_NORMAL_LENGTH_SQUARED) return false;
        double inverseDeterminant = 1.0 / determinant;
        double dotU = planarX * uX + planarY * uY + planarZ * uZ;
        double dotV = planarX * vX + planarY * vY + planarZ * vZ;
        float weightX = (float) ((dotU * vLengthSquared - dotV * axisDot) * inverseDeterminant);
        float weightY = (float) ((dotV * uLengthSquared - dotU * axisDot) * inverseDeterminant);
        if (!Float.isFinite(weightX) || !Float.isFinite(weightY) || !Double.isFinite(weightZ)
                || Math.abs(weightX) > MAX_SAFE_LOCATOR_MAGNITUDE
                || Math.abs(weightY) > MAX_SAFE_LOCATOR_MAGNITUDE
                || Math.abs(weightZ) > MAX_SAFE_LOCATOR_MAGNITUDE) return false;
        boolean preferredAngle = sineSquared >= squaredSine(PREFERRED_BASIS_ANGLE_DEGREES);
        boolean preferredCoordinates = withinPreferredLocatorBounds(weightX) && withinPreferredLocatorBounds(weightY);
        int preferenceTier = BasisSolution.preferenceTier(preferredAngle, preferredCoordinates);
        double affineGain = affineNodeGain(weightX, weightY, 0.0);
        double stabilityScore = pairDistanceSquared * affineGain / sineSquared;
        if (!best.accepts(stabilityScore, preferenceTier, pairDistanceSquared, sineSquared)) return false;
        best.set(vxNode, vyNode, uX, uY, uZ, vX, vY, vZ, nX, nY, nZ,
                uLengthSquared, vLengthSquared, axisDot, inverseDeterminant, weightX, weightY, weightZ,
                stabilityScore, pairDistanceSquared, sineSquared, preferenceTier);
        return true;
    }

    private static double squaredSine(double degrees) {
        double sine = Math.sin(Math.toRadians(degrees));
        return sine * sine;
    }

    private static boolean withinPreferredLocatorBounds(double value) {
        return value >= PREFERRED_LOCATOR_MIN && value <= PREFERRED_LOCATOR_MAX;
    }

    static double affineNodeGain(double weightX, double weightY, double weightZ) {
        double centerWeight = 1.0 - weightX - weightY - weightZ;
        return Math.abs(centerWeight) + Math.abs(weightX) + Math.abs(weightY) + Math.abs(weightZ);
    }

    /**
     * First-order displacement sensitivity of the three-node representation.
     * The affine term covers translation of the planar locators; the second
     * term estimates how strongly the metric normal offset reacts when either
     * planar axis rotates.
     */
    private static double crossBasisSensitivity(BasisSolution basis) {
        double sine = Math.sqrt(basis.sineSquared);
        double inverseAxisScale = 1.0 / Math.sqrt(basis.uLengthSquared)
                + 1.0 / Math.sqrt(basis.vLengthSquared);
        double normalRotationGain = Math.abs(basis.weightZ) * inverseAxisScale / sine;
        return affineNodeGain(basis.weightX, basis.weightY, 0.0) + normalRotationGain;
    }

    private static final class BasisSolution {
        int vxNode, vyNode;
        double uX, uY, uZ, vX, vY, vZ, nX, nY, nZ;
        double uLengthSquared, vLengthSquared, axisDot, inverseDeterminant;
        float weightX, weightY;
        double weightZ;
        double pairDistanceSquared = Double.POSITIVE_INFINITY;
        double stabilityScore = Double.POSITIVE_INFINITY;
        double sineSquared;
        int preferenceTier = Integer.MAX_VALUE;

        static int preferenceTier(boolean preferredAngle, boolean preferredCoordinates) {
            if (preferredAngle && preferredCoordinates) return PREFERRED_ANGLE_AND_COORDINATES;
            if (preferredAngle) return PREFERRED_ANGLE_ONLY;
            if (preferredCoordinates) return PREFERRED_COORDINATES_ONLY;
            return USABLE_FALLBACK;
        }

        private static final int PREFERRED_ANGLE_AND_COORDINATES = 0;
        private static final int PREFERRED_ANGLE_ONLY = 1;
        private static final int PREFERRED_COORDINATES_ONLY = 2;
        private static final int USABLE_FALLBACK = 3;

        boolean accepts(double candidateScore, int tier, double distanceSquared, double candidateSineSquared) {
            int scoreOrder = Double.compare(candidateScore, stabilityScore);
            if (scoreOrder != 0) return scoreOrder < 0;
            if (tier != preferenceTier) return tier < preferenceTier;
            int distanceOrder = Double.compare(distanceSquared, pairDistanceSquared);
            return distanceOrder < 0 || (distanceOrder == 0 && candidateSineSquared > sineSquared);
        }

        void set(int vxNode, int vyNode,
                 double uX, double uY, double uZ, double vX, double vY, double vZ,
                 double nX, double nY, double nZ,
                 double uLengthSquared, double vLengthSquared, double axisDot, double inverseDeterminant,
                 float weightX, float weightY, double weightZ,
                 double stabilityScore, double pairDistanceSquared, double sineSquared, int preferenceTier) {
            this.vxNode = vxNode;
            this.vyNode = vyNode;
            this.uX = uX; this.uY = uY; this.uZ = uZ;
            this.vX = vX; this.vY = vY; this.vZ = vZ;
            this.nX = nX; this.nY = nY; this.nZ = nZ;
            this.uLengthSquared = uLengthSquared;
            this.vLengthSquared = vLengthSquared;
            this.axisDot = axisDot;
            this.inverseDeterminant = inverseDeterminant;
            this.weightX = weightX;
            this.weightY = weightY;
            this.weightZ = weightZ;
            this.stabilityScore = stabilityScore;
            this.pairDistanceSquared = pairDistanceSquared;
            this.sineSquared = sineSquared;
            this.preferenceTier = preferenceTier;
        }
    }

    private static final class ExplicitZSolution {
        boolean found;
        int vzNode;
        double zX, zY, zZ;
        double determinant;
        float weightX, weightY, weightZ;
        int topologyScore = Integer.MAX_VALUE;
        int preferenceTier = Integer.MAX_VALUE;
        double distanceSquared = Double.POSITIVE_INFINITY;
        double sineSquared;

        boolean accepts(int candidateTopologyScore, int tier,
                        double candidateDistanceSquared, double candidateSineSquared) {
            if (tier != preferenceTier) return tier < preferenceTier;
            int distanceOrder = Double.compare(candidateDistanceSquared, distanceSquared);
            if (distanceOrder != 0) return distanceOrder < 0;
            if (candidateTopologyScore != topologyScore) return candidateTopologyScore < topologyScore;
            return candidateSineSquared > sineSquared;
        }

        void set(int vzNode, double zX, double zY, double zZ, double determinant,
                 float weightX, float weightY, float weightZ,
                 int topologyScore, int preferenceTier, double distanceSquared, double sineSquared) {
            found = true;
            this.vzNode = vzNode;
            this.zX = zX;
            this.zY = zY;
            this.zZ = zZ;
            this.determinant = determinant;
            this.weightX = weightX;
            this.weightY = weightY;
            this.weightZ = weightZ;
            this.topologyScore = topologyScore;
            this.preferenceTier = preferenceTier;
            this.distanceSquared = distanceSquared;
            this.sineSquared = sineSquared;
        }
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
        flex.vVzNode[ptr]     = -1;
        flex.vWeightX[ptr]    = 0.0f; flex.vWeightY[ptr]    = 0.0f; flex.vWeightZ[ptr]    = 0.0f;
        flex.vUseCrossZ[ptr]  = false;
        flex.vRestCrossLength[ptr] = 0.0f;

        flex.skinnedPosX[ptr] = (float) (vx - nodes.baseX[bestC]);
        flex.skinnedPosY[ptr] = (float) (vy - nodes.baseY[bestC]);
        flex.skinnedPosZ[ptr] = (float) (vz - nodes.baseZ[bestC]);

        if (flex.vNormWeightX != null) {
            double nLen = Math.sqrt(normX * normX + normY * normY + normZ * normZ);
            flex.vNormWeightX[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normX / nLen : 0);
            flex.vNormWeightY[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normY / nLen : 1);
            flex.vNormWeightZ[ptr] = (float)(nLen > MIN_INPUT_NORMAL_LENGTH ? normZ / nLen : 0);
        }
    }
}
