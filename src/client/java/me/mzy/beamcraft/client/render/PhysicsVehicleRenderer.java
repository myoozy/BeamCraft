package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.model.FlexbodyBindingUtil;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.client.render.*;
import net.minecraft.util.Identifier;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.math.MatrixStack;
import org.joml.Matrix4f;

public class PhysicsVehicleRenderer extends EntityRenderer<PhysicsVehicleEntity> {

    private static final Identifier DEFAULT_TEXTURE = Identifier.of("beamcraft", "textures/entity/vehicle_default.png");

    // 🌟 排查总闸：设为 true 将彻底屏蔽软体节点形变
    private final boolean DEBUG_STATIC_RENDER = false;

    float[] interpNodeX = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeY = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeZ = new float[NodeContainer.INIT_NODE_CAP];

    public PhysicsVehicleRenderer(EntityRendererFactory.Context ctx) { super(ctx); }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) { return DEFAULT_TEXTURE; }

    /**
     * 🚀 极限性能加速器：快速平方根倒数 (Fast InvSqrt)
     * 消除高频着色指令阻塞，确保每帧数万个顶点法线重构全域丝滑。
     */
    public static float fastInvSqrt(float x) {
        float xhalf = 0.5f * x;
        int i = Float.floatToRawIntBits(x);
        i = 0x5f3759df - (i >> 1);
        x = Float.intBitsToFloat(i);
        x = x * (1.5f - xhalf * x * x);
        return x;
    }

    @Override
    public void render(PhysicsVehicleEntity entity, float entityYaw, float partialTicks, MatrixStack matrixStack, VertexConsumerProvider buffer, int packedLight) {
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) return;

        if (!vehicle.flexbodies.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle);
        }

        NodeContainer nodes = vehicle.nodes;
        FlexbodyContainer flex = vehicle.flexbodies;
        int vCount = flex.totalVertexCount;
        if (vCount == 0) return;

        MatrixStack.Entry entry = matrixStack.peek();
        Matrix4f modelMat = entry.getPositionMatrix();
        VertexConsumer builder = buffer.getBuffer(RenderLayer.getEntityCutoutNoCull(getTexture(entity)));

        int globalVertPtr = 0;

        // 提取实体的基准渲染原点锚点坐标
        float entityAnchorX = (float)(nodes.renderSnapPrevX[0] + (nodes.renderSnapCurrX[0] - nodes.renderSnapPrevX[0]) * partialTicks);
        float entityAnchorY = (float)(nodes.renderSnapPrevY[0] + (nodes.renderSnapCurrY[0] - nodes.renderSnapPrevY[0]) * partialTicks);
        float entityAnchorZ = (float)(nodes.renderSnapPrevZ[0] + (nodes.renderSnapCurrZ[0] - nodes.renderSnapPrevZ[0]) * partialTicks);

        // =====================================================================
        // 静态排查视觉层 (完美呈现 Gouraud Shading 高光着色)
        // =====================================================================
        if (DEBUG_STATIC_RENDER) {
            for (int m = 0; m < flex.meshCount; m++) {
                String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
                DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
                if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
                if (geom == null) continue;

                for (int v = 0; v < geom.vertexCount; v++) {
                    int i = globalVertPtr;
                    // 🌟 修复隐身与错位：动态解算顶点的 skinnedPosX 存的是绝对坐标，刚体跟随点存的是相对偏移，需对齐图纸参考系
                    float staticX = flex.vUseCrossZ[i] ? flex.skinnedPosX[i] : (float)nodes.baseX[flex.vCenterNode[i]] + flex.skinnedPosX[i];
                    float staticY = flex.vUseCrossZ[i] ? flex.skinnedPosY[i] : (float)nodes.baseY[flex.vCenterNode[i]] + flex.skinnedPosY[i];
                    float staticZ = flex.vUseCrossZ[i] ? flex.skinnedPosZ[i] : (float)nodes.baseZ[flex.vCenterNode[i]] + flex.skinnedPosZ[i];

                    float finalX = entityAnchorX + staticX;
                    float finalY = entityAnchorY + staticY;
                    float finalZ = entityAnchorZ + staticZ;

                    // 静态重构高保真法线
                    float nx = 0, ny = 1, nz = 0;
                    if (flex.vNormWeightX != null) {
                        if (flex.vUseCrossZ[i]) {
                            int nRef = flex.vCenterNode[i], nX = flex.vVxNode[i], nY = flex.vVyNode[i];
                            double uX = nodes.baseX[nX] - nodes.baseX[nRef], uY = nodes.baseY[nX] - nodes.baseY[nRef], uZ = nodes.baseZ[nX] - nodes.baseZ[nRef];
                            double vX = nodes.baseX[nY] - nodes.baseX[nRef], vY = nodes.baseY[nY] - nodes.baseY[nRef], vZ = nodes.baseZ[nY] - nodes.baseZ[nRef];
                            double bNx = uY * vZ - uZ * vY, bNy = uZ * vX - uX * vZ, bNz = uX * vY - uY * vX;
                            double lenN = Math.sqrt(bNx * bNx + bNy * bNy + bNz * bNz);
                            if (lenN > 1e-7) { bNx /= lenN; bNy /= lenN; bNz /= lenN; }
                            double lenU = Math.sqrt(uX * uX + uY * uY + uZ * uZ);
                            double e1x = uX / lenU, e1y = uY / lenU, e1z = uZ / lenU;
                            double e2x = bNy * e1z - bNz * e1y, e2y = bNz * e1x - bNx * e1z, e2z = bNx * e1y - bNy * e1x;
                            float nwX = flex.vNormWeightX[i], nwY = flex.vNormWeightY[i], nwZ = flex.vNormWeightZ[i];
                            nx = (float)(nwX * e1x + nwY * e2x + nwZ * bNx);
                            ny = (float)(nwX * e1y + nwY * e2y + nwZ * bNy);
                            nz = (float)(nwX * e1z + nwY * e2z + nwZ * bNz);
                        } else {
                            nx = flex.vNormWeightX[i]; ny = flex.vNormWeightY[i]; nz = flex.vNormWeightZ[i];
                        }
                    }

                    builder.vertex(modelMat, finalX, finalY, finalZ)
                            .color(255, 255, 255, 255)
                            .texture(flex.uvU[i], flex.uvV[i])
                            .overlay(OverlayTexture.DEFAULT_UV)
                            .light(packedLight)
                            .normal(entry, nx, ny, nz);
                    globalVertPtr++;
                }
            }
            super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
            return;
        }

        int nodeCount = nodes.count;
        if (nodeCount > interpNodeX.length) {
            interpNodeX = Utility.expand(interpNodeX, nodeCount);
            interpNodeY = Utility.expand(interpNodeY, nodeCount);
            interpNodeZ = Utility.expand(interpNodeZ, nodeCount);
        }

        for (int n = 0; n < nodeCount; n++) {
            interpNodeX[n] = (float)(nodes.renderSnapPrevX[n] + (nodes.renderSnapCurrX[n] - nodes.renderSnapPrevX[n]) * partialTicks);
            interpNodeY[n] = (float)(nodes.renderSnapPrevY[n] + (nodes.renderSnapCurrY[n] - nodes.renderSnapPrevY[n]) * partialTicks);
            interpNodeZ[n] = (float)(nodes.renderSnapPrevZ[n] + (nodes.renderSnapCurrZ[n] - nodes.renderSnapPrevZ[n]) * partialTicks);
        }

        float lastX = 0, lastY = 0, lastZ = 0;
        float lastNx = 0, lastNy = 1, lastNz = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            for (int v = 0; v < geom.vertexCount; v++) {
                int i = globalVertPtr;
                float finalX, finalY, finalZ;
                float nx, ny, nz;

                // 🚀 QUADS 退化点跳过优化：维持 25% 的显存推流性能减负
                if ((v & 3) == 3) {
                    finalX = lastX; finalY = lastY; finalZ = lastZ;
                    nx = lastNx; ny = lastNy; nz = lastNz;
                } else {
                    int nRef = flex.vCenterNode[i];
                    float cx = interpNodeX[nRef], cy = interpNodeY[nRef], cz = interpNodeZ[nRef];

                    if (flex.vUseCrossZ[i]) {
                        int nX = flex.vVxNode[i];
                        int nY = flex.vVyNode[i];

                        float vxX = interpNodeX[nX] - cx, vxY = interpNodeY[nX] - cy, vxZ = interpNodeZ[nX] - cz;
                        float vyX = interpNodeX[nY] - cx, vyY = interpNodeY[nY] - cy, vyZ = interpNodeZ[nY] - cz;

                        // 提取当前伴随平面重构单位法线
                        float baseNx = vxY * vyZ - vxZ * vyY;
                        float baseNy = vxZ * vyX - vxX * vyZ;
                        float baseNz = vxX * vyY - vxY * vyX;
                        float lenSq = baseNx * baseNx + baseNy * baseNy + baseNz * baseNz;

                        if (lenSq > 1e-10f) {
                            float invLen = fastInvSqrt(lenSq);
                            baseNx *= invLen; baseNy *= invLen; baseNz *= invLen;
                        }

                        // 位置解算保持原有高效插值
                        float wX = flex.vWeightX[i], wY = flex.vWeightY[i], wZ = flex.vWeightZ[i];
                        finalX = cx + wX * vxX + wY * vyX + wZ * baseNx;
                        finalY = cy + wX * vxY + wY * vyY + wZ * baseNy;
                        finalZ = cz + wX * vxZ + wY * vyZ + wZ * baseNz;

                        // 🌟 1:1 复刻 RoR 源码的动态法线重构逻辑！
                        // 摒弃强制正交基，直接拥抱软体的倾斜伴随矩阵，物理拉伸时光影会自然跟随撕扯。
                        if (flex.vNormWeightX != null) {
                            float nwX = flex.vNormWeightX[i], nwY = flex.vNormWeightY[i], nwZ = flex.vNormWeightZ[i];

                            // 完美的基底线性组合 (Linear Combination of Basis Vectors)
                            float rawNx = vxX * nwX + vyX * nwY + baseNx * nwZ;
                            float rawNy = vxY * nwX + vyY * nwY + baseNy * nwZ;
                            float rawNz = vxZ * nwX + vyZ * nwY + baseNz * nwZ;

                            // 软体动态归一化
                            float nLenSq = rawNx * rawNx + rawNy * rawNy + rawNz * rawNz;
                            if (nLenSq > 1e-10f) {
                                float nInv = fastInvSqrt(nLenSq);
                                nx = rawNx * nInv; ny = rawNy * nInv; nz = rawNz * nInv;
                            } else { nx = baseNx; ny = baseNy; nz = baseNz; }
                        } else { nx = baseNx; ny = baseNy; nz = baseNz; }

                    } else {
                        // 刚体跟随保护
                        finalX = cx + flex.skinnedPosX[i];
                        finalY = cy + flex.skinnedPosY[i];
                        finalZ = cz + flex.skinnedPosZ[i];

                        nx = flex.vNormWeightX != null ? flex.vNormWeightX[i] : 0;
                        ny = flex.vNormWeightY != null ? flex.vNormWeightY[i] : 1;
                        nz = flex.vNormWeightZ != null ? flex.vNormWeightZ[i] : 0;
                    }

                    if ((v & 3) == 2) {
                        lastX = finalX; lastY = finalY; lastZ = finalZ;
                        lastNx = nx; lastNy = ny; lastNz = nz;
                    }
                }

                builder.vertex(modelMat, finalX, finalY, finalZ)
                        .color(255, 255, 255, 255)
                        .texture(flex.uvU[i], flex.uvV[i])
                        .overlay(OverlayTexture.DEFAULT_UV)
                        .light(packedLight)
                        .normal(entry, nx, ny, nz);

                globalVertPtr++;
            }
        }
        super.render(entity, entityYaw, partialTicks, matrixStack, buffer, packedLight);
    }
}