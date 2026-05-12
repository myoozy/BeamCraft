package me.mzy.beamcraft.client.render;

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
    private final boolean DEBUG_STATIC_RENDER = true;

    float[] interpNodeX = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeY = new float[NodeContainer.INIT_NODE_CAP];
    float[] interpNodeZ = new float[NodeContainer.INIT_NODE_CAP];

    public PhysicsVehicleRenderer(EntityRendererFactory.Context ctx) { super(ctx); }

    @Override
    public Identifier getTexture(PhysicsVehicleEntity entity) { return DEFAULT_TEXTURE; }

    @Override
    public void render(PhysicsVehicleEntity entity, float entityYaw, float partialTicks, MatrixStack matrixStack, VertexConsumerProvider buffer, int packedLight) {
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) return;
        if (true)return;

        if (!vehicle.flexbodies.isSkinningBound) {
            FlexbodyBindingUtil.performBinding(vehicle.flexbodies, vehicle.nodes);
        }

        NodeContainer nodes = vehicle.nodes;
        FlexbodyContainer flex = vehicle.flexbodies;
        int vCount = flex.totalVertexCount;
        if (vCount == 0) return;

        MatrixStack.Entry entry = matrixStack.peek();
        Matrix4f modelMat = entry.getPositionMatrix();
        VertexConsumer builder = buffer.getBuffer(RenderLayer.getEntityCutoutNoCull(getTexture(entity)));

        // 维护内部图元索引指针用于读取原始法线缓冲
        int globalVertPtr = 0;

        if (DEBUG_STATIC_RENDER) {
            for (int m = 0; m < flex.meshCount; m++) {
                String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
                DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
                if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
                if (geom == null) continue;

                for (int v = 0; v < geom.vertexCount; v++) {
                    float nx = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3] : 0;
                    float ny = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3 + 1] : 1;
                    float nz = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3 + 2] : 0;

                    builder.vertex(modelMat, flex.skinnedPosX[globalVertPtr], flex.skinnedPosY[globalVertPtr], flex.skinnedPosZ[globalVertPtr])
                            .color(255, 255, 255, 255)
                            .texture(flex.uvU[globalVertPtr], flex.uvV[globalVertPtr])
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

        for (int m = 0; m < flex.meshCount; m++) {
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            for (int v = 0; v < geom.vertexCount; v++) {
                int i = globalVertPtr;
                int nRef = flex.vCenterNode[i];
                int nX   = flex.vVxNode[i];
                int nY   = flex.vVyNode[i];

                float cx = interpNodeX[nRef], cy = interpNodeY[nRef], cz = interpNodeZ[nRef];
                float vxX = interpNodeX[nX] - cx, vxY = interpNodeY[nX] - cy, vxZ = interpNodeZ[nX] - cz;

                // ✅ 彻底修复审查指出的第 83 行致命笔误：改为 interpNodeX
                float vyX = interpNodeX[nY] - cx;
                float vyY = interpNodeY[nY] - cy;
                float vyZ = interpNodeZ[nY] - cz;

                // 还原当前形变拓扑面的单位法向
                float nx = vxY * vyZ - vxZ * vyY;
                float ny = vxZ * vyX - vxX * vyZ;
                float nz = vxX * vyY - vxY * vyX;
                float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);

                if (len > 1e-7f) {
                    float invLen = 1.0f / len;
                    nx *= invLen; ny *= invLen; nz *= invLen;
                } else {
                    // 退化时读取模型原生平坦法线进行平滑接管
                    nx = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3] : 0;
                    ny = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3 + 1] : 1;
                    nz = geom.normals != null && v * 3 + 2 < geom.normals.length ? geom.normals[v * 3 + 2] : 0;
                }

                float wX = flex.vWeightX[i], wY = flex.vWeightY[i], wZ = flex.vWeightZ[i];

                float finalX = cx + wX * vxX + wY * vyX + wZ * nx;
                float finalY = cy + wX * vxY + wY * vyY + wZ * ny;
                float finalZ = cz + wX * vxZ + wY * vyZ + wZ * nz;

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