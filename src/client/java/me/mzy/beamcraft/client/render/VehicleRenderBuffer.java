package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.*;
import net.minecraft.client.util.math.MatrixStack;

import static me.mzy.beamcraft.client.render.PhysicsVehicleRenderer.fastInvSqrt;

public class VehicleRenderBuffer {
    // 🌟 使用 Minecraft 官方封装的 VBO 管理器！自动处理所有恶心的 OpenGL 状态！
    private VertexBuffer vbo;

    public boolean isVboNull() {
        return vbo == null;
    }

    public void init() {
        // 声明这是一个动态更新的缓冲 (DYNAMIC)
        this.vbo = new VertexBuffer(VertexBuffer.Usage.DYNAMIC);
    }

    public void free() {
        if (this.vbo != null) {
            this.vbo.close(); // 安全释放显存
            this.vbo = null;
        }
    }

    // 将更新和绘制合并，代码更紧凑
    public void updateAndDraw(FlexbodyContainer flex, float[] interpNodeX, float[] interpNodeY, float[] interpNodeZ, int packedLight, MatrixStack matrixStack) {
        if (this.vbo == null || flex.totalVertexCount == 0) return;

        // 1. 召唤原生高能 BufferBuilder (它底层使用 Unsafe 内存直写，速度极快)
        Tessellator tessellator = Tessellator.getInstance();
        BufferBuilder builder = tessellator.begin(VertexFormat.DrawMode.TRIANGLES, VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL);

        int globalVertPtr = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            if (flex.meshName[m].isEmpty()) continue;
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            for (int v = 0; v < geom.vertexCount; v++) {
                int i = globalVertPtr;
                float finalX, finalY, finalZ, nx, ny, nz;

                int nRef = flex.vCenterNode[i];
                float cx = interpNodeX[nRef], cy = interpNodeY[nRef], cz = interpNodeZ[nRef];

                if (flex.vUseCrossZ[i]) {
                    int nX = flex.vVxNode[i];
                    int nY = flex.vVyNode[i];

                    float vxX = interpNodeX[nX] - cx, vxY = interpNodeY[nX] - cy, vxZ = interpNodeZ[nX] - cz;
                    float vyX = interpNodeX[nY] - cx, vyY = interpNodeY[nY] - cy, vyZ = interpNodeZ[nY] - cz;

                    float baseNx = vxY * vyZ - vxZ * vyY;
                    float baseNy = vxZ * vyX - vxX * vyZ;
                    float baseNz = vxX * vyY - vxY * vyX;
                    float lenSq = baseNx * baseNx + baseNy * baseNy + baseNz * baseNz;

                    if (lenSq > 1e-10f) {
                        float invLen = fastInvSqrt(lenSq);
                        baseNx *= invLen; baseNy *= invLen; baseNz *= invLen;
                    }

                    float wX = flex.vWeightX[i], wY = flex.vWeightY[i], wZ = flex.vWeightZ[i];
                    finalX = cx + wX * vxX + wY * vyX + wZ * baseNx;
                    finalY = cy + wX * vxY + wY * vyY + wZ * baseNy;
                    finalZ = cz + wX * vxZ + wY * vyZ + wZ * baseNz;

                    if (flex.vNormWeightX != null) {
                        float nwX = flex.vNormWeightX[i], nwY = flex.vNormWeightY[i], nwZ = flex.vNormWeightZ[i];
                        float rawNx = vxX * nwX + vyX * nwY + baseNx * nwZ;
                        float rawNy = vxY * nwX + vyY * nwY + baseNy * nwZ;
                        float rawNz = vxZ * nwX + vyZ * nwY + baseNz * nwZ;

                        float nLenSq = rawNx * rawNx + rawNy * rawNy + rawNz * rawNz;
                        if (nLenSq > 1e-10f) {
                            float nInv = fastInvSqrt(nLenSq);
                            nx = rawNx * nInv; ny = rawNy * nInv; nz = rawNz * nInv;
                        } else { nx = baseNx; ny = baseNy; nz = baseNz; }
                    } else { nx = baseNx; ny = baseNy; nz = baseNz; }

                } else {
                    finalX = cx + flex.skinnedPosX[i];
                    finalY = cy + flex.skinnedPosY[i];
                    finalZ = cz + flex.skinnedPosZ[i];

                    nx = flex.vNormWeightX != null ? flex.vNormWeightX[i] : 0;
                    ny = flex.vNormWeightY != null ? flex.vNormWeightY[i] : 1;
                    nz = flex.vNormWeightZ != null ? flex.vNormWeightZ[i] : 0;
                }

                // 🌟 直接传递纯净的本地坐标，不需要 CPU 矩阵预乘！
                builder.vertex(finalX, finalY, finalZ)
                        .color(255, 255, 255, 255)
                        .texture(flex.uvU[i], flex.uvV[i])
                        .overlay(OverlayTexture.DEFAULT_UV)
                        .light(packedLight)
                        .normal(nx, ny, nz);

                globalVertPtr++;
            }
        }

        // 2. 结束录制，获取打包好的内存块 (BuiltBuffer 或 MeshData)
        var meshData = builder.end();
        if (meshData != null) {
            // 3. 上传到显卡 VBO
            this.vbo.bind();
            this.vbo.upload(meshData);
            VertexBuffer.unbind();
        }

        // 4. 🚀 终极绘制指令！
        // 官方 VBO 会自动处理 VAO、Projection Matrix 和 Shader 的绑定，绝对安全！
        this.vbo.bind();
        this.vbo.draw(matrixStack.peek().getPositionMatrix(), RenderSystem.getProjectionMatrix(), RenderSystem.getShader());
        VertexBuffer.unbind();
    }
}