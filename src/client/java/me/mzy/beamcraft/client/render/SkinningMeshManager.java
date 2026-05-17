package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.*;

public class SkinningMeshManager {

    public VertexBuffer staticSkinningVbo;

    public void buildStaticVbo(FlexbodyContainer flex) {
        int totalVertices = flex.totalVertexCount;
        if (totalVertices == 0) return;

        if (this.staticSkinningVbo != null) {
            this.staticSkinningVbo.close();
        }

        this.staticSkinningVbo = new VertexBuffer(VertexBuffer.Usage.STATIC);

        Tessellator tessellator = Tessellator.getInstance();
        BufferBuilder builder = tessellator.begin(VertexFormat.DrawMode.TRIANGLES, VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL);

        for (int i = 0; i < totalVertices; i++) {
            // 核心魔法：把真实的顶点编号 i，存在绝对安全的 Position.x 里！
            builder.vertex((float) i, 0f, 0f)
                    .color(255, 255, 255, 255)
                    .texture(flex.uvU[i], flex.uvV[i])
                    .overlay(OverlayTexture.DEFAULT_UV)
                    .light(15728880)
                    .normal(0, 1, 0);
        }

        var meshData = builder.end();
        if (meshData != null) {
            this.staticSkinningVbo.bind();
            this.staticSkinningVbo.upload(meshData);
            VertexBuffer.unbind();
        }
    }

    public void free() {
        if (this.staticSkinningVbo != null) {
            this.staticSkinningVbo.close();
            this.staticSkinningVbo = null;
        }
    }
}