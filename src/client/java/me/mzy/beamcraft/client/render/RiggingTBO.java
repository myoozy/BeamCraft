package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL31;
import org.lwjgl.system.MemoryUtil;

import java.nio.ByteBuffer;

public class RiggingTBO {
    private int tboId = -1;
    private int textureId = -1;

    public void init(FlexbodyContainer flex) {
        int vCount = flex.totalVertexCount;
        if (vCount == 0) return;

        // 每个顶点我们需要 3 个 vec4 (12 个 float = 48 字节)
        ByteBuffer buffer = MemoryUtil.memAlloc(vCount * 48);

        for (int i = 0; i < vCount; i++) {
            // Pixel 0: 权重 + cNode
            buffer.putFloat(flex.vUseCrossZ[i] ? flex.vWeightX[i] : 0f);
            buffer.putFloat(flex.vUseCrossZ[i] ? flex.vWeightY[i] : 0f);
            buffer.putFloat(flex.vUseCrossZ[i] ? flex.vWeightZ[i] : 0f);
            buffer.putFloat((float) flex.vCenterNode[i]);

            // Pixel 1: vxNode + vyNode + useCross + pad
            buffer.putFloat((float) flex.vVxNode[i]);
            buffer.putFloat((float) (flex.vUseCrossZ[i] ? flex.vVyNode[i] : 0f));
            buffer.putFloat(flex.vUseCrossZ[i] ? 1f : 0f);
            buffer.putFloat(0f); // 补齐

            // Pixel 2: skPosX + skPosY + skPosZ + pad (刚体偏移)
            buffer.putFloat(flex.skinnedPosX[i]);
            buffer.putFloat(flex.skinnedPosY[i]);
            buffer.putFloat(flex.skinnedPosZ[i]);
            buffer.putFloat(0f); // 补齐
        }
        buffer.flip();

        this.tboId = GL15.glGenBuffers();
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, this.tboId);
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, buffer, GL15.GL_STATIC_DRAW);

        this.textureId = GL11.glGenTextures();
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, this.textureId);
        GL31.glTexBuffer(GL31.GL_TEXTURE_BUFFER, GL30.GL_RGBA32F, this.tboId);

        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
        MemoryUtil.memFree(buffer);
    }

    public void bind(int textureUnit) {
        if (this.textureId == -1) return;
        RenderSystem.activeTexture(textureUnit);
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, this.textureId);
    }
}