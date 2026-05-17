package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL31;
import org.lwjgl.system.MemoryUtil;

import java.nio.ByteBuffer;

public class DynamicNodeTBO {
    private int tboId = -1;
    private int textureId = -1;
    private int maxNodes = 0;

    /**
     * 初始化 TBO 缓冲区（在车辆初始化时调用一次）
     */
    public void init(int maxNodes) {
        this.maxNodes = maxNodes;

        // 1. 生成底层的显存 Buffer
        this.tboId = GL15.glGenBuffers();
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, this.tboId);
        // 分配空间。每个节点使用 vec4 对齐 (x, y, z, pad) = 16 字节
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, (long) maxNodes * 16, GL15.GL_DYNAMIC_DRAW);

        // 2. 生成 Texture ID，并将其与上面的 Buffer 绑定
        this.textureId = GL11.glGenTextures();
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, this.textureId);
        // 告诉显卡：这张纹理的数据来源于 tboId，且格式为 RGBA32F (4个32位浮点数)
        GL31.glTexBuffer(GL31.GL_TEXTURE_BUFFER, GL30.GL_RGBA32F, this.tboId);

        // 清理绑定状态
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
    }

    /**
     * 每帧更新物理节点坐标（在渲染循环中调用）
     */
    public void update(float[] interpX, float[] interpY, float[] interpZ, int activeNodes) {
        if (activeNodes == 0 || this.tboId == -1) return;

        // 在堆外内存打包数据，避免 Java GC 停顿
        ByteBuffer buffer = MemoryUtil.memAlloc(activeNodes * 16);
        for (int i = 0; i < activeNodes; i++) {
            buffer.putFloat(interpX[i]);
            buffer.putFloat(interpY[i]);
            buffer.putFloat(interpZ[i]);
            buffer.putFloat(0f); // 补齐第四个通道 (Alpha)，满足 vec4 对齐
        }
        buffer.flip();

        // 极速覆盖 Buffer 数据（耗时在 0.1ms 以下）
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, this.tboId);
        GL15.glBufferSubData(GL31.GL_TEXTURE_BUFFER, 0, buffer);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);

        MemoryUtil.memFree(buffer);
    }

    /**
     * 绑定纹理，供 Shader 读取
     * @param textureUnit 例如 GL13.GL_TEXTURE2，表示绑定到几号采样器
     */
    public void bind(int textureUnit) {
        if (this.textureId == -1) return;
        RenderSystem.activeTexture(textureUnit);
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, this.textureId);
    }

    public void free() {
        if (this.textureId != -1) {
            GL11.glDeleteTextures(this.textureId);
            this.textureId = -1;
        }
        if (this.tboId != -1) {
            GL15.glDeleteBuffers(this.tboId);
            this.tboId = -1;
        }
    }
}