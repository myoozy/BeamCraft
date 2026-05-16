package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.render.*;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL43;
import org.lwjgl.system.MemoryUtil;

import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

public class ComputeSkinningPipeline {

    // 🌟 核心改变：持有 MC 官方 VBO，以及它底层的真实显存 ID！
    public VertexBuffer mcVbo;
    public int rawVboId = -1;

    public int computeProgramId = -1;
    public int ssboRigging = -1; // Binding 0: 静态基因数据
    public int vboOutput = -1;   // Binding 1: 动态 VBO 输出 (交给 MC 画图)
    public int ssboNodes = -1;   // Binding 2: 动态物理节点

    public int totalVertices = 0;
    public int maxNodeCount = 0;

    public void init(FlexbodyContainer flex, int maxNodes) {
        this.totalVertices = flex.totalVertexCount;
        this.maxNodeCount = maxNodes;
        if (totalVertices == 0) return;

        // 1. 读取并编译我们的 softbody.comp 着色器
        try {
            // 请确保路径和你的实际文件位置匹配！
            InputStream is = getClass().getResourceAsStream("/assets/beamcraft/shaders/softbody.comp");
            if (is != null) {
                String source = new String(is.readAllBytes(), StandardCharsets.UTF_8);
                this.computeProgramId = ComputeShaderLoader.compileComputeShader(source);
            } else {
                System.err.println("❌ 找不到 softbody.comp 文件！");
                return;
            }
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        // ====================================================================
        // 2. 打包静态基因库 (SSBO 0)，每个顶点严格 64 Bytes (16个 float)
        // ====================================================================
        ByteBuffer rigBuffer = MemoryUtil.memAlloc(totalVertices * 64);
        int ptr = 0;

        for (int m = 0; m < flex.meshCount; m++) {
            if (flex.meshName[m].isEmpty()) continue;
            String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[m];
            DaeMeshLoader.RawGeometry geom = DaeMeshLoader.MESH_CACHE.get(scopedKey);
            if (geom == null) geom = DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[m]);
            if (geom == null) continue;

            for (int v = 0; v < geom.vertexCount; v++) {
                int i = ptr;

                // 写入 vec4 w_and_cNode
                rigBuffer.putFloat(flex.vWeightX[i]);
                rigBuffer.putFloat(flex.vWeightY[i]);
                rigBuffer.putFloat(flex.vWeightZ[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vCenterNode[i]));

                // 写入 vec4 nw_and_vxNode
                rigBuffer.putFloat(flex.vNormWeightX != null ? flex.vNormWeightX[i] : 0f);
                rigBuffer.putFloat(flex.vNormWeightY != null ? flex.vNormWeightY[i] : 1f);
                rigBuffer.putFloat(flex.vNormWeightZ != null ? flex.vNormWeightZ[i] : 0f);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? flex.vVxNode[i] : 0));

                // 写入 vec4 skPos_and_vyNode
                rigBuffer.putFloat(flex.skinnedPosX[i]);
                rigBuffer.putFloat(flex.skinnedPosY[i]);
                rigBuffer.putFloat(flex.skinnedPosZ[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? flex.vVyNode[i] : 0));

                // 写入 vec4 uv_and_flags
                rigBuffer.putFloat(flex.uvU[i]);
                rigBuffer.putFloat(flex.uvV[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? 1 : 0));
                rigBuffer.putFloat(0f); // padding

                ptr++;
            }
        }
        rigBuffer.flip();

        // 创建 SSBO 0 并一次性灌入数据（以后再也不用管它了）
        this.ssboRigging = GL15.glGenBuffers();
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboRigging);
        GL15.glBufferData(GL43.GL_SHADER_STORAGE_BUFFER, rigBuffer, GL15.GL_STATIC_DRAW);
        MemoryUtil.memFree(rigBuffer);

        // ====================================================================
        // 3. 分配 VBO (SSBO 1) 和 Nodes (SSBO 2) 的空白显存
        // ====================================================================
        this.vboOutput = GL15.glGenBuffers();
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, this.vboOutput);
        GL15.glBufferData(GL15.GL_ARRAY_BUFFER, (long) totalVertices * 36, GL15.GL_DYNAMIC_DRAW);

        this.ssboNodes = GL15.glGenBuffers();
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboNodes);
        GL15.glBufferData(GL43.GL_SHADER_STORAGE_BUFFER, (long) maxNodeCount * 16, GL15.GL_DYNAMIC_DRAW);

        // 解绑
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, 0);
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
        // ====================================================================
        // 3. 瞒天过海：生成官方 VBO，并窃取它的底层 OpenGL ID
        // ====================================================================
        this.mcVbo = new VertexBuffer(VertexBuffer.Usage.DYNAMIC);

        // 造一个空壳模型，强制 Minecraft 为我们开辟足额的显存
        Tessellator tessellator = Tessellator.getInstance();
        BufferBuilder builder = tessellator.begin(VertexFormat.DrawMode.TRIANGLES, VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL);
        for (int i = 0; i < totalVertices; i++) {
            builder.vertex(0, 0, 0).color(255, 255, 255, 255).texture(0, 0).overlay(OverlayTexture.DEFAULT_UV).light(15728880).normal(0, 1, 0);
        }
        var meshData = builder.end();

        if (meshData != null) {
            this.mcVbo.bind();
            this.mcVbo.upload(meshData); // 显存分配完毕

            // 🚀 黑客行为：询问显卡当前绑定的 VBO ID 是多少，并据为己有！
            this.rawVboId = GL11.glGetInteger(GL15.GL_ARRAY_BUFFER_BINDING);
            VertexBuffer.unbind();
        }

        // 分配物理节点仓库 SSBO 2
        this.ssboNodes = GL15.glGenBuffers();
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboNodes);
        GL15.glBufferData(GL43.GL_SHADER_STORAGE_BUFFER, (long) maxNodes * 16, GL15.GL_DYNAMIC_DRAW);
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, 0);
    }

    // ====================================================================
    // 🚀 每帧调用的极速发令枪
    // ====================================================================
    public void dispatchCompute(float[] interpX, float[] interpY, float[] interpZ, int activeNodes) {if (this.computeProgramId == -1 || this.totalVertices == 0) return;

        // 1. 打包最新物理节点 (按 vec4 对齐: X, Y, Z, 0)
        ByteBuffer nodeBuffer = MemoryUtil.memAlloc(activeNodes * 16);
        for (int i = 0; i < activeNodes; i++) {
            nodeBuffer.putFloat(interpX[i]);
            nodeBuffer.putFloat(interpY[i]);
            nodeBuffer.putFloat(interpZ[i]);
            nodeBuffer.putFloat(0f); // 凑齐 16 字节
        }
        nodeBuffer.flip();

        // 2. 极速传给 SSBO 2 仓库
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboNodes);
        GL15.glBufferSubData(GL43.GL_SHADER_STORAGE_BUFFER, 0, nodeBuffer);
        MemoryUtil.memFree(nodeBuffer);

        // 3. 激活我们的 Compute Shader
        GL20.glUseProgram(this.computeProgramId);

        // 🌟 将窃取到的 rawVboId 作为 1 号输出仓库绑定给计算着色器！
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 0, this.ssboRigging);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 1, this.rawVboId);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 2, this.ssboNodes);

        int loc = GL20.glGetUniformLocation(this.computeProgramId, "u_vertexCount");
        if (loc != -1) GL20.glUniform1i(loc, this.totalVertices);

        int numGroups = (int) Math.ceil((double) this.totalVertices / 256.0);
        GL43.glDispatchCompute(numGroups, 1, 1);

        GL43.glMemoryBarrier(GL43.GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

        GL20.glUseProgram(0);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 0, 0);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 1, 0);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 2, 0);
    }

    public void free() {
        if (this.computeProgramId != -1) { GL20.glDeleteProgram(this.computeProgramId); this.computeProgramId = -1; }
        if (this.ssboRigging != -1) { GL15.glDeleteBuffers(this.ssboRigging); this.ssboRigging = -1; }
        if (this.vboOutput != -1) { GL15.glDeleteBuffers(this.vboOutput); this.vboOutput = -1; }
        if (this.ssboNodes != -1) { GL15.glDeleteBuffers(this.ssboNodes); this.ssboNodes = -1; }
        if (this.mcVbo != null) { this.mcVbo.close(); this.mcVbo = null; }
    }
}