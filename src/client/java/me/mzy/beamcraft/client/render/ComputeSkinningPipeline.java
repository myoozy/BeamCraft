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

    public static final double VERTEX_GROUP_SIZE = 256.0;

    public VertexBuffer mcVbo;
    public int rawVboId = -1;

    public int computeProgramId = -1;
    public int ssboRigging = -1;
    public int vboOutput = -1;
    public int ssboNodes = -1;

    public int totalVertices = 0;
    public int maxNodeCount = 0;

    public void init(FlexbodyContainer flex, int maxNodes) {
        this.totalVertices = flex.totalVertexCount;
        this.maxNodeCount = maxNodes;
        if (totalVertices == 0) return;

        // 1. 读取并编译 Shader
        try {
            InputStream is = getClass().getResourceAsStream("/assets/beamcraft/shaders/softbody.comp");
            if (is != null) {
                String source = new String(is.readAllBytes(), StandardCharsets.UTF_8);
                this.computeProgramId = ComputeShaderLoader.compileComputeShader(source);
            } else {
                System.err.println("Failed to load softbody.comp");
                return;
            }
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        // 2. 打包静态网格数据 (SSBO 0)
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
                rigBuffer.putFloat(flex.vWeightX[i]);
                rigBuffer.putFloat(flex.vWeightY[i]);
                rigBuffer.putFloat(flex.vWeightZ[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vCenterNode[i]));

                rigBuffer.putFloat(flex.vNormWeightX != null ? flex.vNormWeightX[i] : 0f);
                rigBuffer.putFloat(flex.vNormWeightY != null ? flex.vNormWeightY[i] : 1f);
                rigBuffer.putFloat(flex.vNormWeightZ != null ? flex.vNormWeightZ[i] : 0f);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? flex.vVxNode[i] : 0));

                rigBuffer.putFloat(flex.skinnedPosX[i]);
                rigBuffer.putFloat(flex.skinnedPosY[i]);
                rigBuffer.putFloat(flex.skinnedPosZ[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? flex.vVyNode[i] : 0));

                rigBuffer.putFloat(flex.uvU[i]);
                rigBuffer.putFloat(flex.uvV[i]);
                rigBuffer.putFloat(Float.intBitsToFloat(flex.vUseCrossZ[i] ? 1 : 0));
                rigBuffer.putFloat(0f);

                ptr++;
            }
        }
        rigBuffer.flip();

        this.ssboRigging = GL15.glGenBuffers();
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboRigging);
        GL15.glBufferData(GL43.GL_SHADER_STORAGE_BUFFER, rigBuffer, GL15.GL_STATIC_DRAW);
        MemoryUtil.memFree(rigBuffer);

        // 3. 生成官方 VBO 并获取底层 OpenGL ID
        this.mcVbo = new VertexBuffer(VertexBuffer.Usage.DYNAMIC);
        Tessellator tessellator = Tessellator.getInstance();
        BufferBuilder builder = tessellator.begin(VertexFormat.DrawMode.TRIANGLES, VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL);
        for (int i = 0; i < totalVertices; i++) {
            builder.vertex(0, 0, 0).color(255, 255, 255, 255).texture(0, 0).overlay(OverlayTexture.DEFAULT_UV).light(15728880).normal(0, 1, 0);
        }
        var meshData = builder.end();

        if (meshData != null) {
            this.mcVbo.bind();
            this.mcVbo.upload(meshData);
            // 捕获 VBO ID 供 Shader 写入
            this.rawVboId = GL11.glGetInteger(GL15.GL_ARRAY_BUFFER_BINDING);
            VertexBuffer.unbind();
        }

        // 4. 分配物理节点 SSBO 2
        this.ssboNodes = GL15.glGenBuffers();
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboNodes);
        GL15.glBufferData(GL43.GL_SHADER_STORAGE_BUFFER, (long) maxNodes * 16, GL15.GL_DYNAMIC_DRAW);
        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, 0);
    }

    public void dispatchCompute(float[] interpX, float[] interpY, float[] interpZ, int activeNodes, int packedLight) {
        if (this.computeProgramId == -1 || this.totalVertices == 0) return;

        ByteBuffer nodeBuffer = MemoryUtil.memAlloc(activeNodes * 16);
        for (int i = 0; i < activeNodes; i++) {
            nodeBuffer.putFloat(interpX[i]);
            nodeBuffer.putFloat(interpY[i]);
            nodeBuffer.putFloat(interpZ[i]);
            nodeBuffer.putFloat(0f);
        }
        nodeBuffer.flip();

        GL15.glBindBuffer(GL43.GL_SHADER_STORAGE_BUFFER, this.ssboNodes);
        GL15.glBufferSubData(GL43.GL_SHADER_STORAGE_BUFFER, 0, nodeBuffer);
        MemoryUtil.memFree(nodeBuffer);

        GL20.glUseProgram(this.computeProgramId);

        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 0, this.ssboRigging);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 1, this.rawVboId);
        GL30.glBindBufferBase(GL43.GL_SHADER_STORAGE_BUFFER, 2, this.ssboNodes);

        int loc = GL20.glGetUniformLocation(this.computeProgramId, "u_vertexCount");
        if (loc != -1) GL20.glUniform1i(loc, this.totalVertices);

        int lightLoc = GL20.glGetUniformLocation(this.computeProgramId, "u_packedLight");
        if (lightLoc != -1) GL20.glUniform1i(lightLoc, packedLight);

        int numGroups = (int) Math.ceil((double) this.totalVertices / VERTEX_GROUP_SIZE);
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