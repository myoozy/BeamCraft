package me.mzy.beamcraft.client.render;

import com.mojang.blaze3d.systems.RenderSystem;
import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import net.minecraft.client.gl.ShaderProgram;
import net.minecraft.client.gl.VertexBuffer;
import net.minecraft.client.util.BufferAllocator;
import net.minecraft.client.render.BufferBuilder;
import net.minecraft.client.render.OverlayTexture;
import net.minecraft.client.render.Tessellator;
import net.minecraft.client.render.VertexFormat;
import net.minecraft.client.render.VertexFormatElement;
import net.minecraft.client.render.VertexFormats;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import org.lwjgl.opengl.GL31;
import org.lwjgl.system.MemoryUtil;
import org.joml.Matrix4f;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

/**
 * GPU-only soft-body skinning for Minecraft's OpenGL renderer.
 *
 * <p>The class name is retained for source compatibility, but the implementation
 * deliberately does not use an OpenGL 4.3 compute shader. An OpenGL 3.2 vertex
 * shader performs the same calculation under rasterizer discard and Transform
 * Feedback writes the result into the position/normal VBO.</p>
 */
public class ComputeSkinningPipeline {

    private enum State {
        NEW,
        READY,
        FAILED,
        CLOSED
    }

    private static final int RIG_STREAM_COUNT = 3;
    private static final int TEXTURE_UNIT_WEIGHTS = 0;
    private static final int TEXTURE_UNIT_NORMALS = 1;
    private static final int TEXTURE_UNIT_OFFSETS = 2;
    private static final int TEXTURE_UNIT_NODES = 3;

    private VertexBuffer mcVbo;
    private int customPosNormVbo = -1;

    private final int[] rigBuffers = {-1, -1, -1};
    private final int[] rigTextures = {-1, -1, -1};

    private int nodeBuffer = -1;
    private int nodeTexture = -1;
    private int transformProgramId = -1;
    private int transformVao = -1;

    private int totalVertices;
    private int maxNodeCount;
    private int lightAttributeIndex = -1;
    private ByteBuffer nodeUploadBuffer;

    private State state = State.NEW;
    private boolean hasValidOutput;
    private long lastRenderMoment = Long.MIN_VALUE;

    public boolean init(FlexbodyContainer flex, int requestedNodeCapacity) {
        RenderSystem.assertOnRenderThread();
        if (state != State.NEW) {
            return state == State.READY;
        }

        totalVertices = flex.totalVertexCount;
        maxNodeCount = Math.max(1, requestedNodeCapacity);
        if (totalVertices == 0) {
            state = State.FAILED;
            return false;
        }

        try {
            validateCapabilities();
            transformProgramId = TransformFeedbackShaderLoader.compile(loadShaderSource());
            configureSamplerUnits();
            createRigStreams(flex);
            createNodeStream();
            createMinecraftVertexBuffer(flex);
            createOutputBufferAndConfigureVertexArray();
            transformVao = GL30.glGenVertexArrays();
            state = State.READY;
            BeamCraft.LOGGER.info(
                    "Initialized OpenGL 3.2 GPU skinning for {} vertices on {} / {} ({})",
                    totalVertices,
                    GL11.glGetString(GL11.GL_VENDOR),
                    GL11.glGetString(GL11.GL_RENDERER),
                    GL11.glGetString(GL11.GL_VERSION)
            );
            return true;
        } catch (RuntimeException | IOException exception) {
            BeamCraft.LOGGER.error("Failed to initialize GPU-only soft-body skinning", exception);
            releaseGlResources();
            state = State.FAILED;
            return false;
        }
    }

    public boolean isReady() {
        return state == State.READY;
    }

    public boolean hasValidOutput() {
        return state == State.READY && hasValidOutput;
    }

    private void validateCapabilities() {
        if (!GL.getCapabilities().OpenGL32) {
            throw new IllegalStateException("BeamCraft GPU skinning requires OpenGL 3.2, "
                    + "which is also Minecraft 1.21's minimum graphics API");
        }

        int maxTextureBufferTexels = GL11.glGetInteger(GL31.GL_MAX_TEXTURE_BUFFER_SIZE);
        int requiredTexels = Math.max(totalVertices, maxNodeCount);
        if (requiredTexels > maxTextureBufferTexels) {
            throw new IllegalStateException("Vehicle requires " + requiredTexels
                    + " texture-buffer texels, but this GPU supports " + maxTextureBufferTexels);
        }

        int vertexTextureUnits = GL11.glGetInteger(GL20.GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS);
        if (vertexTextureUnits < 4) {
            throw new IllegalStateException("GPU skinning requires four vertex texture units, but this GPU supports "
                    + vertexTextureUnits);
        }
    }

    private String loadShaderSource() throws IOException {
        String path = "/assets/beamcraft/shaders/softbody_transform.vsh";
        try (InputStream stream = getClass().getResourceAsStream(path)) {
            if (stream == null) {
                throw new IOException("Missing shader resource " + path);
            }
            return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
        }
    }

    private void configureSamplerUnits() {
        int previousProgram = GL11.glGetInteger(GL20.GL_CURRENT_PROGRAM);
        try {
            GL20.glUseProgram(transformProgramId);
            setSampler("uRigWeights", TEXTURE_UNIT_WEIGHTS);
            setSampler("uRigNormals", TEXTURE_UNIT_NORMALS);
            setSampler("uRigOffsets", TEXTURE_UNIT_OFFSETS);
            setSampler("uPhysicsNodes", TEXTURE_UNIT_NODES);
        } finally {
            GL20.glUseProgram(previousProgram);
        }
    }

    private void setSampler(String name, int textureUnit) {
        int location = GL20.glGetUniformLocation(transformProgramId, name);
        if (location < 0) {
            throw new IllegalStateException("Missing GPU skinning sampler " + name);
        }
        GL20.glUniform1i(location, textureUnit);
    }

    private void createRigStreams(FlexbodyContainer flex) {
        ByteBuffer weights = MemoryUtil.memAlloc(totalVertices * 16);
        ByteBuffer normals = MemoryUtil.memAlloc(totalVertices * 16);
        ByteBuffer offsets = MemoryUtil.memAlloc(totalVertices * 16);

        try {
            for (int i = 0; i < totalVertices; i++) {
                weights.putFloat(flex.vWeightX[i]);
                weights.putFloat(flex.vWeightY[i]);
                weights.putFloat(flex.vWeightZ[i]);
                weights.putFloat(flex.vCenterNode[i]);

                normals.putFloat(flex.vNormWeightX[i]);
                normals.putFloat(flex.vNormWeightY[i]);
                normals.putFloat(flex.vNormWeightZ[i]);
                normals.putFloat(flex.vUseCrossZ[i] ? flex.vVxNode[i] : -1.0f);

                offsets.putFloat(flex.skinnedPosX[i]);
                offsets.putFloat(flex.skinnedPosY[i]);
                offsets.putFloat(flex.skinnedPosZ[i]);
                offsets.putFloat(flex.vUseCrossZ[i] ? flex.vVyNode[i] : -1.0f);
            }

            weights.flip();
            normals.flip();
            offsets.flip();
            createTextureBufferStream(0, weights, GL15.GL_STATIC_DRAW);
            createTextureBufferStream(1, normals, GL15.GL_STATIC_DRAW);
            createTextureBufferStream(2, offsets, GL15.GL_STATIC_DRAW);
        } finally {
            MemoryUtil.memFree(weights);
            MemoryUtil.memFree(normals);
            MemoryUtil.memFree(offsets);
        }
    }

    private void createTextureBufferStream(int index, ByteBuffer data, int usage) {
        rigBuffers[index] = GL15.glGenBuffers();
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, rigBuffers[index]);
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, data, usage);

        rigTextures[index] = GL11.glGenTextures();
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, rigTextures[index]);
        GL31.glTexBuffer(GL31.GL_TEXTURE_BUFFER, GL30.GL_RGBA32F, rigBuffers[index]);

        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
    }

    private void createNodeStream() {
        nodeUploadBuffer = MemoryUtil.memAlloc(maxNodeCount * 16);

        nodeBuffer = GL15.glGenBuffers();
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, nodeBuffer);
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, (long) maxNodeCount * 16, GL15.GL_STREAM_DRAW);

        nodeTexture = GL11.glGenTextures();
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, nodeTexture);
        GL31.glTexBuffer(GL31.GL_TEXTURE_BUFFER, GL30.GL_RGBA32F, nodeBuffer);

        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, 0);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
    }

    private void createMinecraftVertexBuffer(FlexbodyContainer flex) {
        int[] indices = buildCombinedIndices(flex);
        if (indices.length < totalVertices) {
            throw new IllegalStateException("Indexed vehicle mesh contains unused render vertices");
        }

        mcVbo = new VertexBuffer(VertexBuffer.Usage.STATIC);
        Tessellator tessellator = Tessellator.getInstance();
        BufferBuilder builder = tessellator.begin(
                VertexFormat.DrawMode.TRIANGLES,
                VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL
        );

        for (int i = 0; i < totalVertices; i++) {
            builder.vertex(0, 0, 0)
                    .color(255, 255, 255, 255)
                    .texture(flex.uvU[i], flex.uvV[i])
                    .overlay(OverlayTexture.DEFAULT_UV)
                    .light(0)
                    .normal(0, 1, 0);
        }
        // BufferBuilder controls VertexBuffer's private draw parameters. Pad
        // with unreferenced vertices so its index count equals our real EBO
        // count; only the compact vertices above are addressed by that EBO.
        for (int i = totalVertices; i < indices.length; i++) {
            builder.vertex(0, 0, 0)
                    .color(255, 255, 255, 255)
                    .texture(0, 0)
                    .overlay(OverlayTexture.DEFAULT_UV)
                    .light(0)
                    .normal(0, 1, 0);
        }

        var meshData = builder.end();
        if (meshData == null) {
            throw new IllegalStateException("Minecraft produced no mesh data for a non-empty vehicle");
        }

        mcVbo.bind();
        mcVbo.upload(meshData);
        uploadIndexBuffer(indices, VertexFormat.IndexType.smallestFor(indices.length));
        VertexBuffer.unbind();
    }

    private int[] buildCombinedIndices(FlexbodyContainer flex) {
        int totalIndexCount = 0;
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            DaeMeshLoader.RawGeometry geometry = findGeometry(flex, mesh);
            if (geometry != null) {
                totalIndexCount += geometry.indexCount;
            }
        }

        int[] combined = new int[totalIndexCount];
        int indexOffset = 0;
        int vertexOffset = 0;
        for (int mesh = 0; mesh < flex.meshCount; mesh++) {
            DaeMeshLoader.RawGeometry geometry = findGeometry(flex, mesh);
            if (geometry == null) {
                continue;
            }
            for (int index = 0; index < geometry.indexCount; index++) {
                combined[indexOffset++] = vertexOffset + geometry.indices[index];
            }
            vertexOffset += geometry.vertexCount;
        }

        if (vertexOffset != totalVertices || indexOffset != combined.length) {
            throw new IllegalStateException("DAE geometry changed while the GPU skinning pipeline was initialized");
        }
        return combined;
    }

    private DaeMeshLoader.RawGeometry findGeometry(FlexbodyContainer flex, int mesh) {
        if (flex.meshName[mesh].isEmpty()) {
            return null;
        }
        String scopedKey = flex.vehicleNamespace + ":" + flex.meshName[mesh];
        DaeMeshLoader.RawGeometry geometry = DaeMeshLoader.MESH_CACHE.get(scopedKey);
        return geometry != null
                ? geometry
                : DaeMeshLoader.MESH_CACHE.get("common:" + flex.meshName[mesh]);
    }

    private void uploadIndexBuffer(int[] indices, VertexFormat.IndexType indexType) {
        int bytesPerIndex = indexType == VertexFormat.IndexType.SHORT
                ? Short.BYTES
                : Integer.BYTES;
        int byteCount = indices.length * bytesPerIndex;

        try (BufferAllocator allocator = new BufferAllocator(byteCount)) {
            long address = allocator.allocate(byteCount);
            ByteBuffer indexData = MemoryUtil.memByteBuffer(address, byteCount);
            if (indexType == VertexFormat.IndexType.SHORT) {
                for (int index : indices) {
                    indexData.putShort((short) index);
                }
            } else {
                for (int index : indices) {
                    indexData.putInt(index);
                }
            }

            BufferAllocator.CloseableBuffer allocated = allocator.getAllocated();
            if (allocated == null) {
                throw new IllegalStateException("Failed to allocate the vehicle index buffer");
            }
            mcVbo.uploadIndexBuffer(allocated);
        }
    }

    private void createOutputBufferAndConfigureVertexArray() {
        customPosNormVbo = GL15.glGenBuffers();
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, customPosNormVbo);
        GL15.glBufferData(GL15.GL_ARRAY_BUFFER, (long) totalVertices * 6 * Float.BYTES, GL15.GL_STREAM_DRAW);
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);

        VertexFormat format = VertexFormats.POSITION_COLOR_TEXTURE_OVERLAY_LIGHT_NORMAL;
        int positionIndex = requireAttributeIndex(format, VertexFormatElement.POSITION);
        int normalIndex = requireAttributeIndex(format, VertexFormatElement.NORMAL);
        lightAttributeIndex = requireAttributeIndex(format, VertexFormatElement.UV_2);

        // This VAO belongs exclusively to mcVbo, so the dynamic bindings can be
        // installed once rather than mutating and restoring them on every draw.
        mcVbo.bind();
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, customPosNormVbo);
        GL20.glEnableVertexAttribArray(positionIndex);
        GL20.glVertexAttribPointer(positionIndex, 3, GL11.GL_FLOAT, false, 24, 0L);
        GL20.glEnableVertexAttribArray(normalIndex);
        GL20.glVertexAttribPointer(normalIndex, 3, GL11.GL_FLOAT, false, 24, 12L);
        GL20.glDisableVertexAttribArray(lightAttributeIndex);
        GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
        VertexBuffer.unbind();
    }

    private int requireAttributeIndex(VertexFormat format, VertexFormatElement element) {
        int index = format.getElements().indexOf(element);
        if (index < 0) {
            throw new IllegalStateException("Minecraft vertex format is missing required element " + element);
        }
        return index;
    }

    public boolean updateGpuSkinning(
            float[] interpX,
            float[] interpY,
            float[] interpZ,
            int activeNodes,
            long renderMoment
    ) {
        RenderSystem.assertOnRenderThread();
        if (state != State.READY || activeNodes <= 0) {
            return false;
        }
        if (hasValidOutput && lastRenderMoment == renderMoment) {
            return true;
        }

        try {
            ensureNodeCapacity(activeNodes);
            uploadNodes(interpX, interpY, interpZ, activeNodes);
            runTransformFeedback();
            hasValidOutput = true;
            lastRenderMoment = renderMoment;
            return true;
        } catch (RuntimeException exception) {
            BeamCraft.LOGGER.error("GPU soft-body skinning failed; this vehicle will no longer be rendered", exception);
            hasValidOutput = false;
            state = State.FAILED;
            return false;
        }
    }

    private void ensureNodeCapacity(int activeNodes) {
        if (activeNodes <= maxNodeCount) {
            return;
        }

        int newCapacity = Math.max(activeNodes, maxNodeCount * 2);
        int maxTextureBufferTexels = GL11.glGetInteger(GL31.GL_MAX_TEXTURE_BUFFER_SIZE);
        if (newCapacity > maxTextureBufferTexels) {
            throw new IllegalStateException("Vehicle node count exceeds GPU texture-buffer capacity");
        }

        MemoryUtil.memFree(nodeUploadBuffer);
        nodeUploadBuffer = MemoryUtil.memAlloc(newCapacity * 16);
        maxNodeCount = newCapacity;

        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, nodeBuffer);
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, (long) maxNodeCount * 16, GL15.GL_STREAM_DRAW);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
    }

    private void uploadNodes(float[] interpX, float[] interpY, float[] interpZ, int activeNodes) {
        nodeUploadBuffer.clear();
        for (int i = 0; i < activeNodes; i++) {
            nodeUploadBuffer.putFloat(interpX[i]);
            nodeUploadBuffer.putFloat(interpY[i]);
            nodeUploadBuffer.putFloat(interpZ[i]);
            nodeUploadBuffer.putFloat(0.0f);
        }
        nodeUploadBuffer.flip();

        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, nodeBuffer);
        GL15.glBufferData(GL31.GL_TEXTURE_BUFFER, (long) maxNodeCount * 16, GL15.GL_STREAM_DRAW);
        GL15.glBufferSubData(GL31.GL_TEXTURE_BUFFER, 0, nodeUploadBuffer);
        GL15.glBindBuffer(GL31.GL_TEXTURE_BUFFER, 0);
    }

    private void runTransformFeedback() {
        int previousProgram = GL11.glGetInteger(GL20.GL_CURRENT_PROGRAM);
        int previousVao = GL11.glGetInteger(GL30.GL_VERTEX_ARRAY_BINDING);
        int previousActiveTexture = GL11.glGetInteger(GL13.GL_ACTIVE_TEXTURE);
        int previousTransformFeedbackBuffer = GL30.glGetIntegeri(
                GL30.GL_TRANSFORM_FEEDBACK_BUFFER_BINDING, 0
        );
        boolean rasterizerDiscardWasEnabled = GL11.glIsEnabled(GL30.GL_RASTERIZER_DISCARD);
        boolean transformFeedbackActive = false;
        int[] previousTextureBindings = new int[4];

        try {
            bindTextureBuffer(TEXTURE_UNIT_WEIGHTS, rigTextures[0], previousTextureBindings);
            bindTextureBuffer(TEXTURE_UNIT_NORMALS, rigTextures[1], previousTextureBindings);
            bindTextureBuffer(TEXTURE_UNIT_OFFSETS, rigTextures[2], previousTextureBindings);
            bindTextureBuffer(TEXTURE_UNIT_NODES, nodeTexture, previousTextureBindings);

            GL20.glUseProgram(transformProgramId);
            GL30.glBindVertexArray(transformVao);
            GL30.glBindBufferBase(GL30.GL_TRANSFORM_FEEDBACK_BUFFER, 0, customPosNormVbo);
            GL11.glEnable(GL30.GL_RASTERIZER_DISCARD);
            GL30.glBeginTransformFeedback(GL11.GL_POINTS);
            transformFeedbackActive = true;
            GL11.glDrawArrays(GL11.GL_POINTS, 0, totalVertices);
            GL30.glEndTransformFeedback();
            transformFeedbackActive = false;
        } finally {
            if (transformFeedbackActive) {
                GL30.glEndTransformFeedback();
            }
            if (!rasterizerDiscardWasEnabled) {
                GL11.glDisable(GL30.GL_RASTERIZER_DISCARD);
            }
            GL30.glBindBufferBase(
                    GL30.GL_TRANSFORM_FEEDBACK_BUFFER,
                    0,
                    previousTransformFeedbackBuffer
            );
            GL30.glBindVertexArray(previousVao);
            GL20.glUseProgram(previousProgram);

            for (int unit = 0; unit < previousTextureBindings.length; unit++) {
                GL13.glActiveTexture(GL13.GL_TEXTURE0 + unit);
                GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, previousTextureBindings[unit]);
            }
            GL13.glActiveTexture(previousActiveTexture);
        }
    }

    /**
     * Draws the latest skinned output. Keeping the raw OpenGL vertex attribute
     * operation here gives a future non-OpenGL backend a single replacement
     * boundary instead of leaking GL details into the entity renderer.
     */
    public void draw(Matrix4f modelView, Matrix4f projection, ShaderProgram shader, int packedLight) {
        RenderSystem.assertOnRenderThread();
        if (!hasValidOutput()) {
            return;
        }

        mcVbo.bind();
        try {
            int blockLight = packedLight & 0xFFFF;
            int skyLight = packedLight >>> 16 & 0xFFFF;
            GL30.glVertexAttribI2i(lightAttributeIndex, blockLight, skyLight);
            mcVbo.draw(modelView, projection, shader);
        } finally {
            VertexBuffer.unbind();
        }
    }

    private void bindTextureBuffer(int unit, int texture, int[] previousBindings) {
        GL13.glActiveTexture(GL13.GL_TEXTURE0 + unit);
        previousBindings[unit] = GL11.glGetInteger(GL31.GL_TEXTURE_BINDING_BUFFER);
        GL11.glBindTexture(GL31.GL_TEXTURE_BUFFER, texture);
    }

    public void free() {
        if (state == State.CLOSED) {
            return;
        }
        state = State.CLOSED;
        hasValidOutput = false;
        if (RenderSystem.isOnRenderThread()) {
            releaseGlResources();
        } else {
            RenderSystem.recordRenderCall(this::releaseGlResources);
        }
    }

    private void releaseGlResources() {
        if (transformProgramId != -1) {
            GL20.glDeleteProgram(transformProgramId);
            transformProgramId = -1;
        }
        if (transformVao != -1) {
            GL30.glDeleteVertexArrays(transformVao);
            transformVao = -1;
        }

        for (int i = 0; i < RIG_STREAM_COUNT; i++) {
            if (rigTextures[i] != -1) {
                GL11.glDeleteTextures(rigTextures[i]);
                rigTextures[i] = -1;
            }
            if (rigBuffers[i] != -1) {
                GL15.glDeleteBuffers(rigBuffers[i]);
                rigBuffers[i] = -1;
            }
        }

        if (nodeTexture != -1) {
            GL11.glDeleteTextures(nodeTexture);
            nodeTexture = -1;
        }
        if (nodeBuffer != -1) {
            GL15.glDeleteBuffers(nodeBuffer);
            nodeBuffer = -1;
        }
        if (customPosNormVbo != -1) {
            GL15.glDeleteBuffers(customPosNormVbo);
            customPosNormVbo = -1;
        }
        if (mcVbo != null) {
            mcVbo.close();
            mcVbo = null;
        }
        if (nodeUploadBuffer != null) {
            MemoryUtil.memFree(nodeUploadBuffer);
            nodeUploadBuffer = null;
        }
    }
}
