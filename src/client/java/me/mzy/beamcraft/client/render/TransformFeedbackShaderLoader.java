package me.mzy.beamcraft.client.render;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;

/**
 * Compiles the OpenGL 3.2 vertex program used as a GPU-only skinning kernel.
 * Rasterization is disabled while this program runs; its two outputs are
 * captured directly into the dynamic position/normal vertex buffer.
 */
public final class TransformFeedbackShaderLoader {

    private TransformFeedbackShaderLoader() {
    }

    public static int compile(String shaderSource) {
        int shaderId = GL20.glCreateShader(GL20.GL_VERTEX_SHADER);
        int programId = 0;

        try {
            GL20.glShaderSource(shaderId, shaderSource);
            GL20.glCompileShader(shaderId);

            if (GL20.glGetShaderi(shaderId, GL20.GL_COMPILE_STATUS) == GL11.GL_FALSE) {
                throw new IllegalStateException("GPU skinning shader compilation failed:\n"
                        + GL20.glGetShaderInfoLog(shaderId));
            }

            programId = GL20.glCreateProgram();
            GL20.glAttachShader(programId, shaderId);
            GL30.glTransformFeedbackVaryings(
                    programId,
                    new CharSequence[]{"tfPosition", "tfNormal"},
                    GL30.GL_INTERLEAVED_ATTRIBS
            );
            GL20.glLinkProgram(programId);

            if (GL20.glGetProgrami(programId, GL20.GL_LINK_STATUS) == GL11.GL_FALSE) {
                throw new IllegalStateException("GPU skinning program link failed:\n"
                        + GL20.glGetProgramInfoLog(programId));
            }

            return programId;
        } catch (RuntimeException exception) {
            if (programId != 0) {
                GL20.glDeleteProgram(programId);
            }
            throw exception;
        } finally {
            GL20.glDeleteShader(shaderId);
        }
    }
}
