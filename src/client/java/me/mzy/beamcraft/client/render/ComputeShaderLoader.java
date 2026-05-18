package me.mzy.beamcraft.client.render;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL43;

public class ComputeShaderLoader {

    public static int compileComputeShader(String shaderSource) {
        // 1. 向显卡申请一个计算着色器的空壳
        int computeShaderId = GL20.glCreateShader(GL43.GL_COMPUTE_SHADER);

        // 2. 把我们的 C 语言源码（GLSL）填进去
        GL20.glShaderSource(computeShaderId, shaderSource);

        // 3. 命令显卡驱动进行编译
        GL20.glCompileShader(computeShaderId);

        // 4. 严谨的错误检查：如果语法写错了，把显卡的报错日志打印出来
        if (GL20.glGetShaderi(computeShaderId, GL20.GL_COMPILE_STATUS) == GL11.GL_FALSE) {
            String log = GL20.glGetShaderInfoLog(computeShaderId);
            System.err.println("❌ Compute Shader 编译失败 (语法错误):\\n" + log);
            return -1;
        }

        // 5. 申请一个主程序 (Program)
        int programId = GL20.glCreateProgram();

        // 6. 把编译好的 Shader 挂载上去并链接
        GL20.glAttachShader(programId, computeShaderId);
        GL20.glLinkProgram(programId);

        if (GL20.glGetProgrami(programId, GL20.GL_LINK_STATUS) == GL11.GL_FALSE) {
            String log = GL20.glGetProgramInfoLog(programId);
            System.err.println("❌ Compute Shader 链接失败:\\n" + log);
            return -1;
        }

        // 7. 链接完后，中间产物就可以删除了，省点显存
        GL20.glDeleteShader(computeShaderId);

        System.out.println("✅ Compute Shader 注入显卡成功！程序 ID: " + programId);
        return programId;
    }
}