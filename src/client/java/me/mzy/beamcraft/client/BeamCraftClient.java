package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.physics.PhysicsWorld;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.fabricmc.fabric.api.client.rendering.v1.HudRenderCallback;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.RenderLayer;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.InputUtil;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.Vec3d;
import net.minecraft.util.shape.VoxelShape;
import org.lwjgl.glfw.GLFW;
import java.util.List;

public class BeamCraftClient implements ClientModInitializer {
	// 【关键】：把它放在类的最外层，用来记录上一帧 G 键有没有被按下
	private static boolean gWasPressed = false;
	public static final double DELTA_TIME = 0.05;

	// 记录上一帧的物理耗时 (毫秒)
	public static double lastPhysicsMs = 0.0;
	// 记录扫描方块的耗时
	public static double lastScanMs = 0.0;

	@Override
	public void onInitializeClient() {

		// 1. 物理计算与控制循环 (每帧运行)
		ClientTickEvents.END_CLIENT_TICK.register(client -> {
			if (client.player == null || client.world == null) return;
			PhysicsWorld world = BeamCraft.PHYSICS_WORLD;

			// 检测 G 键
			boolean isG = InputUtil.isKeyPressed(client.getWindow().getHandle(), GLFW.GLFW_KEY_G);

			// 只有按下瞬间才召唤！(防连发，防悬停)
			if (isG && !gWasPressed) {
				double HEIGHT_OFFSET = 5;
				world.spawnAt(
					client.player.getX(),
					client.player.getY() + HEIGHT_OFFSET,
					client.player.getZ()
				);
			}
			gWasPressed = isG; // 更新状态

			if (world.nodes.count > 0) {
				double DELTA_TIME = 0.05;

				long t1 = System.nanoTime();
				world.preStep(client.world, DELTA_TIME);
				long t2 = System.nanoTime();
				world.step(DELTA_TIME);
				long t3 = System.nanoTime();

				// 计算并转换为毫秒 (1毫秒 = 1,000,000纳秒)
				lastScanMs = (t2 - t1) / 1_000_000.0;
				lastPhysicsMs = (t3 - t2) / 1_000_000.0;
			}
		});

		HudRenderCallback.EVENT.register((drawContext, tickCounter) -> {
			MinecraftClient client = MinecraftClient.getInstance();
			if (client.options.hudHidden) return; // 如果按了 F1 隐藏界面，就不画

			// 拼凑你想显示的字符串
			String physicsStepText = String.format("BeamCraft Physics: %.2f ms", lastPhysicsMs);

			String physicsScanText = String.format("BeamCraft Physics Scan: %.2f ms", lastScanMs);

			// 字体颜色：如果耗时超过 10ms 就变红警告，否则是亮绿色
			int color = (lastPhysicsMs + lastScanMs > 10.0) ? 0xFF0000 : 0x00FF00;

			// 在屏幕左上角 (X=10, Y=10) 画出来，带阴影
			drawContext.drawTextWithShadow(client.textRenderer, physicsStepText, 10, 10, color);
			drawContext.drawTextWithShadow(client.textRenderer, physicsScanText, 10, 20, color);
		});

		// 2. 渲染循环 (你之前的画线代码，不用变)
		WorldRenderEvents.AFTER_ENTITIES.register(context -> {
			PhysicsWorld world = BeamCraft.PHYSICS_WORLD;
			if (world == null) return;

			Vec3d cameraPos = context.camera().getPos();
			MatrixStack stack = context.matrixStack();
			stack.push();
			stack.translate(-cameraPos.x, -cameraPos.y, -cameraPos.z);
			org.joml.Matrix4f matrix = stack.peek().getPositionMatrix();

			// === 1. 渲染梁/骨架 (使用 beamBuffer) ===
			VertexConsumer beamBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			for (int i = 0; i < world.beams.count; i++) {
				int n1 = world.beams.node1[i];
				int n2 = world.beams.node2[i];

				float x1 = (float)world.nodes.posX[n1]; float y1 = (float)world.nodes.posY[n1]; float z1 = (float)world.nodes.posZ[n1];
				float x2 = (float)world.nodes.posX[n2]; float y2 = (float)world.nodes.posY[n2]; float z2 = (float)world.nodes.posZ[n2];

				if (world.beams.broken[i])
				{
					//beamBuffer.vertex(matrix, x1, y1, z1).color(255, 0, 0, 255).normal(0, 1, 0);
					//beamBuffer.vertex(matrix, x2, y2, z2).color(255, 0, 0, 255).normal(0, 1, 0);
				}
				else
				{
					// 用亮绿色画骨架 (注意全部是 beamBuffer)
					//beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
					//beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
				}
			}

			// === 2. 渲染三角面 (使用 triBuffer) ===
			// 这里我们用 RenderLayer.getDebugQuads() 或者其他面渲染层，暂时用线条勾勒面
			VertexConsumer triBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			for (int i = 0; i < world.triangles.count; i++) {
				int n1 = world.triangles.node1[i];
				int n2 = world.triangles.node2[i];
				int n3 = world.triangles.node3[i];

				float x1 = (float)world.nodes.posX[n1]; float y1 = (float)world.nodes.posY[n1]; float z1 = (float)world.nodes.posZ[n1];
				float x2 = (float)world.nodes.posX[n2]; float y2 = (float)world.nodes.posY[n2]; float z2 = (float)world.nodes.posZ[n2];
				float x3 = (float)world.nodes.posX[n3]; float y3 = (float)world.nodes.posY[n3]; float z3 = (float)world.nodes.posZ[n3];

				// 用浅蓝色画三角面的轮廓 (注意全部是 triBuffer)
				triBuffer.vertex(matrix, x1, y1, z1).color(100, 150, 255, 255).normal(0, 1, 0);
				triBuffer.vertex(matrix, x2, y2, z2).color(100, 150, 255, 255).normal(0, 1, 0);

				triBuffer.vertex(matrix, x2, y2, z2).color(100, 150, 255, 255).normal(0, 1, 0);
				triBuffer.vertex(matrix, x3, y3, z3).color(100, 150, 255, 255).normal(0, 1, 0);

				triBuffer.vertex(matrix, x3, y3, z3).color(100, 150, 255, 255).normal(0, 1, 0);
				triBuffer.vertex(matrix, x1, y1, z1).color(100, 150, 255, 255).normal(0, 1, 0);
			}

			// === 3. 渲染扭杆 (使用 torsionBuffer，橙色) ===
			VertexConsumer torsionBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			for (int i = 0; i < world.torsionbars.count; i++) {
				int n1 = world.torsionbars.node1[i];
				int n2 = world.torsionbars.node2[i];
				int n3 = world.torsionbars.node3[i];
				int n4 = world.torsionbars.node4[i];

				float x1 = (float)world.nodes.posX[n1]; float y1 = (float)world.nodes.posY[n1]; float z1 = (float)world.nodes.posZ[n1];
				float x2 = (float)world.nodes.posX[n2]; float y2 = (float)world.nodes.posY[n2]; float z2 = (float)world.nodes.posZ[n2];
				float x3 = (float)world.nodes.posX[n3]; float y3 = (float)world.nodes.posY[n3]; float z3 = (float)world.nodes.posZ[n3];
				float x4 = (float)world.nodes.posX[n4]; float y4 = (float)world.nodes.posY[n4]; float z4 = (float)world.nodes.posZ[n4];

				// 用醒目的橙色(255, 165, 0)画出扭杆的折线：A->B, B->C, C->D
				torsionBuffer.vertex(matrix, x1, y1, z1).color(255, 165, 0, 255).normal(0, 1, 0);
				torsionBuffer.vertex(matrix, x2, y2, z2).color(255, 165, 0, 255).normal(0, 1, 0);

				torsionBuffer.vertex(matrix, x2, y2, z2).color(255, 165, 0, 255).normal(0, 1, 0);
				torsionBuffer.vertex(matrix, x3, y3, z3).color(255, 165, 0, 255).normal(0, 1, 0);

				torsionBuffer.vertex(matrix, x3, y3, z3).color(255, 165, 0, 255).normal(0, 1, 0);
				torsionBuffer.vertex(matrix, x4, y4, z4).color(255, 165, 0, 255).normal(0, 1, 0);
			}

			stack.pop();
		});
	}
}