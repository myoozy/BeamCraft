package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.render.PhysicsVehicleRenderer;
import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.rendering.v1.EntityRendererRegistry;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.fabricmc.fabric.api.client.rendering.v1.HudRenderCallback;
import net.fabricmc.loader.api.FabricLoader;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.RenderLayer;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.InputUtil;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.Vec3d;
import org.lwjgl.glfw.GLFW;

import java.io.File;

public class BeamCraftClient implements ClientModInitializer {
	// 记录上一帧 G 键有没有被按下
	private static boolean gWasPressed = false;
	public static final double DELTA_TIME = 0.05;

	public static final PhysicsWorld PHYSICS_WORLD = new PhysicsWorld();
	public static final File GAME_DIR = FabricLoader.getInstance().getGameDir().toFile();
	public static final File VEHICLES_DIR = new File(GAME_DIR, "mods/beamcraft/vehicles");

	// 记录物理和扫描耗时 (毫秒)
	public static double lastPhysicsMs = 0.0;
	public static double[] lastPhysicsMsDetail = new double[9];

	@Override
	public void onInitializeClient() {

		// 确保目录存在
		if (!VEHICLES_DIR.exists()) VEHICLES_DIR.mkdirs();

		ClientTickEvents.END_CLIENT_TICK.register(client -> {
			// 让管理器接管一切生命周期
			ClientVehicleManager.update(client);
		});

		DaeMeshLoader.scanAndLoadAllVehicles(VEHICLES_DIR);
		ClientVehicleManager.initRenderHooks(); // 初始化渲染
		EntityRendererRegistry.register(BeamCraft.PHYSICS_VEHICLE_ENTITY, PhysicsVehicleRenderer::new);

		// 1. 物理计算与控制循环 (每帧运行)
		ClientTickEvents.END_CLIENT_TICK.register(client -> {
			if (client.player == null || client.world == null) return;
			PhysicsWorld world = PHYSICS_WORLD;

			// 检测 G 键 (调试功能：瞬间重置所有现存车辆，并传送到玩家头顶)
			boolean isG = InputUtil.isKeyPressed(client.getWindow().getHandle(), GLFW.GLFW_KEY_G);
			if (isG && !gWasPressed) {
				double HEIGHT_OFFSET = 1;
				for (SoftBodyVehicle vehicle : world.vehicles) {
					vehicle.reset();
					// 把 MC 实体强行瞬移过来
					vehicle.parentEntity.setPosition(client.player.getX(), client.player.getY() + HEIGHT_OFFSET, client.player.getZ());
					vehicle.nodes.rotateNodes(client.player.getYaw(), 0, 0);
				}
			}
			gWasPressed = isG;

			// 统一执行物理世界所有车辆的更新
			if (!world.vehicles.isEmpty()) {
				long t1 = System.nanoTime();
				world.step(client.world, DELTA_TIME, lastPhysicsMsDetail);
				long t2 = System.nanoTime();

				lastPhysicsMs = (t2 - t1) / 1_000_000.0;
			}
		});

		// 2. HUD 性能监控面板
		HudRenderCallback.EVENT.register((drawContext, tickCounter) -> {
			MinecraftClient client = MinecraftClient.getInstance();
			if (client.options.hudHidden) return; // 如果按了 F1 隐藏界面，就不画

			String physicsStepText = String.format("BeamCraft Physics: %.2f ms", lastPhysicsMs);
			String[] lines = {
					String.format("mcWorldScan: %.2f ms", lastPhysicsMsDetail[1]),
					String.format("internalForce: %.2f ms", lastPhysicsMsDetail[2]),
					String.format("globalSAP: %.2f ms", lastPhysicsMsDetail[3]),
					String.format("dyeCollision: %.2f ms", lastPhysicsMsDetail[4]),
					String.format("softCollision: %.2f ms", lastPhysicsMsDetail[5]),
					String.format("mcCollision: %.2f ms", lastPhysicsMsDetail[6]),
					String.format("postUpdate: %.2f ms", lastPhysicsMsDetail[7]),
					String.format("moveEntity: %.2f ms", lastPhysicsMsDetail[8])
			};

			int color = (lastPhysicsMs > 10.0) ? 0xFF0000 : 0x00FF00;

			// 标题
			drawContext.drawTextWithShadow(
					client.textRenderer,
					physicsStepText,
					10,
					10,
					color
			);

			// 逐行绘制 detail
			int startY = 25;

			for (int i = 0; i < lines.length; i++) {
				drawContext.drawTextWithShadow(
						client.textRenderer,
						lines[i],
						10,
						startY + i * 10,
						color
				);
			}
		});

		// 3. 渲染循环 (遍历所有车，并将局部坐标叠加上实体坐标)
		WorldRenderEvents.AFTER_ENTITIES.register(context -> {
			PhysicsWorld world = PHYSICS_WORLD;
			boolean DEBUG_DRAW = true;
			if (!DEBUG_DRAW) return;
			if (world == null || world.vehicles.isEmpty()) return;

			Vec3d cameraPos = context.camera().getPos();
			MatrixStack stack = context.matrixStack();
			stack.push();
			stack.translate(-cameraPos.x, -cameraPos.y, -cameraPos.z);
			org.joml.Matrix4f matrix = stack.peek().getPositionMatrix();

			VertexConsumer beamBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			VertexConsumer triBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			VertexConsumer torsionBuffer = context.consumers().getBuffer(RenderLayer.getLines());

			// 遍历管理器里的每一辆车
			for (SoftBodyVehicle vehicle : world.vehicles) {

				// 获取这辆车绑定的 MC 实体当前的世界坐标
				double eX = vehicle.parentEntity.getX();
				double eY = vehicle.parentEntity.getY();
				double eZ = vehicle.parentEntity.getZ();

				boolean DEBUG_SHOW_BEAMS = true;

				// === 1. 渲染梁/骨架 ===
				if (DEBUG_SHOW_BEAMS) {

					// === 1. 渲染普通梁（NORMAL） ===
					for (int i = 0; i < vehicle.normalBeams.count; i++) {
						int n1 = vehicle.normalBeams.node1[i];
						int n2 = vehicle.normalBeams.node2[i];

						float x1 = (float) (vehicle.nodes.posX[n1] + eX);
						float y1 = (float) (vehicle.nodes.posY[n1] + eY);
						float z1 = (float) (vehicle.nodes.posZ[n1] + eZ);
						float x2 = (float) (vehicle.nodes.posX[n2] + eX);
						float y2 = (float) (vehicle.nodes.posY[n2] + eY);
						float z2 = (float) (vehicle.nodes.posZ[n2] + eZ);

						if (vehicle.normalBeams.restLength[i] != vehicle.normalBeams.targetRestLength[i]) {
							// 过渡中：黄色
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 255, 0, 255).normal(0, 1, 0);
						} else if (vehicle.normalBeams.broken[i]) {
							// 断裂：红色
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 0, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 0, 0, 255).normal(0, 1, 0);
						} else {
							// 正常：绿色
							beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
						}
					}

					// === 2. 渲染支撑梁（SUPPORT） ===
					for (int i = 0; i < vehicle.supportBeams.count; i++) {
						int n1 = vehicle.supportBeams.node1[i];
						int n2 = vehicle.supportBeams.node2[i];

						float x1 = (float) (vehicle.nodes.posX[n1] + eX);
						float y1 = (float) (vehicle.nodes.posY[n1] + eY);
						float z1 = (float) (vehicle.nodes.posZ[n1] + eZ);
						float x2 = (float) (vehicle.nodes.posX[n2] + eX);
						float y2 = (float) (vehicle.nodes.posY[n2] + eY);
						float z2 = (float) (vehicle.nodes.posZ[n2] + eZ);

						if (vehicle.supportBeams.restLength[i] != vehicle.supportBeams.targetRestLength[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 255, 0, 255).normal(0, 1, 0);
						} else if (vehicle.supportBeams.broken[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 0, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 0, 0, 255).normal(0, 1, 0);
						} else {
							beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
						}
					}

					// === 3. 渲染限界梁（	BOUNDED） ===
					for (int i = 0; i < vehicle.boundedBeams.count; i++) {
						int n1 = vehicle.boundedBeams.node1[i];
						int n2 = vehicle.boundedBeams.node2[i];

						float x1 = (float) (vehicle.nodes.posX[n1] + eX);
						float y1 = (float) (vehicle.nodes.posY[n1] + eY);
						float z1 = (float) (vehicle.nodes.posZ[n1] + eZ);
						float x2 = (float) (vehicle.nodes.posX[n2] + eX);
						float y2 = (float) (vehicle.nodes.posY[n2] + eY);
						float z2 = (float) (vehicle.nodes.posZ[n2] + eZ);

						if (vehicle.boundedBeams.restLength[i] != vehicle.boundedBeams.targetRestLength[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 255, 0, 255).normal(0, 1, 0);
						} else if (vehicle.boundedBeams.broken[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 0, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 0, 0, 255).normal(0, 1, 0);
						} else {
							beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
						}
					}

					// === 3. 渲染LBeam===
					for (int i = 0; i < vehicle.lBeams.count; i++) {
						int n1 = vehicle.lBeams.node1[i];
						int n2 = vehicle.lBeams.node2[i];

						float x1 = (float) (vehicle.nodes.posX[n1] + eX);
						float y1 = (float) (vehicle.nodes.posY[n1] + eY);
						float z1 = (float) (vehicle.nodes.posZ[n1] + eZ);
						float x2 = (float) (vehicle.nodes.posX[n2] + eX);
						float y2 = (float) (vehicle.nodes.posY[n2] + eY);
						float z2 = (float) (vehicle.nodes.posZ[n2] + eZ);

						if (vehicle.lBeams.restCosTheta[i] != vehicle.lBeams.targetCosTheta[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 255, 0, 255).normal(0, 1, 0);
						} else if (vehicle.lBeams.broken[i]) {
							beamBuffer.vertex(matrix, x1, y1, z1).color(255, 0, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(255, 0, 0, 255).normal(0, 1, 0);
						} else {
							beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
							beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
						}
					}
				}

				// === 2. 渲染三角面 (浅蓝色轮廓) ===
				for (int i = 0; i < vehicle.triangles.count; i++) {
					int n1 = vehicle.triangles.node1[i];
					int n2 = vehicle.triangles.node2[i];
					int n3 = vehicle.triangles.node3[i];

					float x1 = (float)(vehicle.nodes.posX[n1] + eX); float y1 = (float)(vehicle.nodes.posY[n1] + eY); float z1 = (float)(vehicle.nodes.posZ[n1] + eZ);
					float x2 = (float)(vehicle.nodes.posX[n2] + eX); float y2 = (float)(vehicle.nodes.posY[n2] + eY); float z2 = (float)(vehicle.nodes.posZ[n2] + eZ);
					float x3 = (float)(vehicle.nodes.posX[n3] + eX); float y3 = (float)(vehicle.nodes.posY[n3] + eY); float z3 = (float)(vehicle.nodes.posZ[n3] + eZ);

					triBuffer.vertex(matrix, x1, y1, z1).color(100, 150, 255, 255).normal(0, 1, 0);
					triBuffer.vertex(matrix, x2, y2, z2).color(100, 150, 255, 255).normal(0, 1, 0);
					triBuffer.vertex(matrix, x2, y2, z2).color(100, 150, 255, 255).normal(0, 1, 0);
					triBuffer.vertex(matrix, x3, y3, z3).color(100, 150, 255, 255).normal(0, 1, 0);
					triBuffer.vertex(matrix, x3, y3, z3).color(100, 150, 255, 255).normal(0, 1, 0);
					triBuffer.vertex(matrix, x1, y1, z1).color(100, 150, 255, 255).normal(0, 1, 0);
				}

				// === 3. 渲染扭杆 (橙色) ===
				for (int i = 0; i < vehicle.torsionbars.count; i++) {
					int n1 = vehicle.torsionbars.node1[i];
					int n2 = vehicle.torsionbars.node2[i];
					int n3 = vehicle.torsionbars.node3[i];
					int n4 = vehicle.torsionbars.node4[i];

					float x1 = (float)(vehicle.nodes.posX[n1] + eX); float y1 = (float)(vehicle.nodes.posY[n1] + eY); float z1 = (float)(vehicle.nodes.posZ[n1] + eZ);
					float x2 = (float)(vehicle.nodes.posX[n2] + eX); float y2 = (float)(vehicle.nodes.posY[n2] + eY); float z2 = (float)(vehicle.nodes.posZ[n2] + eZ);
					float x3 = (float)(vehicle.nodes.posX[n3] + eX); float y3 = (float)(vehicle.nodes.posY[n3] + eY); float z3 = (float)(vehicle.nodes.posZ[n3] + eZ);
					float x4 = (float)(vehicle.nodes.posX[n4] + eX); float y4 = (float)(vehicle.nodes.posY[n4] + eY); float z4 = (float)(vehicle.nodes.posZ[n4] + eZ);

					torsionBuffer.vertex(matrix, x1, y1, z1).color(255, 165, 0, 255).normal(0, 1, 0);
					torsionBuffer.vertex(matrix, x2, y2, z2).color(255, 165, 0, 255).normal(0, 1, 0);
					torsionBuffer.vertex(matrix, x2, y2, z2).color(255, 165, 0, 255).normal(0, 1, 0);
					torsionBuffer.vertex(matrix, x3, y3, z3).color(255, 165, 0, 255).normal(0, 1, 0);
					torsionBuffer.vertex(matrix, x3, y3, z3).color(255, 165, 0, 255).normal(0, 1, 0);
					torsionBuffer.vertex(matrix, x4, y4, z4).color(255, 165, 0, 255).normal(0, 1, 0);
				}
			}

			stack.pop();
		});
	}
}