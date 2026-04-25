package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.physics.PhysicsWorld;
import me.mzy.beamcraft.physics.SoftBodyVehicle;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.rendering.v1.EntityRendererRegistry;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.fabricmc.fabric.api.client.rendering.v1.HudRenderCallback;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.RenderLayer;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.render.entity.EntityRenderer;
import net.minecraft.client.render.entity.EntityRendererFactory;
import net.minecraft.client.util.InputUtil;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.Vec3d;
import org.lwjgl.glfw.GLFW;

public class BeamCraftClient implements ClientModInitializer {
	// 记录上一帧 G 键有没有被按下
	private static boolean gWasPressed = false;
	public static final double DELTA_TIME = 0.05;

	// 记录物理和扫描耗时 (毫秒)
	public static double lastPhysicsMs = 0.0;
	public static double lastScanMs = 0.0;

	@Override
	public void onInitializeClient() {

		EntityRendererRegistry.register(BeamCraft.PHYSICS_VEHICLE_ENTITY, new EntityRendererFactory<PhysicsVehicleEntity>() {
			@Override
			public EntityRenderer<PhysicsVehicleEntity> create(Context context) {
				return new EntityRenderer<PhysicsVehicleEntity>(context) {
					@Override
					public Identifier getTexture(PhysicsVehicleEntity entity) {
						// 返回一个虚拟的贴图路径即可（因为我们自己用事件画线框，不需要它的贴图）
						return Identifier.of(BeamCraft.MOD_ID, "textures/entity/dummy.png");
					}
				};
			}
		});

		// 1. 物理计算与控制循环 (每帧运行)
		ClientTickEvents.END_CLIENT_TICK.register(client -> {
			if (client.player == null || client.world == null) return;
			PhysicsWorld world = BeamCraft.PHYSICS_WORLD;

			// 检测 G 键 (调试功能：瞬间重置所有现存车辆，并传送到玩家头顶)
			boolean isG = InputUtil.isKeyPressed(client.getWindow().getHandle(), GLFW.GLFW_KEY_G);
			if (isG && !gWasPressed) {
				double HEIGHT_OFFSET = 5;
				for (SoftBodyVehicle vehicle : world.vehicles) {
					// 把 MC 实体强行瞬移过来
					vehicle.parentEntity.setPosition(client.player.getX(), client.player.getY() + HEIGHT_OFFSET, client.player.getZ());
					// 局部坐标系下清空形变和速度
					vehicle.spawnAt(0, 0, 0);
				}
			}
			gWasPressed = isG;

			// 统一执行物理世界所有车辆的更新
			if (!world.vehicles.isEmpty()) {
				long t1 = System.nanoTime();
				world.preStep(client.world, DELTA_TIME);
				long t2 = System.nanoTime();
				world.step(DELTA_TIME);
				long t3 = System.nanoTime();

				lastScanMs = (t2 - t1) / 1_000_000.0;
				lastPhysicsMs = (t3 - t2) / 1_000_000.0;
			}
		});

		// 2. HUD 性能监控面板
		HudRenderCallback.EVENT.register((drawContext, tickCounter) -> {
			MinecraftClient client = MinecraftClient.getInstance();
			if (client.options.hudHidden) return; // 如果按了 F1 隐藏界面，就不画

			String physicsStepText = String.format("BeamCraft Physics: %.2f ms", lastPhysicsMs);
			String physicsScanText = String.format("BeamCraft Physics Scan: %.2f ms", lastScanMs);

			// 耗时超过 10ms 变红警告，否则亮绿
			int color = (lastPhysicsMs + lastScanMs > 10.0) ? 0xFF0000 : 0x00FF00;

			drawContext.drawTextWithShadow(client.textRenderer, physicsStepText, 10, 10, color);
			drawContext.drawTextWithShadow(client.textRenderer, physicsScanText, 10, 20, color);
		});

		// 3. 渲染循环 (遍历所有车，并将局部坐标叠加上实体坐标)
		WorldRenderEvents.AFTER_ENTITIES.register(context -> {
			PhysicsWorld world = BeamCraft.PHYSICS_WORLD;
			if (world == null || world.vehicles.isEmpty()) return;

			Vec3d cameraPos = context.camera().getPos();
			MatrixStack stack = context.matrixStack();
			stack.push();
			stack.translate(-cameraPos.x, -cameraPos.y, -cameraPos.z);
			org.joml.Matrix4f matrix = stack.peek().getPositionMatrix();

			VertexConsumer beamBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			VertexConsumer triBuffer = context.consumers().getBuffer(RenderLayer.getLines());
			VertexConsumer torsionBuffer = context.consumers().getBuffer(RenderLayer.getLines());

			// 核心改变：遍历管理器里的每一辆车！
			for (SoftBodyVehicle vehicle : world.vehicles) {

				// 获取这辆车绑定的 MC 实体当前的世界坐标
				double eX = vehicle.parentEntity.getX();
				double eY = vehicle.parentEntity.getY();
				double eZ = vehicle.parentEntity.getZ();

				// === 1. 渲染梁/骨架 ===
				for (int i = 0; i < vehicle.beams.count; i++) {
					if (vehicle.beams.broken[i]) continue; // 断裂的隐藏

					int n1 = vehicle.beams.node1[i];
					int n2 = vehicle.beams.node2[i];

					// 【核心逻辑】：将 SoftBody 的内部坐标 + 实体的世界坐标
					float x1 = (float)(vehicle.nodes.posX[n1] + eX);
					float y1 = (float)(vehicle.nodes.posY[n1] + eY);
					float z1 = (float)(vehicle.nodes.posZ[n1] + eZ);

					float x2 = (float)(vehicle.nodes.posX[n2] + eX);
					float y2 = (float)(vehicle.nodes.posY[n2] + eY);
					float z2 = (float)(vehicle.nodes.posZ[n2] + eZ);

					//beamBuffer.vertex(matrix, x1, y1, z1).color(0, 255, 0, 255).normal(0, 1, 0);
					//beamBuffer.vertex(matrix, x2, y2, z2).color(0, 255, 0, 255).normal(0, 1, 0);
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