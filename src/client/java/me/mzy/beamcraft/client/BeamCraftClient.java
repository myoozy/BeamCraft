package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.assets.BeamCraftConfig;
import me.mzy.beamcraft.client.model.DaeMeshLoader;
import me.mzy.beamcraft.client.render.PhysicsVehicleRenderer;
import me.mzy.beamcraft.client.render.VehicleTextureUploader;
import me.mzy.beamcraft.client.physics.AsyncPhysicsScheduler;
import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
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
import net.minecraft.text.Text;
import net.minecraft.util.math.Vec3d;
import org.lwjgl.glfw.GLFW;

import java.io.File;
import java.util.List;

public class BeamCraftClient implements ClientModInitializer {
	private static final boolean DEBUG_DRAW = false;
	private static final boolean DEBUG_SHOW_BEAMS = true;
	// 记录上一帧 G 键有没有被按下
	private static boolean gWasPressed = false;
	private static boolean shiftUpWasPressed = false;
	private static boolean shiftDownWasPressed = false;
	private static long lastOverrunNoticeNanos = 0L;
	private static boolean physicsFailureReported = false;
	public static final double DELTA_TIME = 0.05;

	public static final PhysicsWorld PHYSICS_WORLD = new PhysicsWorld();
	public static final AsyncPhysicsScheduler PHYSICS_SCHEDULER = new AsyncPhysicsScheduler(PHYSICS_WORLD);
	public static final File GAME_DIR = FabricLoader.getInstance().getGameDir().toFile();
	public static final File VEHICLES_DIR = new File(GAME_DIR, "mods/beamcraft/vehicles");
	// 资产根列表，来自 config/beamcraft.json；默认仍指向 VEHICLES_DIR
	public static volatile List<File> ASSET_ROOTS = List.of(VEHICLES_DIR);

	// 记录物理和扫描耗时 (毫秒)
	public static double lastPhysicsMs = 0.0;
	public static double lastPhysicsWaitMs = 0.0;
	public static boolean lastPhysicsOverBudget = false;
	public static double[] lastPhysicsMsDetail = new double[9];

	@Override
	public void onInitializeClient() {

		// 加载配置文件，确定资产根列表并确保目录存在
		BeamCraftConfig config = BeamCraftConfig.load(FabricLoader.getInstance().getConfigDir());
		ASSET_ROOTS = config.resolveAssetRoots(GAME_DIR);
		AssetScanner.INSTANCE.configure(config.policy());
		for (File root : ASSET_ROOTS) {
			if (!root.exists()) root.mkdirs();
		}

		ClientVehicleManager.initRenderHooks(); // 初始化渲染
		EntityRendererRegistry.register(BeamCraft.PHYSICS_VEHICLE_ENTITY, PhysicsVehicleRenderer::new);

		// Stop CPU physics before closing render-thread-owned resources.
		ClientLifecycleEvents.CLIENT_STOPPING.register(client -> {
			PHYSICS_SCHEDULER.close();
			VehicleTextureUploader.INSTANCE.closeFromAnyThread();
		});

		// One game tick owns one fixed physics step. The preceding step is
		// committed first; only an over-budget step blocks this tick boundary.
		ClientTickEvents.START_CLIENT_TICK.register(client -> {
			PhysicsWorld world = PHYSICS_WORLD;
			AsyncPhysicsScheduler.Completion completion = PHYSICS_SCHEDULER.finishPreviousStep();
			if (completion != null) {
				lastPhysicsWaitMs = completion.waitMs();
				lastPhysicsOverBudget = completion.overBudget();
				if (completion.timings() != null) {
					lastPhysicsMsDetail = completion.timings();
					lastPhysicsMs = lastPhysicsMsDetail[0];
				}

				if (completion.failure() != null && !physicsFailureReported) {
					physicsFailureReported = true;
					BeamCraft.LOGGER.error("Asynchronous BeamCraft physics stopped after a worker failure",
							completion.failure());
					if (client.player != null) {
						client.player.sendMessage(Text.literal(
								"[BeamCraft] Physics worker failed; simulation stopped. Check latest.log."), false);
					}
				} else if (completion.overBudget()) {
					long now = System.nanoTime();
					if (now - lastOverrunNoticeNanos >= 5_000_000_000L) {
						lastOverrunNoticeNanos = now;
						BeamCraft.LOGGER.error("BeamCraft physics step exceeded the 50 ms tick budget: {} ms (tick waited {} ms)",
								String.format("%.2f", lastPhysicsMs), String.format("%.2f", lastPhysicsWaitMs));
						if (client.player != null) {
							client.player.sendMessage(Text.literal(String.format(
									"[BeamCraft] Physics overrun: %.2f ms (tick barrier %.2f ms)",
									lastPhysicsMs, lastPhysicsWaitMs)), false);
						}
					}
				}
			}

			// Vehicle creation/removal is safe only after the previous job joined.
			ClientVehicleManager.update(client);
			if (client.player == null || client.world == null || PHYSICS_SCHEDULER.failure() != null) return;

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

			// Temporary global binary controls until seat/input ownership exists.
			long window = client.getWindow().getHandle();
			float throttle = InputUtil.isKeyPressed(window, GLFW.GLFW_KEY_W) ? 1.0f : 0.0f;
			float clutchPedal = InputUtil.isKeyPressed(window, GLFW.GLFW_KEY_LEFT_SHIFT) ? 1.0f : 0.0f;
			boolean starter = InputUtil.isKeyPressed(window, GLFW.GLFW_KEY_V);
			boolean shiftUp = InputUtil.isKeyPressed(window, GLFW.GLFW_KEY_X);
			boolean shiftDown = InputUtil.isKeyPressed(window, GLFW.GLFW_KEY_Z);
			for (SoftBodyVehicle vehicle : world.vehicles) {
				vehicle.powertrain.setControls(throttle, clutchPedal, starter);
				if (shiftUp && !shiftUpWasPressed) vehicle.powertrain.requestShiftUp();
				if (shiftDown && !shiftDownWasPressed) vehicle.powertrain.requestShiftDown();
			}
			shiftUpWasPressed = shiftUp;
			shiftDownWasPressed = shiftDown;

			// World access happens synchronously in prepareStep; the 100 substeps
			// then run independently until the next game-tick barrier.
			if (!world.vehicles.isEmpty()) {
				PHYSICS_SCHEDULER.startStep(client.world, DELTA_TIME);
			}
		});

		// 2. HUD 性能监控面板
		HudRenderCallback.EVENT.register((drawContext, tickCounter) -> {
			MinecraftClient client = MinecraftClient.getInstance();
			if (client.options.hudHidden) return; // 如果按了 F1 隐藏界面，就不画

			String physicsStepText = String.format("BeamCraft Physics: %.2f ms", lastPhysicsMs);
			SoftBodyVehicle debugVehicle = PHYSICS_WORLD.vehicles.isEmpty() ? null : PHYSICS_WORLD.vehicles.getFirst();
			String powertrainState = debugVehicle == null ? "no vehicle" : debugVehicle.powertrain.diagnostic();
			float engineRPM = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugEngineRPM();
			float throttleInput = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugThrottle();
			float actualThrottle = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugActualThrottle();
			float clutchEngagement = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugClutchEngagement();
			float clutchTorque = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugClutchTorque();
			float combustionTorque = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugCombustionTorque();
			int torqueCurvePoints = debugVehicle == null ? 0 : debugVehicle.powertrain.debugTorqueCurveCount();
			boolean starterActive = debugVehicle != null && debugVehicle.powertrain.debugStarterActive();
			boolean sparkEnabled = debugVehicle != null && debugVehicle.powertrain.debugSparkEnabled();
			boolean fuelEnabled = debugVehicle != null && debugVehicle.powertrain.debugFuelEnabled();
			boolean limiterActive = debugVehicle != null && debugVehicle.powertrain.debugLimiterActive();
			float limiterTime = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugLimiterCutRemaining();
			String gearName = debugVehicle == null ? "?" : debugVehicle.powertrain.debugCurrentGearName();
			float gearRatio = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugActiveRatio();
			float shiftTime = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugShiftRemaining();
			String[] lines = {
					"powertrain: " + powertrainState,
					String.format("engine: %.0f rpm | pedal: %.0f%% | throttle: %.0f%%", engineRPM,
							throttleInput * 100.0f, actualThrottle * 100.0f),
					String.format("combustion: %.1f Nm | curve: %d | starter: %s", combustionTorque,
							torqueCurvePoints, starterActive ? "on" : "off"),
					String.format("spark/fuel: %s/%s | limiter: %s %.3fs",
							sparkEnabled ? "on" : "off", fuelEnabled ? "on" : "off",
							limiterActive ? "cut" : "ready", limiterTime),
					String.format("gear: %s | ratio: %.3f | shift: %.3fs", gearName, gearRatio, shiftTime),
					String.format("clutch engagement: %.0f%% | torque: %.1f Nm", clutchEngagement * 100.0f, clutchTorque),
					String.format("tickBarrierWait: %.2f ms", lastPhysicsWaitMs),
					String.format("mcWorldScan: %.2f ms", lastPhysicsMsDetail[1]),
					String.format("internalForce: %.2f ms", lastPhysicsMsDetail[2]),
					String.format("globalSAP: %.2f ms", lastPhysicsMsDetail[3]),
					String.format("dyeCollision: %.2f ms", lastPhysicsMsDetail[4]),
					String.format("softCollision: %.2f ms", lastPhysicsMsDetail[5]),
					String.format("mcCollision: %.2f ms", lastPhysicsMsDetail[6]),
					String.format("postUpdate: %.2f ms", lastPhysicsMsDetail[7]),
					String.format("moveEntity: %.2f ms", lastPhysicsMsDetail[8])
			};

			int color = (lastPhysicsOverBudget || PHYSICS_SCHEDULER.failure() != null)
					? 0xFF0000
					: (lastPhysicsMs > 10.0 ? 0xFFFF00 : 0x00FF00);

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
