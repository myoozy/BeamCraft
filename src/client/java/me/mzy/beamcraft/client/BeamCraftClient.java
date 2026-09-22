package me.mzy.beamcraft.client;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.assets.AssetScanner;
import me.mzy.beamcraft.client.config.BeamCraftConfig;
import me.mzy.beamcraft.client.config.BeamCraftConfigManager;
import me.mzy.beamcraft.client.input.VehicleInputHandler;
import me.mzy.beamcraft.client.render.PhysicsVehicleRenderer;
import me.mzy.beamcraft.client.render.VehicleTextureUploader;
import me.mzy.beamcraft.client.physics.AsyncPhysicsScheduler;
import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.VehicleCameraData;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.keybinding.v1.KeyBindingHelper;
import net.fabricmc.fabric.api.client.rendering.v1.EntityRendererRegistry;
import net.fabricmc.fabric.api.client.rendering.v1.WorldRenderEvents;
import net.fabricmc.fabric.api.client.rendering.v1.HudRenderCallback;
import net.fabricmc.loader.api.FabricLoader;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.option.KeyBinding;
import net.minecraft.client.render.RenderLayer;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.client.util.InputUtil;
import net.minecraft.text.Text;
import net.minecraft.util.math.Vec3d;
import org.lwjgl.glfw.GLFW;

import java.io.File;

public class BeamCraftClient implements ClientModInitializer {
	private static final boolean DEBUG_DRAW = false;
	private static final boolean DEBUG_SHOW_BEAMS = true;
	private static long lastOverrunNoticeNanos = 0L;
	private static boolean physicsFailureReported = false;
	private static KeyBinding physicsTraceToggleKey;
	public static final double DELTA_TIME = 0.05;

	public static final PhysicsWorld PHYSICS_WORLD = new PhysicsWorld();
	public static final AsyncPhysicsScheduler PHYSICS_SCHEDULER = new AsyncPhysicsScheduler(PHYSICS_WORLD);
	public static final File GAME_DIR = FabricLoader.getInstance().getGameDir().toFile();

	// 记录物理和扫描耗时 (毫秒)
	public static double lastPhysicsMs = 0.0;

	/**
	 * Scratch for the HUD's body-attitude readout, reused every frame so the render
	 * path stays allocation-free.
	 */
	private static final float[] ATTITUDE_DEG = new float[2];
	public static double lastPhysicsWaitMs = 0.0;
	public static boolean lastPhysicsOverBudget = false;
	public static double[] lastPhysicsMsDetail = new double[60];
	private static final int PHYSICS_TIMING_WINDOW = 100;
	private static final int[] ROLLING_TIMING_INDICES = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 42, 43, 44, 45, 46, 47, 48, 49, 50, 52, 53
	};
	private static final double[][] ROLLING_TIMING_SAMPLES =
			new double[ROLLING_TIMING_INDICES.length][PHYSICS_TIMING_WINDOW];
	private static final double[] ROLLING_TIMING_SUMS = new double[ROLLING_TIMING_INDICES.length];
	private static final double[] ROLLING_TIMING_MINS = new double[ROLLING_TIMING_INDICES.length];
	private static final double[] ROLLING_TIMING_MAXES = new double[ROLLING_TIMING_INDICES.length];
	private static int rollingTimingSampleCount;
	private static int rollingTimingWriteIndex;
	private static int rollingTimingVehicleCount = -1;

	@Override
	public void onInitializeClient() {

		// 加载配置文件，确定资产根列表并确保目录存在
		BeamCraftConfig config = BeamCraftConfigManager.initialize(
				FabricLoader.getInstance().getConfigDir(), GAME_DIR);
		PHYSICS_WORLD.configureEventTrace(
				config.diagnostics.physicsEventTrace,
				FabricLoader.getInstance().getGameDir().resolve("beamcraft-traces"));
		if (config.diagnostics.physicsEventTrace) {
			physicsTraceToggleKey = KeyBindingHelper.registerKeyBinding(new KeyBinding(
					"key.beamcraft.physics_trace_toggle",
					InputUtil.Type.KEYSYM,
					GLFW.GLFW_KEY_F8,
					"key.categories.misc"));
		}
		VehicleInputHandler inputHandler = new VehicleInputHandler(config.input);
		AssetScanner.INSTANCE.configure(config.policy());
		for (File root : BeamCraftConfigManager.assetRoots()) {
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
					updateRollingPhysicsTimings(lastPhysicsMsDetail, PHYSICS_WORLD.vehicles.size());
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

			if (physicsTraceToggleKey != null) {
				while (physicsTraceToggleKey.wasPressed()) world.toggleEventTrace();
			}

			// Vehicle creation/removal is safe only after the previous job joined.
			ClientVehicleManager.update(client);
			if (client.player == null || client.world == null || PHYSICS_SCHEDULER.failure() != null) return;

			inputHandler.tick(client);

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

			String physicsStepText = String.format(
					"BeamCraft Physics current/avg/min/max (%d): %.2f / %.2f / %.2f / %.2f ms",
					rollingTimingSampleCount, lastPhysicsMs, rollingAverage(0),
					ROLLING_TIMING_MINS[0], ROLLING_TIMING_MAXES[0]);
			SoftBodyVehicle debugVehicle = PHYSICS_WORLD.vehicles.isEmpty() ? null : PHYSICS_WORLD.vehicles.getFirst();
			String powertrainState = debugVehicle == null ? "no vehicle" : debugVehicle.powertrain.diagnostic();
			float engineRPM = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugEngineRPM();
			float throttleInput = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugThrottle();
			float actualThrottle = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugActualThrottle();
			float clutchEngagement = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugClutchEngagement();
			float clutchTorque = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugClutchTorque();
			float combustionTorque = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugCombustionTorque();
			float turboRPM = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugTurboRPM();
			float turboBoostPSI = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugTurboBoostPSI();
			boolean turboExisting = debugVehicle != null && debugVehicle.powertrain.debugTurboExisting();
			float superchargerRPM = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugSuperchargerRPM();
			float superchargerBoostPSI = debugVehicle == null ? 0.0f
					: debugVehicle.powertrain.debugSuperchargerBoostPSI();
			boolean superchargerExisting = debugVehicle != null
					&& debugVehicle.powertrain.debugSuperchargerExisting();
			int torqueCurvePoints = debugVehicle == null ? 0 : debugVehicle.powertrain.debugTorqueCurveCount();
			boolean starterActive = debugVehicle != null && debugVehicle.powertrain.debugStarterActive();
			boolean sparkEnabled = debugVehicle != null && debugVehicle.powertrain.debugSparkEnabled();
			boolean fuelEnabled = debugVehicle != null && debugVehicle.powertrain.debugFuelEnabled();
			boolean limiterActive = debugVehicle != null && debugVehicle.powertrain.debugLimiterActive();
			float limiterTime = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugLimiterCutRemaining();
			String gearName = debugVehicle == null ? "?" : debugVehicle.powertrain.debugCurrentGearName();
			float gearRatio = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugActiveRatio();
			float shiftTime = debugVehicle == null ? 0.0f : debugVehicle.powertrain.debugShiftRemaining();
			String rangeMode = debugVehicle == null ? "-" : debugVehicle.powertrain.debugRangeBoxMode();
			float rangeRatio = debugVehicle == null ? 1.0f : debugVehicle.powertrain.debugRangeBoxRatio();

			// Body attitude, measured from the vehicle's authored refNodes triple the way
			// BeamNG builds its body frame, so the numbers can be lined up against
			// BeamNG's own pitch/roll readout instead of judged by eye.
			boolean hasAttitude = debugVehicle != null && debugVehicle.bodyAttitudeDeg(ATTITUDE_DEG);
			VehicleCameraData.RefNodes refNodes = debugVehicle == null ? null : debugVehicle.cameras.refNodes();
			String refLabel = debugVehicle == null || refNodes == null
					? "none"
					: debugVehicle.nodes.names[refNodes.ref()] + "->" + debugVehicle.nodes.names[refNodes.back()]
							+ "/" + debugVehicle.nodes.names[refNodes.left()];

			double narrowChecks = lastPhysicsMsDetail[16];
			double narrowAabbPassed = lastPhysicsMsDetail[17];
			double narrowResolved = lastPhysicsMsDetail[18];
			double narrowCertificateSkipped = lastPhysicsMsDetail[37];
			double aabbPassPercent = narrowChecks == 0.0 ? 0.0 : narrowAabbPassed * 100.0 / narrowChecks;
			double resolvePercent = narrowChecks == 0.0 ? 0.0 : narrowResolved * 100.0 / narrowChecks;
			String batchSizes = String.format("%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f | %.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f",
					lastPhysicsMsDetail[21], lastPhysicsMsDetail[22], lastPhysicsMsDetail[23], lastPhysicsMsDetail[24],
					lastPhysicsMsDetail[25], lastPhysicsMsDetail[26], lastPhysicsMsDetail[27], lastPhysicsMsDetail[28],
					lastPhysicsMsDetail[29], lastPhysicsMsDetail[30], lastPhysicsMsDetail[31], lastPhysicsMsDetail[32],
					lastPhysicsMsDetail[33], lastPhysicsMsDetail[34], lastPhysicsMsDetail[35], lastPhysicsMsDetail[36]);

			String[] lines = {
					hasAttitude
							? String.format("pitch: %+.2f deg (nose up +) | roll: %+.2f deg (left up +) | refNodes: %s",
									ATTITUDE_DEG[0], ATTITUDE_DEG[1], refLabel)
							: "pitch/roll: refNodes unavailable",
					debugVehicle == null
							? "accel: n/a"
							: String.format("accel: %+.3f g longitudinal", debugVehicle.longitudinalAccelG),
					"powertrain: " + powertrainState,
					//String.format("engine: %.0f rpm | pedal: %.0f%% | throttle: %.0f%%", engineRPM,
					//		throttleInput * 100.0f, actualThrottle * 100.0f),
					//String.format("combustion: %.1f Nm | curve: %d | starter: %s", combustionTorque,
					//		torqueCurvePoints, starterActive ? "on" : "off"),
					//turboExisting
					//		? String.format("turbo: %.0f rpm | boost: %.1f psi", turboRPM, turboBoostPSI)
					//		: "turbo: off",
					//superchargerExisting
					//		? String.format("supercharger: %.0f rpm | boost: %.1f psi",
					//				superchargerRPM, superchargerBoostPSI)
					//		: "supercharger: off",
					//String.format("spark/fuel: %s/%s | limiter: %s %.3fs",
					//		sparkEnabled ? "on" : "off", fuelEnabled ? "on" : "off",
					//		limiterActive ? "cut" : "ready", limiterTime),
					//String.format("gear: %s | ratio: %.3f | shift: %.3fs | range: %s %.3f",
					//		gearName, gearRatio, shiftTime, rangeMode, rangeRatio),
					//String.format("clutch engagement: %.0f%% | torque: %.1f Nm", clutchEngagement * 100.0f, clutchTorque),
					String.format("tickBarrierWait: %.2f ms", lastPhysicsWaitMs),
					timingSummary("mcWorldScan", 1),
					timingSummary("internalForce", 2),
					timingSummary("chunk refit", 3),
					timingBreakdown("refit node/chunk/coarseSAP", 42, 43, 44),
					timingBreakdown("refit localSAP/triangle/meshlet", 45, 46, 47),
					timingBreakdown("localSAP key/sort/prefix", 48, 49, 50),
					String.format("chunk span >2x eligible node/meshlet: %.0f/%.0f / %.0f/%.0f | max: %.2fx / %.2fx",
							lastPhysicsMsDetail[55], lastPhysicsMsDetail[54],
							lastPhysicsMsDetail[58], lastPhysicsMsDetail[57],
							lastPhysicsMsDetail[56], lastPhysicsMsDetail[59]),
					timingSummary("candidate+color", 4),
					timingSummary("candidate wall", 9),
					String.format("candidate tasks: %.0f | %s", lastPhysicsMsDetail[51],
							timingPair("work/merge", 52, 53)),
					timingSummary("color", 12),
					String.format("last fine AABB hits/stored/dropped: %.0f / %.0f / %.0f",
							lastPhysicsMsDetail[13], lastPhysicsMsDetail[14], lastPhysicsMsDetail[15]),
					String.format("last chunk pairs productive/overlap/tested: %.0f / %.0f / %.0f (%.2f%% productive)",
							lastPhysicsMsDetail[41], lastPhysicsMsDetail[39], lastPhysicsMsDetail[38],
							lastPhysicsMsDetail[39] > 0.0
									? lastPhysicsMsDetail[41] * 100.0 / lastPhysicsMsDetail[39]
									: 0.0),
					String.format("last fine AABB passed/tested: %.0f / %.0f (%.2f%%)",
							lastPhysicsMsDetail[13], lastPhysicsMsDetail[40],
							lastPhysicsMsDetail[40] > 0.0
									? lastPhysicsMsDetail[13] * 100.0 / lastPhysicsMsDetail[40]
									: 0.0),
					timingSummary("softCollision", 5),
					String.format("narrow checks/cert-skip/AABB/resolved: %.0f / %.0f / %.0f (%.1f%%) / %.0f (%.2f%%)",
							narrowChecks, narrowCertificateSkipped, narrowAabbPassed, aabbPassPercent,
							narrowResolved, resolvePercent),
					String.format("batches: %.0f | largest: %.0f | sizes: %s",
							lastPhysicsMsDetail[19], lastPhysicsMsDetail[20], batchSizes),
					timingSummary("mcCollision", 6),
					timingSummary("postUpdate", 7),
					timingSummary("moveEntity", 8)
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

	private static void updateRollingPhysicsTimings(double[] timings, int vehicleCount) {
		if (vehicleCount != rollingTimingVehicleCount) {
			rollingTimingVehicleCount = vehicleCount;
			rollingTimingSampleCount = 0;
			rollingTimingWriteIndex = 0;
			for (int metric = 0; metric < ROLLING_TIMING_SUMS.length; metric++) {
				ROLLING_TIMING_SUMS[metric] = 0.0;
			}
		}
		int overwrittenIndex = rollingTimingWriteIndex;
		boolean windowFull = rollingTimingSampleCount == PHYSICS_TIMING_WINDOW;
		if (!windowFull) rollingTimingSampleCount++;

		for (int metric = 0; metric < ROLLING_TIMING_INDICES.length; metric++) {
			double value = timings[ROLLING_TIMING_INDICES[metric]];
			if (windowFull) ROLLING_TIMING_SUMS[metric] -= ROLLING_TIMING_SAMPLES[metric][overwrittenIndex];
			ROLLING_TIMING_SAMPLES[metric][overwrittenIndex] = value;
			ROLLING_TIMING_SUMS[metric] += value;
		}
		rollingTimingWriteIndex = (rollingTimingWriteIndex + 1) % PHYSICS_TIMING_WINDOW;

		for (int metric = 0; metric < ROLLING_TIMING_INDICES.length; metric++) {
			double min = Double.POSITIVE_INFINITY;
			double max = Double.NEGATIVE_INFINITY;
			for (int sample = 0; sample < rollingTimingSampleCount; sample++) {
				double value = ROLLING_TIMING_SAMPLES[metric][sample];
				if (value < min) min = value;
				if (value > max) max = value;
			}
			ROLLING_TIMING_MINS[metric] = min;
			ROLLING_TIMING_MAXES[metric] = max;
		}
	}

	private static String timingSummary(String label, int timingIndex) {
		int metric = rollingMetricForTimingIndex(timingIndex);
		return String.format("%s current/avg/min/max: %.2f / %.2f / %.2f / %.2f ms", label,
				lastPhysicsMsDetail[timingIndex], rollingAverage(metric),
				ROLLING_TIMING_MINS[metric], ROLLING_TIMING_MAXES[metric]);
	}

	private static String timingBreakdown(String label, int first, int second, int third) {
		return String.format("%s current/avg: %.2f/%.2f / %.2f/%.2f / %.2f/%.2f ms", label,
				lastPhysicsMsDetail[first], rollingAverage(rollingMetricForTimingIndex(first)),
				lastPhysicsMsDetail[second], rollingAverage(rollingMetricForTimingIndex(second)),
				lastPhysicsMsDetail[third], rollingAverage(rollingMetricForTimingIndex(third)));
	}

	private static String timingPair(String label, int first, int second) {
		return String.format("%s current/avg: %.2f/%.2f / %.2f/%.2f ms", label,
				lastPhysicsMsDetail[first], rollingAverage(rollingMetricForTimingIndex(first)),
				lastPhysicsMsDetail[second], rollingAverage(rollingMetricForTimingIndex(second)));
	}

	private static int rollingMetricForTimingIndex(int timingIndex) {
		for (int metric = 0; metric < ROLLING_TIMING_INDICES.length; metric++) {
			if (ROLLING_TIMING_INDICES[metric] == timingIndex) return metric;
		}
		throw new IllegalArgumentException("Timing index is not tracked: " + timingIndex);
	}

	private static double rollingAverage(int metric) {
		return rollingTimingSampleCount == 0 ? 0.0 : ROLLING_TIMING_SUMS[metric] / rollingTimingSampleCount;
	}
}
