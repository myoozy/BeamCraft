package me.mzy.beamcraft;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.network.VehicleSyncPayload;
import me.mzy.beamcraft.network.VehicleRidePayload;
import me.mzy.beamcraft.network.VehicleSpawnPayload;

import net.fabricmc.api.ModInitializer;
import net.fabricmc.fabric.api.object.builder.v1.entity.FabricEntityTypeBuilder;
import net.minecraft.entity.EntityDimensions;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.SpawnGroup;
import net.minecraft.registry.Registries;
import net.minecraft.registry.Registry;
import net.minecraft.util.Identifier;
import net.minecraft.server.network.ServerPlayerEntity;
import net.minecraft.text.Text;
import net.minecraft.entity.Entity;
import net.fabricmc.fabric.api.networking.v1.PayloadTypeRegistry;
import net.fabricmc.fabric.api.networking.v1.ServerPlayNetworking;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class BeamCraft implements ModInitializer {
	public static final String MOD_ID = "beamcraft";

	// This logger is used to write text to the console and the log file.
	// It is considered best practice to use your mod id as the logger's name.
	// That way, it's clear which mod wrote info, warnings, and errors.
	public static final Logger LOGGER = LoggerFactory.getLogger(MOD_ID);

	// 1. 注册载具实体类型 (1.21 必须使用 Identifier.of)
	public static final EntityType<PhysicsVehicleEntity> PHYSICS_VEHICLE_ENTITY = Registry.register(
			Registries.ENTITY_TYPE,
			Identifier.of(MOD_ID, "physics_vehicle"),
			FabricEntityTypeBuilder.create(SpawnGroup.MISC, PhysicsVehicleEntity::new)
					.dimensions(EntityDimensions.fixed(2.5f, 2.0f)) // 设置一个粗略的逻辑碰撞箱
					.fireImmune() // 载具燃烧完全由未来的油箱/电池事件系统驱动，禁用原版火焰点燃与着火动画
					.build()
	);

	@Override
	public void onInitialize() {
		// This code runs as soon as Minecraft is in a mod-load-ready state.
		// However, some things (like resources) may still be uninitialized.
		// Proceed with mild caution.

		LOGGER.info("Hello Fabric world!");

		// 1. 注册 Payload 类型
		PayloadTypeRegistry.playC2S().register(VehicleSyncPayload.ID, VehicleSyncPayload.CODEC);
		PayloadTypeRegistry.playC2S().register(VehicleRidePayload.ID, VehicleRidePayload.CODEC);
		PayloadTypeRegistry.playC2S().register(VehicleSpawnPayload.ID, VehicleSpawnPayload.CODEC);

		// 2. 注册全局服务端接收器
		ServerPlayNetworking.registerGlobalReceiver(VehicleSyncPayload.ID, (payload, context) -> {
			// 必须在主线程中执行实体操作
			context.server().execute(() -> {
				Entity entity = context.player().getWorld().getEntityById(payload.entityId());
				if (entity instanceof PhysicsVehicleEntity vehicle
						&& (!vehicle.hasPassengers() || vehicle.hasPassenger(context.player()))) {
					// 同步服务端实体位置，防止服务端进行视距卸载或判定移动作弊
					entity.setPosition(payload.x(), payload.y(), payload.z());
					entity.setYaw(payload.yaw());
					VehicleSyncPayload.RiderAnchor riderAnchor = payload.riderAnchor();
					vehicle.setRiderAnchor(
							riderAnchor.x(), riderAnchor.y(), riderAnchor.z(), riderAnchor.eyePosition());
					entity.velocityModified = true; // 挂起原版速度修正预测
				}
			});
		});

		ServerPlayNetworking.registerGlobalReceiver(VehicleRidePayload.ID, (payload, context) ->
				context.server().execute(() -> {
					ServerPlayerEntity player = context.player();
					Entity entity = player.getWorld().getEntityById(payload.entityId());
					if (!(entity instanceof PhysicsVehicleEntity vehicle)) {
						return;
					}

					if (payload.mount()) {
						if (!player.hasVehicle() && !vehicle.hasPassengers()
								&& player.squaredDistanceTo(vehicle) <= 36.0) {
							player.startRiding(vehicle);
						}
					} else if (player.getVehicle() == vehicle) {
						player.stopRiding();
					}
				}));

		ServerPlayNetworking.registerGlobalReceiver(VehicleSpawnPayload.ID, (payload, context) ->
				context.server().execute(() -> spawnVehicle(context.player(), payload)));
	}

	private static void spawnVehicle(ServerPlayerEntity player, VehicleSpawnPayload payload) {
		String rootName = payload.vehicleName().trim();
		String pcFile = payload.pcFileName().trim();
		if (!isSafeAssetName(rootName, 128) || (!pcFile.isEmpty() && !isSafeAssetName(pcFile, 256))) {
			player.sendMessage(Text.literal("Invalid vehicle or PC file name"), false);
			return;
		}

		PhysicsVehicleEntity vehicle = new PhysicsVehicleEntity(PHYSICS_VEHICLE_ENTITY, player.getWorld());
		vehicle.setSetupConfig(rootName, pcFile);
		vehicle.refreshPositionAndAngles(
				player.getX(), player.getY() + 1, player.getZ(), player.getYaw(), player.getPitch());
		player.getWorld().spawnEntity(vehicle);
		String configMessage = pcFile.isEmpty() ? "default parts" : "Config: " + pcFile;
		player.sendMessage(Text.literal("🚗 Vehicle spawned: " + rootName + " (" + configMessage + ")"), false);
	}

	private static boolean isSafeAssetName(String value, int maxLength) {
		if (value.isEmpty() || value.length() > maxLength
				|| value.contains("/") || value.contains("\\") || value.contains("..")) {
			return false;
		}
		for (int i = 0; i < value.length(); i++) {
			if (Character.isISOControl(value.charAt(i))) {
				return false;
			}
		}
		return true;
	}
}
