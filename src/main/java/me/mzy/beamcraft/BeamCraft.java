package me.mzy.beamcraft;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.network.VehicleSyncPayload;

import net.fabricmc.api.ModInitializer;
import net.fabricmc.fabric.api.object.builder.v1.entity.FabricEntityTypeBuilder;
import net.fabricmc.loader.api.FabricLoader;
import net.minecraft.entity.EntityDimensions;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.SpawnGroup;
import net.minecraft.registry.Registries;
import net.minecraft.registry.Registry;
import net.minecraft.util.Identifier;
import net.fabricmc.fabric.api.command.v2.CommandRegistrationCallback;
import net.minecraft.server.command.CommandManager;
import net.minecraft.server.network.ServerPlayerEntity;
import com.mojang.brigadier.arguments.StringArgumentType;
import net.minecraft.text.Text;
import net.minecraft.entity.Entity;
import net.fabricmc.fabric.api.networking.v1.PayloadTypeRegistry;
import net.fabricmc.fabric.api.networking.v1.ServerPlayNetworking;

import java.io.File;

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

		// 2. 注册全局服务端接收器
		ServerPlayNetworking.registerGlobalReceiver(VehicleSyncPayload.ID, (payload, context) -> {
			// 必须在主线程中执行实体操作
			context.server().execute(() -> {
				Entity entity = context.player().getWorld().getEntityById(payload.entityId());
				if (entity != null) {
					// 同步服务端实体位置，防止服务端进行视距卸载或判定移动作弊
					entity.setPosition(payload.x(), payload.y(), payload.z());
					entity.setYaw(payload.yaw());
					entity.velocityModified = true; // 挂起原版速度修正预测
				}
			});
		});

		// /spawnvehicle <车辆名> <pc配置文件名>
		CommandRegistrationCallback.EVENT.register((dispatcher, registryAccess, environment) -> {
			dispatcher.register(CommandManager.literal("spawnvehicle")
					// 第一个参数：车辆根名字 (如 pickup)
					.then(CommandManager.argument("name", StringArgumentType.string())
							// 第二个参数：PC文件名
							.then(CommandManager.argument("pcFile", StringArgumentType.string())
									.executes(context -> {
										// 获取两个参数
										String rootName = StringArgumentType.getString(context, "name");
										String pcFile = StringArgumentType.getString(context, "pcFile");
										ServerPlayerEntity player = context.getSource().getPlayer();

										if (player != null) {
											PhysicsVehicleEntity vehicle =
													new PhysicsVehicleEntity(PHYSICS_VEHICLE_ENTITY, player.getWorld());
											vehicle.setSetupConfig(rootName, pcFile);
											// 设置坐标和视角
											vehicle.refreshPositionAndAngles(player.getX(), player.getY() + 1, player.getZ(), player.getYaw(), player.getPitch());

											player.getWorld().spawnEntity(vehicle);
											player.sendMessage(Text.literal("🚗 Vehicle spawned: " + rootName + " (Config: " + pcFile + ")"), false);
										}
										return 1;
									}))));
		});
	}
}