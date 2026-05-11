package me.mzy.beamcraft.entity;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.physics.JBeamLoader;
import me.mzy.beamcraft.physics.SoftBodyVehicle;
import me.mzy.beamcraft.physics.JBeamAssembler;
import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.data.DataTracker;
import net.minecraft.entity.data.TrackedData;
import net.minecraft.entity.data.TrackedDataHandlerRegistry;
import net.minecraft.nbt.NbtCompound;
import net.minecraft.world.World;

import java.io.File;

public class PhysicsVehicleEntity extends Entity {
    private static final TrackedData<String> VEHICLE_TYPE = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.STRING);
    public SoftBodyVehicle softBody = null;
    // 必须要保存这两个参数，否则退出重进游戏车就没了
    private String rootPartName = "";
    private String pcFileName = "";

    // 【必须保留的标准构造函数】Minecraft 底层生成实体、读取存档时调用的就是这个！
    public PhysicsVehicleEntity(EntityType<?> type, World world) {
        super(type, world);
        // 注意：这里不要直接 new SoftBodyVehicle，等初始化时再 new
    }

    // 当在游戏里用代码/物品生成车时，手动调用这个
    public void initializeVehicle(String rootPartName, String pcFileName, File vehiclesDir) {
        this.rootPartName = rootPartName;
        this.pcFileName = pcFileName;

        this.softBody = new SoftBodyVehicle(this);
        this.softBody.flexbodies.vehicleNamespace = rootPartName; // make sure to save the name
        this.setVehicleType(rootPartName);

        assemble(rootPartName, pcFileName, vehiclesDir);
    }

    private void assemble(String rootPartName, String pcFileName, File vehiclesDir) {
        if (!this.getWorld().isClient) {
            // 确保绝不重复添加SoftBody到PhysicsWorld
            BeamCraft.PHYSICS_WORLD.removeVehicle(this.softBody);

            // Parts Registry: The key is the part name (e.g., “pickup_frame”), and the value is the corresponding JSON object.
            java.util.Map<String, com.google.gson.JsonObject> localRegistry = new java.util.HashMap<>();
            // Player configuration: “Key” is the slot name, and “Value” is the name of the selected part.
            java.util.Map<String, String> localConfig = new java.util.HashMap<>();

            // 执行加载：它会自动寻找 vehiclesDir 下的 common.zip 和 pickup.zip
            JBeamLoader.loadVehicle(vehiclesDir, rootPartName, pcFileName, localRegistry, localConfig);

            JBeamAssembler assembler = new JBeamAssembler();
            assembler.assembleVehicle(rootPartName, localConfig, localRegistry, this.softBody);

            // 登记到物理世界
            BeamCraft.PHYSICS_WORLD.addVehicle(this.softBody);

            int beamsCount = softBody.normalBeams.count + softBody.supportBeams.count + softBody.boundedBeams.count + softBody.lBeams.count + softBody.anisotropicBeams.count;
            BeamCraft.LOGGER.info("Physics Vehicle assembled: " +
                    "nodes = " + softBody.nodes.count +
                    " | beams = " + beamsCount +
                    " | triangles = " + softBody.triangles.count +
                    " | torsion bars = " + softBody.torsionbars.count
            );
        }
    }

    // 写入数据到 NBT（存盘）
    @Override
    protected void writeCustomDataToNbt(NbtCompound nbt) {
        nbt.putString("VehicleType", getVehicleType());
        nbt.putString("RootPartName", this.rootPartName);
        nbt.putString("PcFileName", this.pcFileName);
    }

    // 从 NBT 读取数据（读盘）
    @Override
    protected void readCustomDataFromNbt(NbtCompound nbt) {
        if (nbt.contains("VehicleType")) setVehicleType(nbt.getString("VehicleType"));
        this.rootPartName = nbt.getString("RootPartName");
        this.pcFileName = nbt.getString("PcFileName");

        // 这里的逻辑：如果名字不为空，说明是旧车读档，自动初始化物理模型
        if (!this.rootPartName.isEmpty() && !this.pcFileName.isEmpty()) {
            this.initializeVehicle(this.rootPartName, this.pcFileName, BeamCraft.VEHICLES_DIR);
        }
    }

    @Override
    public void setYaw(float yaw) {
        super.setYaw(yaw);
        if (this.softBody != null) {
            softBody.nodes.rotateNodes(yaw, 0, 0);
        }
    }

    @Override
    public void setPitch(float pitch) {
        super.setPitch(pitch);
        //if (this.softBody != null) {
        //    softBody.nodes.rotateNodes(0, pitch, 0);
        //}
    }

    @Override
    public void tick() {
        super.tick();
        // 物理逻辑现在由 BeamCraft.PHYSICS_WORLD 的 tick 统一调用
        // 这个实体的 tick 只需要负责把自己的逻辑包围盒更新一下，防止被 MC 刷没
    }

    @Override
    public void remove(RemovalReason reason) {
        // 先调用原生的清除逻辑
        super.remove(reason);

        // 【关键清理】：当 MC 世界要删除这个实体时，必须同时把它从我们的物理引擎中剥离！
        if (this.softBody != null && this.getWorld() != null) {
            // 这里需要获取你全局的 PhysicsWorld 实例。
            // 假设你把它存在了你 Mod 的主类 BeamCraftServer 中，或者绑定在世界(ServerWorld)上
            // 示例调用 (请根据你的实际单例/管理类修改路径)：
            BeamCraft.PHYSICS_WORLD.removeVehicle(this.softBody);

            // 将引用置空，彻底切断牵连，让 Java 的 GC 能够回收掉上百个数组
            this.softBody = null;
        }
    }

    @Override
    protected void initDataTracker(DataTracker.Builder builder) {
        // 默认初始化为空字符串
        builder.add(VEHICLE_TYPE, "");
    }

    // 设置车辆类型（仅在服务端生成车辆时调用一次）
    public void setVehicleType(String typeName) {
        this.dataTracker.set(VEHICLE_TYPE, typeName);
    }

    public String getVehicleType() {
        return this.dataTracker.get(VEHICLE_TYPE);
    }
}