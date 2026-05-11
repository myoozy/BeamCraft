package me.mzy.beamcraft.entity;

import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.data.DataTracker;
import net.minecraft.entity.data.TrackedData;
import net.minecraft.entity.data.TrackedDataHandlerRegistry;
import net.minecraft.nbt.NbtCompound;
import net.minecraft.world.World;

public class PhysicsVehicleEntity extends Entity {
    // 自动在双端同步的通道
    private static final TrackedData<String> ROOT_PART_NAME = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.STRING);
    private static final TrackedData<String> PC_FILE_NAME = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.STRING);

    // 【标准构造函数】
    public PhysicsVehicleEntity(EntityType<?> type, World world) {
        super(type, world);
    }

    @Override
    protected void initDataTracker(DataTracker.Builder builder) {
        builder.add(ROOT_PART_NAME, "");
        builder.add(PC_FILE_NAME, "");
    }

    // 当在游戏里用代码/物品生成车时，服务端仅需调用此方法下发配置指令
    public void setSetupConfig(String rootPartName, String pcFileName) {
        this.dataTracker.set(ROOT_PART_NAME, rootPartName == null ? "" : rootPartName);
        this.dataTracker.set(PC_FILE_NAME, pcFileName == null ? "" : pcFileName);
    }

    public String getRootPartName() { return this.dataTracker.get(ROOT_PART_NAME); }
    public String getPcFileName() { return this.dataTracker.get(PC_FILE_NAME); }

    @Override
    protected void writeCustomDataToNbt(NbtCompound nbt) {
        nbt.putString("RootPartName", getRootPartName());
        nbt.putString("PcFileName", getPcFileName());
    }

    @Override
    protected void readCustomDataFromNbt(NbtCompound nbt) {
        if (nbt.contains("RootPartName")) this.dataTracker.set(ROOT_PART_NAME, nbt.getString("RootPartName"));
        if (nbt.contains("PcFileName")) this.dataTracker.set(PC_FILE_NAME, nbt.getString("PcFileName"));
    }

    @Override
    public void tick() {
        super.tick();
        // 仅维护基础包围盒，物理更新完全交由客户端处理
    }

    // 不再在此处直接调用 PhysicsWorld.removeVehicle
}