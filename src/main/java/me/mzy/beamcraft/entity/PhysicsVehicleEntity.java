package me.mzy.beamcraft.entity;

import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.EntityPose;
import net.minecraft.entity.LivingEntity;
import net.minecraft.entity.Dismounting;
import net.minecraft.entity.data.DataTracker;
import net.minecraft.entity.data.TrackedData;
import net.minecraft.entity.data.TrackedDataHandlerRegistry;
import net.minecraft.nbt.NbtCompound;
import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.util.math.Box;

import java.util.ArrayList;
import java.util.List;

public class PhysicsVehicleEntity extends Entity {
    // 自动在双端同步的通道
    private static final TrackedData<String> ROOT_PART_NAME = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.STRING);
    private static final TrackedData<String> PC_FILE_NAME = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.STRING);
    private static final TrackedData<Float> RIDER_ANCHOR_X = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.FLOAT);
    private static final TrackedData<Float> RIDER_ANCHOR_Y = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.FLOAT);
    private static final TrackedData<Float> RIDER_ANCHOR_Z = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.FLOAT);
    private static final TrackedData<Boolean> RIDER_ANCHOR_IS_EYE = DataTracker.registerData(PhysicsVehicleEntity.class, TrackedDataHandlerRegistry.BOOLEAN);

    private Box visibilityBoundingBox;

    // 【标准构造函数】
    public PhysicsVehicleEntity(EntityType<?> type, World world) {
        super(type, world);
    }

    @Override
    public boolean canHit() {
        // 允许被玩家准星射线命中 (findCrosshairTarget 过滤条件 !spectator && canHit)。
        // 默认 Entity.canHit() 返回 false，导致准星永远得不到指向车辆的
        // EntityHitResult，进而无法触发右键上车交互。
        return !this.isRemoved();
    }

    @Override
    protected void initDataTracker(DataTracker.Builder builder) {
        builder.add(ROOT_PART_NAME, "");
        builder.add(PC_FILE_NAME, "");
        builder.add(RIDER_ANCHOR_X, 0.0f);
        builder.add(RIDER_ANCHOR_Y, 0.0f);
        builder.add(RIDER_ANCHOR_Z, 0.0f);
        builder.add(RIDER_ANCHOR_IS_EYE, false);
    }

    // 当在游戏里用代码/物品生成车时，服务端仅需调用此方法下发配置指令
    public void setSetupConfig(String rootPartName, String pcFileName) {
        this.dataTracker.set(ROOT_PART_NAME, rootPartName == null ? "" : rootPartName);
        this.dataTracker.set(PC_FILE_NAME, pcFileName == null ? "" : pcFileName);
    }

    public String getRootPartName() { return this.dataTracker.get(ROOT_PART_NAME); }
    public String getPcFileName() { return this.dataTracker.get(PC_FILE_NAME); }

    public void setRiderAnchor(float x, float y, float z, boolean eyePosition) {
        if (!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)) {
            return;
        }
        this.dataTracker.set(RIDER_ANCHOR_X, x);
        this.dataTracker.set(RIDER_ANCHOR_Y, y);
        this.dataTracker.set(RIDER_ANCHOR_Z, z);
        this.dataTracker.set(RIDER_ANCHOR_IS_EYE, eyePosition);
    }

    /** Client-only visibility envelope; interaction continues to use the normal entity box. */
    public void setVisibilityBoundingBox(Box box) {
        this.visibilityBoundingBox = box;
    }

    @Override
    public Box getVisibilityBoundingBox() {
        return visibilityBoundingBox != null ? visibilityBoundingBox : super.getVisibilityBoundingBox();
    }

    @Override
    public boolean shouldRender(double distance) {
        double size = getVisibilityBoundingBox().getAverageSideLength();
        if (!Double.isFinite(size) || size <= 0.0) {
            size = 1.0;
        }
        double range = Math.max(1.0, size) * 64.0 * getRenderDistanceMultiplier();
        return distance < range * range;
    }

    @Override
    public Vec3d getPassengerRidingPos(Entity passenger) {
        Vec3d anchor = this.getPos().add(
                this.dataTracker.get(RIDER_ANCHOR_X),
                this.dataTracker.get(RIDER_ANCHOR_Y),
                this.dataTracker.get(RIDER_ANCHOR_Z)
        );
        if (this.dataTracker.get(RIDER_ANCHOR_IS_EYE)) {
            anchor = anchor.subtract(0.0, passenger.getEyeHeight(passenger.getPose()), 0.0);
        }
        return anchor;
    }

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

    @Override
    public Vec3d updatePassengerForDismount(LivingEntity passenger) {
        Vec3d offset = getPassengerDismountOffset(
                this.getWidth() * MathHelper.SQUARE_ROOT_OF_TWO,
                passenger.getWidth(),
                passenger.getYaw()
        );
        double x = this.getX() + offset.x;
        double z = this.getZ() + offset.z;
        BlockPos upper = BlockPos.ofFloored(x, this.getBoundingBox().maxY, z);
        BlockPos lower = upper.down();
        List<Vec3d> candidates = new ArrayList<>();
        addDismountCandidate(candidates, upper, x, z);
        addDismountCandidate(candidates, lower, x, z);

        for (EntityPose pose : passenger.getPoses()) {
            for (Vec3d candidate : candidates) {
                if (Dismounting.canPlaceEntityAt(this.getWorld(), candidate, passenger, pose)) {
                    passenger.setPose(pose);
                    return candidate;
                }
            }
        }
        return super.updatePassengerForDismount(passenger);
    }

    private void addDismountCandidate(List<Vec3d> candidates, BlockPos pos, double x, double z) {
        double height = this.getWorld().getDismountHeight(pos);
        if (Dismounting.canDismountInBlock(height)) {
            candidates.add(new Vec3d(x, pos.getY() + height, z));
        }
    }

    @Override
    public void updateTrackedPositionAndAngles(double x, double y, double z, float yaw, float pitch, int interpolationSteps) {
        // 如果当前处于客户端世界，直接 return 舍弃服务端的插值同步要求
        if (this.getWorld().isClient) {
            return;
        }
        super.updateTrackedPositionAndAngles(x, y, z, yaw, pitch, interpolationSteps);
    }
}
