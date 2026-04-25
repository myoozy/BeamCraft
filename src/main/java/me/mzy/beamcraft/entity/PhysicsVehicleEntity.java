package me.mzy.beamcraft.entity;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.physics.SoftBodyVehicle;
import me.mzy.beamcraft.physics.JBeamAssembler;
import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityType;
import net.minecraft.nbt.NbtCompound;
import net.minecraft.world.World;

public class PhysicsVehicleEntity extends Entity {

    public final SoftBodyVehicle softBody;
    private boolean isAssembled = false; // 确保只组装一次

    public PhysicsVehicleEntity(EntityType<?> type, World world) {
        super(type, world);
        this.softBody = new SoftBodyVehicle(this);
    }

    // 提供一个供外部调用的装配接口
    public void assemble(String rootPartName, String pcFileName) {
        if (!isAssembled && !this.getWorld().isClient) {

            // 1. 彻底抛弃 BeamCraft.PART_REGISTRY！
            // 为这辆车创建一次性的临时图纸库和配置表
            java.util.Map<String, com.google.gson.JsonObject> localRegistry = new java.util.HashMap<>();
            java.util.Map<String, String> localConfig = new java.util.HashMap<>();

            // 2. 加载 ZIP 数据到这个临时库里
            String[] zipFiles = {"/Debug/common.zip", "/Debug/pickup.zip"};
            me.mzy.beamcraft.physics.JBeamLoader.loadJBeamByPC(
                    zipFiles, pcFileName, localRegistry, localConfig
            );

            // 3. 将临时的图纸库和配置表交给包工头
            JBeamAssembler assembler = new JBeamAssembler();
            assembler.assembleVehicle(rootPartName, localConfig, localRegistry, this.softBody);

            // 4. 调用你发明的绝妙方法：生成后一次性批量旋转！
            this.softBody.rotateNodes(this.getYaw(), this.getPitch(), 0.0f);

            // 5. 登记到物理世界
            BeamCraft.PHYSICS_WORLD.addVehicle(this.softBody);
            this.isAssembled = true;

            BeamCraft.LOGGER.info("Physics Vehicle assembled: nodes = " + softBody.nodes.count + " | beams = " + softBody.beams.count + " | triangles = " + softBody.triangles.count + " | torsion bars = " + softBody.torsionbars.count);

            // 方法结束时，localRegistry 会被自动销毁，内存永不爆炸！
        }
    }

    @Override
    public void tick() {
        super.tick();
        // 物理逻辑现在由 BeamCraft.PHYSICS_WORLD 的 tick 统一调用
        // 这个实体的 tick 只需要负责把自己的逻辑包围盒更新一下，防止被 MC 刷没
    }

    @Override
    protected void initDataTracker(net.minecraft.entity.data.DataTracker.Builder builder) {}
    @Override
    protected void readCustomDataFromNbt(NbtCompound nbt) {}
    @Override
    protected void writeCustomDataToNbt(NbtCompound nbt) {}
}