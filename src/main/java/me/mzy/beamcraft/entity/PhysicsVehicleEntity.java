package me.mzy.beamcraft.entity;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.physics.SoftBodyVehicle;
import me.mzy.beamcraft.physics.JBeamAssembler;
import net.minecraft.entity.Entity;
import net.minecraft.entity.EntityType;
import net.minecraft.entity.MovementType;
import net.minecraft.nbt.NbtCompound;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.World;

public class PhysicsVehicleEntity extends Entity {

    public SoftBodyVehicle softBody;
    // 必须要保存这两个参数，否则退出重进游戏车就没了
    private String rootPartName = "";
    private String pcFileName = "";

    // 【必须保留的标准构造函数】Minecraft 底层生成实体、读取存档时调用的就是这个！
    public PhysicsVehicleEntity(EntityType<?> type, World world) {
        super(type, world);
        // 注意：这里不要直接 new SoftBodyVehicle，等初始化时再 new
    }

    // 【你自己的初始化方法】当你在游戏里用代码/物品生成车时，手动调用这个
    public void initializeVehicle(String rootPartName, String pcFileName) {
        this.rootPartName = rootPartName;
        this.pcFileName = pcFileName;

        this.softBody = new SoftBodyVehicle(this);

        assemble(rootPartName, pcFileName);
    }

    // 提供一个供外部调用的装配接口
    private void assemble(String rootPartName, String pcFileName) {
        if (!this.getWorld().isClient) {
            // 确保绝不重复添加SoftBody到PhysicsWorld
            BeamCraft.PHYSICS_WORLD.removeVehicle(this.softBody);

            // 为这辆车创建一次性的临时图纸库和配置表
            // Parts Registry: The key is the part name (e.g., “pickup_frame”), and the value is the corresponding JSON object.
            java.util.Map<String, com.google.gson.JsonObject> localRegistry = new java.util.HashMap<>();
            // Player configuration: “Key” is the slot name, and “Value” is the name of the selected part.
            java.util.Map<String, String> localConfig = new java.util.HashMap<>();

            // 2. 加载 ZIP 数据到这个临时库里
            String[] zipFiles = {"/Debug/common.zip", "/Debug/pickup.zip"};
            me.mzy.beamcraft.physics.JBeamLoader.loadJBeamByPC(
                    zipFiles, pcFileName, localRegistry, localConfig
            );

            JBeamAssembler assembler = new JBeamAssembler();
            assembler.assembleVehicle(rootPartName, localConfig, localRegistry, this.softBody);

            // 登记到物理世界
            BeamCraft.PHYSICS_WORLD.addVehicle(this.softBody);

            BeamCraft.LOGGER.info("Physics Vehicle assembled: nodes = " + softBody.nodes.count + " | beams = " + softBody.beams.count + " | triangles = " + softBody.triangles.count + " | torsion bars = " + softBody.torsionbars.count);

            // 方法结束时，localRegistry 会被自动销毁，内存永不爆炸！
        }
    }

    // 写入数据到 NBT（存盘）
    @Override
    protected void writeCustomDataToNbt(NbtCompound nbt) {
        nbt.putString("RootPartName", this.rootPartName);
        nbt.putString("PcFileName", this.pcFileName);
    }

    // 从 NBT 读取数据（读盘）
    @Override
    protected void readCustomDataFromNbt(NbtCompound nbt) {
        this.rootPartName = nbt.getString("RootPartName");
        this.pcFileName = nbt.getString("PcFileName");

        // 这里的逻辑：如果名字不为空，说明是旧车读档，自动初始化物理模型
        if (!this.rootPartName.isEmpty() && !this.pcFileName.isEmpty()) {
            this.initializeVehicle(this.rootPartName, this.pcFileName);
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
        if (this.softBody != null) {
            softBody.nodes.rotateNodes(0, pitch, 0);
        }
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
    protected void initDataTracker(net.minecraft.entity.data.DataTracker.Builder builder) {}
}