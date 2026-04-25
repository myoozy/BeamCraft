package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.utility.Utility;
import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import java.util.List;
import java.util.Arrays;

/**
 * Core physical world controller for beam-based vehicle simulation
 * Manages nodes, beams, collision caching and physics integration
 */
public class PhysicsWorld {
    public static final double GRAVITY = -9.81;
    public static final double SOUND_SPEED = 340;
    public static final double BLOCK_REBOUND = 0.0;
    public static final double METAL_PLASTIC_FLOW_RATE = 10.0;
    public static final double KINDA_SMALL_NUMBER = 1e-8;
    public static final double KINDA_BIG_NUMBER = 1e8;
    public static final int MAX_AABB_SIZE = 30;
    public final VoxelSnapshot voxelSnapshot = new VoxelSnapshot();
    public final DynamicAxisSweep globalSap = new DynamicAxisSweep();
    BlockPos.Mutable mutablePos = new BlockPos.Mutable();

    public final java.util.List<SoftBodyVehicle> vehicles = new java.util.concurrent.CopyOnWriteArrayList<>();

    public PhysicsWorld() {
        // Empty constructor, data will be injected by JBeam parser
    }

    public void addVehicle(SoftBodyVehicle vehicle) {
        vehicles.add(vehicle);
    }

    /**
     * Pre-update step: cache block collision around vehicle for performance
     * Generate per-part bounding box and sample Minecraft voxel data
     */
    public void preStep(World mcWorld, double dt) {
        voxelSnapshot.clear();
        // 让每辆车自己去扫描周围的方块并写入全局 snapshot
        for (SoftBodyVehicle vehicle : vehicles) {
            vehicle.updateVoxelSnapshot(mcWorld, voxelSnapshot, mutablePos, dt);
        }
    }

    /**
     * Main physics update loop, high substep count for stable simulation
     */
    public void step(double dt) {
        int subSteps = 100;
        double subDt = dt / subSteps;

        for (int s = 0; s < subSteps; s++) {
            globalSap.clear();

            //// 1. 宽阶段：每 10 步更新一次网格 (嵌套在主循环里！)
            //if (s % 10 == 0) {
            //    softBodyHashGrid.clear();
            //    for (SoftBodyVehicle vehicle : vehicles) {
            //        vehicle.updateSoftBodyGrid(softBodyHashGrid, subDt * 10.0);
            //    }
            //}

            // 2. 算力与软体碰撞 (并行安全)
            for (SoftBodyVehicle vehicle : vehicles) {
                vehicle.solveInternalForces(subDt); // 只在这里做一次预测积分！
                globalSap.insertNodes(vehicle);
            }

            // 【关键】：节点全塞进去后，构建(排序)一次
            globalSap.updateAndSort();

            for (SoftBodyVehicle vehicle : vehicles) {
                vehicle.solveVehicleCollisions(globalSap, subDt);
            }

            // 3. 应用位移与环境碰撞 (PBD硬约束)
            for (SoftBodyVehicle vehicle : vehicles) {
                vehicle.flushCollisionDeltas(); // 合并车车碰撞推力
                vehicle.solveEnvironmentCollisions(voxelSnapshot, subDt); // 纯 PBD 方块碰撞
            }
        }
    }

    public void clear() {
        vehicles.clear();
        System.out.println("🧹 Physics world data cleared");
    }
}