package me.mzy.beamcraft.client.physics.powertrain;

import java.util.List;
import java.util.Map;

/**
 * 构建期 powertrain 的 typed 配置描述（build-time specs）。
 *
 * <p>这是纯数据层：每个设备以不可变 record 表达解析结果，不引用任何运行时
 * 物理对象。所有 {@link List}/{@link Map} 字段在构造时都做 defensive copy
 * （{@link List#copyOf}/{@link Map#copyOf}），因此调用方持有的数组不会被解析器
 * 或其它代码随后修改。
 *
 * <p>优先级约定（见 {@link JBeamPowertrainParser} 的 javadoc）：
 * <ol>
 *   <li>设备命名配置对象 {@code part["<deviceName>"]}（最高，BeamNG
 *       {@code tableMergeRecursive(row, part[name])} 语义，命名对象覆盖行内配置）；</li>
 *   <li>powertrain 行尾部的行内对象 {@code {...}}；</li>
 *   <li>powertrain 数组中独立出现的 {@code {...}} 配置修改器（黑板）；</li>
 *   <li>内置默认值（最低）。</li>
 * </ol>
 *
 * <p>注意：{@code "$*key"} / {@code "$+key"} 之类的值修改器需要跨 part 的基准值，
 * 单个 part 内无法求值，因此被原样保留在 {@link DeviceSpec#valueModifiers()} 中，
 * 由后续 build 阶段按序应用，不会被静默丢弃。
 */
public final class PowertrainSpecs {
    private PowertrainSpecs() {}

    /**
     * 单个 powertrain 设备。公共拓扑字段（type/name/inputName/inputIndex）
     * 在所有实现上直接可用。
     */
    public sealed interface DeviceSpec
            permits CombustionEngineSpec, FrictionClutchSpec, GearboxSpec,
                    ShaftSpec, TorsionReactorSpec, DifferentialSpec, DevicePatchSpec, UnsupportedConfig {
        String type();
        String name();
        String inputName();
        int inputIndex();
        List<ValueModifier> valueModifiers();
    }

    /**
     * BeamNG 的值修改器：配置对象里以 {@code $*} / {@code $+} / {@code $-} / {@code $/}
     * （以及 {@code $=}）前缀开头的键，表示对某个字段做乘/加/减/除/赋值的修改。
     * 由于基准值可能来自父 part，构建期解析只把它原样记录，不执行运算。
     */
    public record ValueModifier(String targetKey, char operation, double value) {}

    /**
     * A named-device override contributed by a separate active part. BeamNG final-drive
     * parts commonly contain only {@code "differential_F": {"gearRatio": 3.9}}
     * and no {@code powertrain} table of their own.
     */
    public record DevicePatchSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public DevicePatchSpec(String name, List<ValueModifier> valueModifiers) {
            this("devicePatch", name, null, 0, valueModifiers);
        }

        public DevicePatchSpec {
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /** 扭矩曲线上的一个采样点（rpm → 扭矩 N·m）。 */
    public record TorquePoint(double rpm, double torque) {}

    /**
     * 内燃机：{@code combustionEngine}。
     *
     * <p>从本阶段起，发动机不再是"永远运行"：它拥有一个起动机
     * （{@code starterTorque}/{@code starterMaxRPM}）和一个转速限制器
     * （{@code revLimiter*}）。{@code crankingRPM} 是燃烧所需的最低转速；
     * {@code revLimiterRPM} 未给出时回退到 {@code maxRPM}。
     */
    public record CombustionEngineSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double inertia,
            double idleRPM,
            double maxRPM,
            double friction,
            double dynamicFriction,
            double engineBrakeTorque,
            List<TorquePoint> torqueCurve,
            List<String> torqueReactionNodes,
            List<ValueModifier> valueModifiers,
            double starterTorque,
            double starterMaxRPM,
            double crankingRPM,
            double revLimiterRPM,
            String revLimiterType,
            double revLimiterCutTime,
            double revLimiterMaxRPMDrop,
            double idleControllerP,
            double maxIdleThrottle
    ) implements DeviceSpec {
        public CombustionEngineSpec {
            torqueCurve = List.copyOf(torqueCurve);
            torqueReactionNodes = List.copyOf(torqueReactionNodes);
            valueModifiers = List.copyOf(valueModifiers);
        }

        /**
         * 兼容构造函数：按 engine 级默认值补全新字段。{@code revLimiterRPM} 默认取
         * {@code maxRPM}（限制器在红线处截断），起动机扭矩默认 0（编译期由扭矩峰值派生）。
         */
        public CombustionEngineSpec(
                String type,
                String name,
                String inputName,
                int inputIndex,
                double inertia,
                double idleRPM,
                double maxRPM,
                double friction,
                double dynamicFriction,
                double engineBrakeTorque,
                List<TorquePoint> torqueCurve,
                List<String> torqueReactionNodes,
                List<ValueModifier> valueModifiers) {
            this(type, name, inputName, inputIndex, inertia, idleRPM, maxRPM, friction, dynamicFriction,
                    engineBrakeTorque, torqueCurve, torqueReactionNodes, valueModifiers,
                    0.0, 400.0, 100.0, maxRPM, "time", 0.15, 300.0,
                    0.01, 0.15);
        }
    }

    /** 摩擦离合器：{@code frictionClutch}。 */
    public record FrictionClutchSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double lockTorque,
            double lockSpring,
            double lockSpringCoef,
            double lockDampRatio,
            double clutchFreePlay,
            double clutchStiffness,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public FrictionClutchSpec {
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /**
     * 齿轮变速箱：{@code manualGearbox} / {@code gearbox}，以及结构相同的
     * {@code automaticGearbox} / {@code sequentialGearbox} / {@code dctGearbox}。
     *
     * <p>{@code gearRatios} 保存原始顺序（BeamNG 约定：负数=倒挡、0=空挡、
     * 正数=前进挡）。{@link #firstPositiveGearRatio()} 返回第一个正数，即被选定的
     * "固定一挡"。{@code fixedFirstGear} 是"第一个正数为固定一档"的选择开关：
     * 配置里显式给出 {@code fixedFirstGear} 时为 true，表示该变速箱应被当作只使用
     * 第一个正挡速比的单挡箱。
     */
    public record GearboxSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            List<Double> gearRatios,
            boolean fixedFirstGear,
            double friction,
            double dynamicFriction,
            double torqueLossCoef,
            List<ValueModifier> valueModifiers,
            double shiftTime
    ) implements DeviceSpec {
        public GearboxSpec {
            gearRatios = List.copyOf(gearRatios);
            valueModifiers = List.copyOf(valueModifiers);
        }

        /** 兼容构造函数：默认换挡时间 0.25 s（解析器按类型给出更合适的默认值）。 */
        public GearboxSpec(
                String type,
                String name,
                String inputName,
                int inputIndex,
                List<Double> gearRatios,
                boolean fixedFirstGear,
                double friction,
                double dynamicFriction,
                double torqueLossCoef,
                List<ValueModifier> valueModifiers) {
            this(type, name, inputName, inputIndex, gearRatios, fixedFirstGear,
                    friction, dynamicFriction, torqueLossCoef, valueModifiers, 0.25);
        }

        /** 第一个正挡速比；没有正挡时返回 0。 */
        public double firstPositiveGearRatio() {
            for (double r : gearRatios) {
                if (r > 0.0) return r;
            }
            return 0.0;
        }
    }

    /** 传动轴：{@code shaft}。 */
    public record ShaftSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double gearRatio,
            String connectedWheel,
            double friction,
            double dynamicFriction,
            double torqueLossCoef,
            List<String> torqueReactionNodes,
            List<Integer> outputPortOverride,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public ShaftSpec {
            torqueReactionNodes = List.copyOf(torqueReactionNodes);
            outputPortOverride = List.copyOf(outputPortOverride);
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /** 扭矩反作用节点设备：{@code torsionReactor}。 */
    public record TorsionReactorSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double gearRatio,
            String connectedWheel,
            double friction,
            double dynamicFriction,
            double torqueLossCoef,
            List<String> torqueReactionNodes,
            List<Integer> outputPortOverride,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public TorsionReactorSpec {
            torqueReactionNodes = List.copyOf(torqueReactionNodes);
            outputPortOverride = List.copyOf(outputPortOverride);
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /** 差速器：{@code differential} / {@code openDifferential}。 */
    public record DifferentialSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double gearRatio,
            double diffTorqueSplit,
            double friction,
            double dynamicFriction,
            double torqueLossCoef,
            String diffType,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public DifferentialSpec {
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /**
     * 暂不支持的设备类型。行不会被静默丢弃：拓扑字段、原始配置快照和
     * 未解析的值修改器都被保留，便于后续扩展或报告。
     */
    public record UnsupportedConfig(
            String type,
            String name,
            String inputName,
            int inputIndex,
            Map<String, String> config,
            String reason,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public UnsupportedConfig {
            config = Map.copyOf(config);
            valueModifiers = List.copyOf(valueModifiers);
        }
    }
}
