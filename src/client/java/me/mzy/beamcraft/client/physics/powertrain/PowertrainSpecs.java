/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
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
            permits CombustionEngineSpec, ElectricMotorSpec, ClutchlikeSpec, GearSelectableSpec,
                    ShaftSpec, SplitShaftSpec, TorsionReactorSpec, DifferentialSpec,
                    TurbochargerSpec, SuperchargerSpec, UnsupportedConfig {
        String type();
        String name();
        String inputName();
        int inputIndex();
        List<ValueModifier> valueModifiers();
    }

    /** A valid compliant boundary immediately downstream of a combustion engine. */
    public sealed interface ClutchlikeSpec extends DeviceSpec
            permits FrictionClutchSpec, TorqueConverterSpec, DctGearboxSpec {
    }

    /** A device that owns selectable transmission ratios. */
    public sealed interface GearSelectableSpec extends DeviceSpec
            permits GearboxSpec, DctGearboxSpec {
        List<Double> gearRatios();
        boolean fixedFirstGear();
        double friction();
        double dynamicFriction();
        double torqueLossCoef();
        double shiftTime();

        default double firstPositiveGearRatio() {
            for (double ratio : gearRatios()) {
                if (ratio > 0.0) return ratio;
            }
            return 0.0;
        }
    }

    /**
     * BeamNG 的值修改器：配置对象里以 {@code $*} / {@code $+} / {@code $-} / {@code $/}
     * （以及 {@code $=}）前缀开头的键，表示对某个字段做乘/加/减/除/赋值的修改。
     * 由于基准值可能来自父 part，构建期解析只把它原样记录，不执行运算。
     */
    public record ValueModifier(String targetKey, char operation, double value) {}

    /** 扭矩曲线上的一个采样点（rpm → 扭矩 N·m）。 */
    public record TorquePoint(double rpm, double torque) {}

    /** Sparse turbo-shaft RPM to compressor pressure (PSI) sample. */
    public record TurboPressurePoint(double turboRPM, double pressurePSI) {}

    /** Engine RPM to compressor efficiency and exhaust-drive factor sample. */
    public record TurboEnginePoint(double engineRPM, double efficiency, double exhaustFactor) {}

    /** Throttle percentage to available supercharger boost fraction. */
    public record SuperchargerBoostPoint(double throttlePercent, double factor) {}

    /**
     * A turbocharger attached by name to a combustion engine. It is build-time metadata,
     * not a node in the rotational powertrain tree.
     */
    public record TurbochargerSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            List<ValueModifier> valueModifiers,
            String engineName,
            List<TurboPressurePoint> pressureCurve,
            List<TurboEnginePoint> engineCurve,
            double inertia,
            double wastegateStartPSI,
            double wastegateLimitPSI,
            double maxExhaustPower,
            double backPressureCoef,
            double frictionCoef,
            double pressureRatePSI,
            double wastegatePCoef,
            double wastegateICoef,
            double wastegateDCoef,
            boolean bovEnabled,
            double bovOpenThreshold,
            double bovOpenChangeThreshold
    ) implements DeviceSpec {
        public TurbochargerSpec {
            valueModifiers = List.copyOf(valueModifiers);
            pressureCurve = List.copyOf(pressureCurve);
            engineCurve = List.copyOf(engineCurve);
        }

        public TurbochargerSpec(String engineName, List<TurboPressurePoint> pressureCurve,
                                List<TurboEnginePoint> engineCurve, double inertia,
                                double wastegateStartPSI, double wastegateLimitPSI,
                                double maxExhaustPower, double backPressureCoef,
                                double frictionCoef, double pressureRatePSI,
                                double wastegatePCoef, double wastegateICoef,
                                double wastegateDCoef, boolean bovEnabled,
                                double bovOpenThreshold, double bovOpenChangeThreshold) {
            this("turbocharger", "turbocharger", null, 0, List.of(), engineName,
                    pressureCurve, engineCurve, inertia, wastegateStartPSI, wastegateLimitPSI,
                    maxExhaustPower, backPressureCoef, frictionCoef, pressureRatePSI,
                    wastegatePCoef, wastegateICoef, wastegateDCoef, bovEnabled,
                    bovOpenThreshold, bovOpenChangeThreshold);
        }

        public TurbochargerSpec(String turboName, String engineName,
                                List<TurboPressurePoint> pressureCurve,
                                List<TurboEnginePoint> engineCurve, double inertia,
                                double wastegateStartPSI, double wastegateLimitPSI,
                                double maxExhaustPower, double backPressureCoef,
                                double frictionCoef, double pressureRatePSI,
                                double wastegatePCoef, double wastegateICoef,
                                double wastegateDCoef, boolean bovEnabled,
                                double bovOpenThreshold, double bovOpenChangeThreshold) {
            this("turbocharger", turboName, null, 0, List.of(), engineName,
                    pressureCurve, engineCurve, inertia, wastegateStartPSI, wastegateLimitPSI,
                    maxExhaustPower, backPressureCoef, frictionCoef, pressureRatePSI,
                    wastegatePCoef, wastegateICoef, wastegateDCoef, bovEnabled,
                    bovOpenThreshold, bovOpenChangeThreshold);
        }
    }

    /** A belt-driven supercharger attached by name to a combustion engine. */
    public record SuperchargerSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            List<ValueModifier> valueModifiers,
            String engineName,
            String superchargerType,
            double gearRatio,
            double maxRPM,
            double pressurePSIPer1kRPM,
            double crankLossPer1kRPM,
            double pressureRatePSI,
            double clutchEngageRPM,
            double clutchEngageRange,
            double clutchDisengageRPM,
            double clutchDisengageRange,
            int lobes,
            boolean twistedLobes,
            double pulseCoefModifier,
            List<SuperchargerBoostPoint> boostController
    ) implements DeviceSpec {
        public SuperchargerSpec {
            valueModifiers = List.copyOf(valueModifiers);
            boostController = List.copyOf(boostController);
        }

        public SuperchargerSpec(String name, String engineName, String superchargerType,
                                double gearRatio, double maxRPM, double pressurePSIPer1kRPM,
                                double crankLossPer1kRPM, double pressureRatePSI,
                                double clutchEngageRPM, double clutchEngageRange,
                                double clutchDisengageRPM, double clutchDisengageRange,
                                int lobes, boolean twistedLobes, double pulseCoefModifier,
                                List<SuperchargerBoostPoint> boostController) {
            this("supercharger", name, null, 0, List.of(), engineName, superchargerType,
                    gearRatio, maxRPM, pressurePSIPer1kRPM, crankLossPer1kRPM, pressureRatePSI,
                    clutchEngageRPM, clutchEngageRange, clutchDisengageRPM,
                    clutchDisengageRange, lobes, twistedLobes, pulseCoefModifier, boostController);
        }
    }

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

    /**
     * Direct-drive electric motor. The MVP treats it as an ideal torque source whose
     * angular velocity is read from the rigid downstream wheel domain; rotor inertia,
     * losses, energy storage and regeneration deliberately remain outside this model.
     */
    public record ElectricMotorSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            List<TorquePoint> torqueCurve,
            List<String> torqueReactionNodes,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public ElectricMotorSpec {
            torqueCurve = List.copyOf(torqueCurve);
            torqueReactionNodes = List.copyOf(torqueReactionNodes);
            valueModifiers = List.copyOf(valueModifiers);
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
    ) implements ClutchlikeSpec {
        public FrictionClutchSpec {
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /** Hydrodynamic {@code torqueConverter}, including its externally commanded lock-up clutch. */
    public record TorqueConverterSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double couplingAVRatio,
            double stallTorqueRatio,
            double converterStiffness,
            double converterDiameter,
            double converterTorque,
            double additionalEngineInertia,
            String lockupClutchRatioName,
            double lockupClutchTorque,
            double lockupClutchSpring,
            double lockupClutchDampRatio,
            List<ValueModifier> valueModifiers
    ) implements ClutchlikeSpec {
        public TorqueConverterSpec {
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
    ) implements GearSelectableSpec {
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
    /** Two-output primary shaft with a disconnectable locked or viscous secondary coupling. */
    public record SplitShaftSpec(
            String type,
            String name,
            String inputName,
            int inputIndex,
            double gearRatio,
            int primaryOutputID,
            String splitType,
            boolean canDisconnect,
            boolean isDisconnected,
            double defaultClutchRatio,
            double lockTorque,
            double lockSpring,
            double lockSpringCoef,
            double lockDampRatio,
            double clutchStiffness,
            double viscousCoef,
            double viscousTorque,
            double viscousExponent,
            double viscousSmoothing,
            double friction,
            double dynamicFriction,
            double torqueLossCoef,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public SplitShaftSpec {
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

    /** One DCT node with two torque paths and no separately integrated input shafts. */
    public record DctGearboxSpec(
            String type, String name, String inputName, int inputIndex,
            List<Double> gearRatios, boolean fixedFirstGear,
            double friction, double dynamicFriction, double torqueLossCoef,
            double lockTorque, double lockSpring, double clutchStiffness,
            double lockDampRatio1, double lockDampRatio2, double additionalEngineInertia,
            List<ValueModifier> valueModifiers, double shiftTime
    ) implements ClutchlikeSpec, GearSelectableSpec {
        public DctGearboxSpec {
            gearRatios = List.copyOf(gearRatios);
            valueModifiers = List.copyOf(valueModifiers);
        }
    }

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
            List<String> availableModes,
            double lsdPreload,
            double lsdLockCoef,
            double lsdRevLockCoef,
            double viscousCoef,
            double viscousTorque,
            double viscousExponent,
            double viscousSmoothing,
            double lockTorque,
            double lockSpring,
            double lockDampRatio,
            double activeLockTorque,
            List<ValueModifier> valueModifiers
    ) implements DeviceSpec {
        public DifferentialSpec {
            diffType = diffType == null || diffType.isBlank() ? "open" : diffType;
            availableModes = availableModes == null || availableModes.isEmpty()
                    ? List.of(diffType) : List.copyOf(availableModes);
            valueModifiers = List.copyOf(valueModifiers);
        }

        /** Backwards-compatible constructor for callers that only need an open differential. */
        public DifferentialSpec(String type, String name, String inputName, int inputIndex,
                                double gearRatio, double diffTorqueSplit, double friction,
                                double dynamicFriction, double torqueLossCoef, String diffType,
                                List<ValueModifier> valueModifiers) {
            this(type, name, inputName, inputIndex, gearRatio, diffTorqueSplit,
                    friction, dynamicFriction, torqueLossCoef, diffType,
                    List.of(diffType == null || diffType.isBlank() ? "open" : diffType),
                    50.0, 0.2, 0.2,
                    5.0, 50.0, 1.0, 25.0,
                    500.0, -1.0, 0.1, -1.0, valueModifiers);
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
