package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Applies cross-part named patches and JBeam numeric modifiers before graph compilation. */
final class PowertrainSpecNormalizer {
    private PowertrainSpecNormalizer() {
    }

    static List<DeviceSpec> normalize(List<DeviceSpec> rawSpecs) {
        List<DeviceSpec> result = new ArrayList<>();
        Map<String, Integer> byName = new HashMap<>();
        Map<String, List<ValueModifier>> pendingPatches = new HashMap<>();
        for (DeviceSpec spec : rawSpecs) {
            if (spec instanceof DevicePatchSpec patch) {
                Integer existing = byName.get(patch.name());
                if (existing != null) {
                    result.set(existing, applyModifiers(result.get(existing), patch.valueModifiers()));
                } else {
                    pendingPatches.computeIfAbsent(patch.name(), ignored -> new ArrayList<>())
                            .addAll(patch.valueModifiers());
                }
                continue;
            }
            Integer existing = byName.get(spec.name());
            if (existing == null) {
                byName.put(spec.name(), result.size());
                DeviceSpec normalized = applyModifiers(spec, spec.valueModifiers());
                List<ValueModifier> pending = pendingPatches.remove(spec.name());
                if (pending != null) normalized = applyModifiers(normalized, pending);
                result.add(normalized);
            } else if (!spec.valueModifiers().isEmpty()
                    && result.get(existing).type().equalsIgnoreCase(spec.type())) {
                result.set(existing, applyModifiers(result.get(existing), spec.valueModifiers()));
            } else {
                result.add(spec); // let topology validation report a true duplicate
            }
        }
        return List.copyOf(result);
    }

    private static DeviceSpec applyModifiers(DeviceSpec spec, List<ValueModifier> modifiers) {
        DeviceSpec current = spec;
        for (ValueModifier modifier : modifiers) current = applyModifier(current, modifier);
        return current;
    }

    private static DeviceSpec applyModifier(DeviceSpec spec, ValueModifier modifier) {
        String key = modifier.targetKey();
        if (spec instanceof CombustionEngineSpec e) {
            double inertia = e.inertia(), idle = e.idleRPM(), max = e.maxRPM(), friction = e.friction();
            double dynamic = e.dynamicFriction(), brake = e.engineBrakeTorque();
            double starterTorque = e.starterTorque(), starterMaxRPM = e.starterMaxRPM();
            double crankingRPM = e.crankingRPM(), revLimiterRPM = e.revLimiterRPM();
            double revLimiterCutTime = e.revLimiterCutTime(), revLimiterMaxRPMDrop = e.revLimiterMaxRPMDrop();
            double idleControllerP = e.idleControllerP(), maxIdleThrottle = e.maxIdleThrottle();
            List<TorquePoint> curve = e.torqueCurve();
            switch (key) {
                case "inertia" -> inertia = modify(inertia, modifier);
                case "idleRPM" -> idle = modify(idle, modifier);
                case "maxRPM" -> max = modify(max, modifier);
                case "friction" -> friction = modify(friction, modifier);
                case "dynamicFriction" -> dynamic = modify(dynamic, modifier);
                case "engineBrakeTorque" -> brake = modify(brake, modifier);
                case "starterTorque" -> starterTorque = modify(starterTorque, modifier);
                case "starterMaxRPM", "starterRPM", "startRPM" -> starterMaxRPM = modify(starterMaxRPM, modifier);
                case "crankingRPM" -> crankingRPM = modify(crankingRPM, modifier);
                case "revLimiterRPM" -> revLimiterRPM = modify(revLimiterRPM, modifier);
                case "revLimiterCutTime" -> revLimiterCutTime = modify(revLimiterCutTime, modifier);
                case "revLimiterMaxRPMDrop", "revLimiterRPMChange" ->
                        revLimiterMaxRPMDrop = modify(revLimiterMaxRPMDrop, modifier);
                case "idleControllerP" -> idleControllerP = modify(idleControllerP, modifier);
                case "maxIdleThrottle" -> maxIdleThrottle = modify(maxIdleThrottle, modifier);
                case "torque" -> {
                    List<TorquePoint> changed = new ArrayList<>(curve.size());
                    for (TorquePoint point : curve) {
                        changed.add(new TorquePoint(point.rpm(), modify(point.torque(), modifier)));
                    }
                    curve = changed;
                }
                default -> { return spec; }
            }
            return new CombustionEngineSpec(e.type(), e.name(), e.inputName(), e.inputIndex(), inertia, idle, max,
                    friction, dynamic, brake, curve, e.torqueReactionNodes(), List.of(),
                    starterTorque, starterMaxRPM, crankingRPM, revLimiterRPM, e.revLimiterType(),
                    revLimiterCutTime, revLimiterMaxRPMDrop, idleControllerP, maxIdleThrottle);
        }
        if (spec instanceof FrictionClutchSpec c) {
            double capacity = c.lockTorque(), spring = c.lockSpring(), coefficient = c.lockSpringCoef();
            double damping = c.lockDampRatio(), freePlay = c.clutchFreePlay(), stiffness = c.clutchStiffness();
            switch (key) {
                case "lockTorque" -> capacity = modify(capacity, modifier);
                case "lockSpring" -> spring = modify(spring, modifier);
                case "lockSpringCoef" -> coefficient = modify(coefficient, modifier);
                case "lockDampRatio" -> damping = modify(damping, modifier);
                case "clutchFreePlay" -> freePlay = modify(freePlay, modifier);
                case "clutchStiffness" -> stiffness = modify(stiffness, modifier);
                default -> { return spec; }
            }
            return new FrictionClutchSpec(c.type(), c.name(), c.inputName(), c.inputIndex(), capacity, spring,
                    coefficient, damping, freePlay, stiffness, List.of());
        }
        if (spec instanceof GearboxSpec g) {
            double friction = g.friction(), dynamic = g.dynamicFriction(), loss = g.torqueLossCoef();
            double shiftTime = g.shiftTime();
            List<Double> ratios = g.gearRatios();
            switch (key) {
                case "friction" -> friction = modify(friction, modifier);
                case "dynamicFriction" -> dynamic = modify(dynamic, modifier);
                case "torqueLossCoef" -> loss = modify(loss, modifier);
                case "gearChangeTime", "maxGearChangeTime", "dctClutchTime" ->
                        shiftTime = modify(shiftTime, modifier);
                case "gearRatios" -> {
                    List<Double> changed = new ArrayList<>(ratios.size());
                    for (double ratio : ratios) changed.add(modify(ratio, modifier));
                    ratios = changed;
                }
                default -> { return spec; }
            }
            return new GearboxSpec(g.type(), g.name(), g.inputName(), g.inputIndex(), ratios, g.fixedFirstGear(),
                    friction, dynamic, loss, List.of(), shiftTime);
        }
        if (spec instanceof ShaftSpec s) {
            RigidValues values = modifyRigid(s.gearRatio(), s.friction(), s.dynamicFriction(),
                    s.torqueLossCoef(), modifier);
            if (values == null) return spec;
            return new ShaftSpec(s.type(), s.name(), s.inputName(), s.inputIndex(), values.ratio,
                    s.connectedWheel(), values.friction, values.dynamicFriction, values.torqueLossCoef,
                    s.torqueReactionNodes(), s.outputPortOverride(), List.of());
        }
        if (spec instanceof TorsionReactorSpec r) {
            RigidValues values = modifyRigid(r.gearRatio(), r.friction(), r.dynamicFriction(),
                    r.torqueLossCoef(), modifier);
            if (values == null) return spec;
            return new TorsionReactorSpec(r.type(), r.name(), r.inputName(), r.inputIndex(), values.ratio,
                    r.connectedWheel(), values.friction, values.dynamicFriction, values.torqueLossCoef,
                    r.torqueReactionNodes(), r.outputPortOverride(), List.of());
        }
        if (spec instanceof DifferentialSpec d) {
            RigidValues values = modifyRigid(d.gearRatio(), d.friction(), d.dynamicFriction(),
                    d.torqueLossCoef(), modifier);
            double split = d.diffTorqueSplit();
            if ("diffTorqueSplit".equals(key)) split = modify(split, modifier);
            else if (values == null) return spec;
            if (values == null) values = new RigidValues(d.gearRatio(), d.friction(), d.dynamicFriction(), d.torqueLossCoef());
            return new DifferentialSpec(d.type(), d.name(), d.inputName(), d.inputIndex(), values.ratio, split,
                    values.friction, values.dynamicFriction, values.torqueLossCoef, d.diffType(), List.of());
        }
        return spec;
    }

    private static RigidValues modifyRigid(double ratio, double friction, double dynamicFriction,
                                           double torqueLossCoef, ValueModifier modifier) {
        switch (modifier.targetKey()) {
            case "gearRatio" -> ratio = modify(ratio, modifier);
            case "friction" -> friction = modify(friction, modifier);
            case "dynamicFriction" -> dynamicFriction = modify(dynamicFriction, modifier);
            case "torqueLossCoef" -> torqueLossCoef = modify(torqueLossCoef, modifier);
            default -> { return null; }
        }
        return new RigidValues(ratio, friction, dynamicFriction, torqueLossCoef);
    }

    private static double modify(double base, ValueModifier modifier) {
        return switch (modifier.operation()) {
            case '*' -> base * modifier.value();
            case '+' -> base + modifier.value();
            case '-' -> base - modifier.value();
            case '/' -> Math.abs(modifier.value()) > 1e-12 ? base / modifier.value() : base;
            case '=' -> modifier.value();
            default -> base;
        };
    }

    private record RigidValues(double ratio, double friction, double dynamicFriction, double torqueLossCoef) {
    }
}
