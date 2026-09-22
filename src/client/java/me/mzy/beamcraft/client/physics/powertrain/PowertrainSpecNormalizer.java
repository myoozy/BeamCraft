/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive Lua powertrain sources. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Applies unresolved JBeam numeric modifiers before graph compilation. */
final class PowertrainSpecNormalizer {
    private PowertrainSpecNormalizer() {
    }

    static List<DeviceSpec> normalize(List<DeviceSpec> rawSpecs) {
        List<DeviceSpec> result = new ArrayList<>();
        Map<String, Integer> byName = new HashMap<>();
        for (DeviceSpec spec : rawSpecs) {
            Integer existing = byName.get(spec.name());
            if (existing == null) {
                byName.put(spec.name(), result.size());
                DeviceSpec normalized = applyModifiers(spec, spec.valueModifiers());
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
        if (spec instanceof TorqueConverterSpec c) {
            double coupling = c.couplingAVRatio(), stall = c.stallTorqueRatio();
            double stiffness = c.converterStiffness(), diameter = c.converterDiameter();
            double limit = c.converterTorque(), extraInertia = c.additionalEngineInertia();
            double lockCapacity = c.lockupClutchTorque(), lockSpring = c.lockupClutchSpring();
            double lockDamping = c.lockupClutchDampRatio();
            switch (key) {
                case "couplingAVRatio" -> coupling = modify(coupling, modifier);
                case "stallTorqueRatio" -> stall = modify(stall, modifier);
                case "converterStiffness" -> stiffness = modify(stiffness, modifier);
                case "converterDiameter" -> diameter = modify(diameter, modifier);
                case "converterTorque" -> limit = modify(limit, modifier);
                case "additionalEngineInertia" -> extraInertia = modify(extraInertia, modifier);
                case "lockupClutchTorque" -> lockCapacity = modify(lockCapacity, modifier);
                case "lockupClutchSpring" -> lockSpring = modify(lockSpring, modifier);
                case "lockupClutchDampRatio" -> lockDamping = modify(lockDamping, modifier);
                default -> { return spec; }
            }
            return new TorqueConverterSpec(c.type(), c.name(), c.inputName(), c.inputIndex(),
                    coupling, stall, stiffness, diameter, limit, extraInertia,
                    c.lockupClutchRatioName(), lockCapacity, lockSpring, lockDamping, List.of());
        }
        if (spec instanceof DctGearboxSpec d) {
            double friction = d.friction(), dynamic = d.dynamicFriction(), loss = d.torqueLossCoef();
            double lockTorque = d.lockTorque(), lockSpring = d.lockSpring();
            double stiffness = d.clutchStiffness(), damp1 = d.lockDampRatio1(), damp2 = d.lockDampRatio2();
            double extraInertia = d.additionalEngineInertia(), shiftTime = d.shiftTime();
            List<Double> ratios = d.gearRatios();
            switch (key) {
                case "friction" -> friction = modify(friction, modifier);
                case "dynamicFriction" -> dynamic = modify(dynamic, modifier);
                case "torqueLossCoef" -> loss = modify(loss, modifier);
                case "lockTorque" -> lockTorque = modify(lockTorque, modifier);
                case "lockSpring" -> lockSpring = modify(lockSpring, modifier);
                case "clutchStiffness" -> stiffness = modify(stiffness, modifier);
                case "lockDampRatio1" -> damp1 = modify(damp1, modifier);
                case "lockDampRatio2" -> damp2 = modify(damp2, modifier);
                case "additionalEngineInertia" -> extraInertia = modify(extraInertia, modifier);
                case "dctClutchTime", "gearChangeTime", "maxGearChangeTime" ->
                        shiftTime = modify(shiftTime, modifier);
                case "gearRatios" -> {
                    List<Double> changed = new ArrayList<>(ratios.size());
                    for (double ratio : ratios) changed.add(modify(ratio, modifier));
                    ratios = changed;
                }
                default -> { return spec; }
            }
            return new DctGearboxSpec(d.type(), d.name(), d.inputName(), d.inputIndex(), ratios, false,
                    friction, dynamic, loss, lockTorque, lockSpring, stiffness, damp1, damp2,
                    extraInertia, List.of(), shiftTime);
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
        if (spec instanceof SplitShaftSpec s) {
            double ratio = s.gearRatio(), clutchRatio = s.defaultClutchRatio();
            double lockTorque = s.lockTorque(), lockSpring = s.lockSpring();
            double lockSpringCoef = s.lockSpringCoef(), lockDamp = s.lockDampRatio();
            double stiffness = s.clutchStiffness(), viscousCoef = s.viscousCoef();
            double viscousTorque = s.viscousTorque(), viscousExponent = s.viscousExponent();
            double viscousSmoothing = s.viscousSmoothing(), friction = s.friction();
            double dynamic = s.dynamicFriction(), loss = s.torqueLossCoef();
            switch (key) {
                case "gearRatio" -> ratio = modify(ratio, modifier);
                case "defaultClutchRatio" -> clutchRatio = modify(clutchRatio, modifier);
                case "lockTorque" -> lockTorque = modify(lockTorque, modifier);
                case "lockSpring" -> lockSpring = modify(lockSpring, modifier);
                case "lockSpringCoef" -> lockSpringCoef = modify(lockSpringCoef, modifier);
                case "lockDampRatio" -> lockDamp = modify(lockDamp, modifier);
                case "clutchStiffness" -> stiffness = modify(stiffness, modifier);
                case "viscousCoef" -> viscousCoef = modify(viscousCoef, modifier);
                case "viscousTorque" -> viscousTorque = modify(viscousTorque, modifier);
                case "viscousExponent" -> viscousExponent = modify(viscousExponent, modifier);
                case "viscousSmoothing" -> viscousSmoothing = modify(viscousSmoothing, modifier);
                case "friction" -> friction = modify(friction, modifier);
                case "dynamicFriction" -> dynamic = modify(dynamic, modifier);
                case "torqueLossCoef" -> loss = modify(loss, modifier);
                default -> { return spec; }
            }
            return new SplitShaftSpec(s.type(), s.name(), s.inputName(), s.inputIndex(), ratio,
                    s.primaryOutputID(), s.splitType(), s.canDisconnect(), s.isDisconnected(), clutchRatio,
                    lockTorque, lockSpring, lockSpringCoef, lockDamp, stiffness,
                    viscousCoef, viscousTorque, viscousExponent, viscousSmoothing,
                    friction, dynamic, loss, List.of());
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
            double lsdPreload = d.lsdPreload();
            double lsdLockCoef = d.lsdLockCoef();
            double lsdRevLockCoef = d.lsdRevLockCoef();
            double viscousCoef = d.viscousCoef();
            double viscousTorque = d.viscousTorque();
            double viscousExponent = d.viscousExponent();
            double viscousSmoothing = d.viscousSmoothing();
            double lockTorque = d.lockTorque();
            double lockSpring = d.lockSpring();
            double lockDampRatio = d.lockDampRatio();
            double activeLockTorque = d.activeLockTorque();
            boolean changed = values != null;
            switch (key) {
                case "diffTorqueSplit" -> { split = modify(split, modifier); changed = true; }
                case "lsdPreload" -> { lsdPreload = modify(lsdPreload, modifier); changed = true; }
                case "lsdLockCoef" -> { lsdLockCoef = modify(lsdLockCoef, modifier); changed = true; }
                case "lsdRevLockCoef" -> { lsdRevLockCoef = modify(lsdRevLockCoef, modifier); changed = true; }
                case "viscousCoef" -> { viscousCoef = modify(viscousCoef, modifier); changed = true; }
                case "viscousTorque" -> { viscousTorque = modify(viscousTorque, modifier); changed = true; }
                case "viscousExponent" -> { viscousExponent = modify(viscousExponent, modifier); changed = true; }
                case "viscousSmoothing" -> { viscousSmoothing = modify(viscousSmoothing, modifier); changed = true; }
                case "lockTorque" -> { lockTorque = modify(lockTorque, modifier); changed = true; }
                case "lockSpring" -> { lockSpring = modify(lockSpring, modifier); changed = true; }
                case "lockDampRatio" -> { lockDampRatio = modify(lockDampRatio, modifier); changed = true; }
                case "activeLockTorque" -> { activeLockTorque = modify(activeLockTorque, modifier); changed = true; }
                default -> { }
            }
            if (!changed) return spec;
            if (values == null) values = new RigidValues(d.gearRatio(), d.friction(), d.dynamicFriction(), d.torqueLossCoef());
            return new DifferentialSpec(d.type(), d.name(), d.inputName(), d.inputIndex(), values.ratio, split,
                    values.friction, values.dynamicFriction, values.torqueLossCoef,
                    d.diffType(), d.availableModes(), lsdPreload, lsdLockCoef, lsdRevLockCoef,
                    viscousCoef, viscousTorque, viscousExponent, viscousSmoothing,
                    lockTorque, lockSpring, lockDampRatio, activeLockTorque, List.of());
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
