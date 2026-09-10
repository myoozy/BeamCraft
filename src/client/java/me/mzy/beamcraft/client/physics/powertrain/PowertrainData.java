package me.mzy.beamcraft.client.physics.powertrain;

/**
 * Bundle of every compiled powertrain SoA container produced by {@link PowertrainCompiler}.
 * {@link PowertrainSystem} adopts one of these on each {@code finalizeSetup} and exposes
 * the runtime containers (topology, engines, clutches, wheel paths, reactions) for the
 * substep and for diagnostics.
 */
public final class PowertrainData {

    /** Device graph: what is connected to what, and at what ratio. */
    public final PowertrainTopologyContainer topology = new PowertrainTopologyContainer();

    /** Combustion engines (one row per compiled engine→clutch unit). */
    public final CombustionEngineContainer engines = new CombustionEngineContainer();

    /** Turbochargers attached to combustion engines, using the same unit ordering. */
    public final TurbochargerContainer turbochargers = new TurbochargerContainer();

    /** Friction clutches (same unit ordering as {@link #engines}). */
    public final FrictionClutchContainer clutches = new FrictionClutchContainer();

    /** Concrete clutch-like device selected for each compiled engine unit. */
    public final ClutchlikeContainer clutchlikes = new ClutchlikeContainer();

    /** Torque-converter parameters/state, indexed in the same order as engines. */
    public final TorqueConverterContainer torqueConverters = new TorqueConverterContainer();

    /** Dual-clutch gearbox parameters and state, indexed by engine unit. */
    public final DctGearboxContainer dctGearboxes = new DctGearboxContainer();

    /** Rigid driven-wheel paths out of each unit. */
    public final DrivenWheelPathContainer wheelPaths = new DrivenWheelPathContainer();

    /** Crank-reaction and torsion-reactor node ranges. */
    public final TorqueReactionContainer reactions = new TorqueReactionContainer();

    /** Compiled gearbox devices (build-time diagnostics). */
    public final GearboxContainer gearboxes = new GearboxContainer();

    /** Optional selectable high/low range box for each engine unit. */
    public final RangeBoxContainer rangeBoxes = new RangeBoxContainer();

    /** Every split-shaft coupling, grouped into contiguous ranges per engine unit. */
    public final SplitShaftContainer splitShafts = new SplitShaftContainer();

    /** Compiled shaft devices (build-time diagnostics). */
    public final ShaftContainer shafts = new ShaftContainer();

    /** Compiled differential devices (build-time diagnostics). */
    public final DifferentialContainer differentials = new DifferentialContainer();

    /** Compiled torsion-reactor devices (build-time diagnostics). */
    public final TorsionReactorContainer torsionReactors = new TorsionReactorContainer();

    /** Human-readable state string for the HUD debug overlay. */
    public String diagnostic = "not compiled";

    /** Resets every container to its empty state. */
    public void clear() {
        topology.clear();
        engines.clear();
        turbochargers.clear();
        clutches.clear();
        clutchlikes.clear();
        torqueConverters.clear();
        dctGearboxes.clear();
        wheelPaths.clear();
        reactions.clear();
        gearboxes.clear();
        rangeBoxes.clear();
        splitShafts.clear();
        shafts.clear();
        differentials.clear();
        torsionReactors.clear();
        diagnostic = "cleared";
    }
}
