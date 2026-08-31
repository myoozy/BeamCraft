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

    /** Friction clutches (same unit ordering as {@link #engines}). */
    public final FrictionClutchContainer clutches = new FrictionClutchContainer();

    /** Rigid driven-wheel paths out of each unit. */
    public final DrivenWheelPathContainer wheelPaths = new DrivenWheelPathContainer();

    /** Crank-reaction and torsion-reactor node ranges. */
    public final TorqueReactionContainer reactions = new TorqueReactionContainer();

    /** Compiled gearbox devices (build-time diagnostics). */
    public final GearboxContainer gearboxes = new GearboxContainer();

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
        clutches.clear();
        wheelPaths.clear();
        reactions.clear();
        gearboxes.clear();
        shafts.clear();
        differentials.clear();
        torsionReactors.clear();
        diagnostic = "cleared";
    }
}
