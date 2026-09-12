package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Diagnostic (not an assertion) test: runs the real drivetrain one sub-step and
 * compares the torque the torsion reactors apply with the torque delivered to the
 * wheels.
 *
 * <p>BeamNG's reactor torque is its parent shaft's torque — for these cars the
 * driveshaft or transfer case, i.e. upstream of the final drive — while the wheels
 * receive that torque multiplied by the final drive ratio. So the reactor total should
 * come out around the final drive ratio <em>smaller</em> than the sum of the wheel
 * torques. A reactor of comparable or larger magnitude means the ratio is being
 * applied twice.
 *
 * <p>Run with {@code BEAMCRAFT_JBEAM_CORPUS=<...>/content/vehicles}.
 */
class TorsionReactorAuditTest {

    private static final float DT = 1.0f / PhysicsWorld.invPhysicsDT;

    @Test
    void compareReactorTorqueWithWheelTorque() {
        String corpus = System.getenv("BEAMCRAFT_JBEAM_CORPUS");
        Assumptions.assumeTrue(corpus != null && !corpus.isBlank());
        File corpusRoot = new File(corpus);
        Assumptions.assumeTrue(corpusRoot.isDirectory());

        audit(corpusRoot, "etk800", "846x_ttsport_plus_DCT.pc");
        audit(corpusRoot, "vivace", "vivace_S_410q_DCT.pc");
    }

    private void audit(File corpusRoot, String model, String pcFile) {
        Map<String, JsonObject> registry = new HashMap<>();
        Map<String, String> config = new HashMap<>();
        JBeamLoader.loadVehicle(corpusRoot, model, pcFile, registry, config);
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        if (!new JBeamAssembler().assembleVehicle(model, config, registry, vehicle)) {
            System.out.println("!! assembly failed for " + model);
            return;
        }

        // Spin the engine up and hold a gear so the drivetrain transmits torque.
        vehicle.powertrain.engines.engineAV[0] = 300.0f;
        vehicle.powertrain.setControls(1.0f, 0.0f);
        for (int i = 0; i < 60; i++) {
            vehicle.powertrain.solve(DT);
        }

        System.out.println();
        System.out.println("================================================================");
        System.out.println("TORSION REACTOR: " + model + "   reactors="
                + vehicle.powertrain.reactions.reactorGain.length
                + "  wheels=" + vehicle.wheels.count);
        System.out.println("================================================================");

        double wheelTotal = 0.0;
        for (int w = 0; w < vehicle.wheels.count; w++) {
            double moment = hubMoment(vehicle, w);
            wheelTotal += moment;
            System.out.printf(Locale.ROOT, "  wheel %-3s hub torque = %10.1f Nm%n",
                    vehicle.wheels.name[w], moment);
        }

        double reactorTotal = 0.0;
        var reactions = vehicle.powertrain.reactions;
        for (int r = 0; r < reactions.reactorGain.length; r++) {
            int start = reactions.reactorNodeStart[r];
            int count = reactions.reactorNodeCount[r];
            int n1 = reactions.reactionNodes[start];
            int n2 = reactions.reactionNodes[start + 1];
            double ax = vehicle.nodes.posX[n2] - vehicle.nodes.posX[n1];
            double ay = vehicle.nodes.posY[n2] - vehicle.nodes.posY[n1];
            double az = vehicle.nodes.posZ[n2] - vehicle.nodes.posZ[n1];
            double len = Math.sqrt(ax * ax + ay * ay + az * az);
            if (len < 1.0e-9) continue;
            ax /= len; ay /= len; az /= len;

            double mx = 0.0, my = 0.0, mz = 0.0;
            for (int i = 0; i < count; i++) {
                int node = reactions.reactionNodes[start + i];
                double fx = vehicle.nodes.forceX[node];
                double fy = vehicle.nodes.forceY[node];
                double fz = vehicle.nodes.forceZ[node];
                double px = vehicle.nodes.posX[node];
                double py = vehicle.nodes.posY[node];
                double pz = vehicle.nodes.posZ[node];
                mx += py * fz - pz * fy;
                my += pz * fx - px * fz;
                mz += px * fy - py * fx;
            }
            double aboutAxis = mx * ax + my * ay + mz * az;
            reactorTotal += aboutAxis;
            StringBuilder names = new StringBuilder();
            for (int i = 0; i < count; i++) {
                names.append(vehicle.nodes.names[reactions.reactionNodes[start + i]]).append(' ');
            }
            System.out.printf(Locale.ROOT,
                    "  reactor %d gain=%.4f nodes=[%s] torque about axis = %10.1f Nm%n",
                    r, reactions.reactorGain[r], names.toString().trim(), aboutAxis);
        }

        System.out.printf(Locale.ROOT, "  TOTAL wheel = %.1f Nm   TOTAL reactor = %.1f Nm   reactor/wheel = %.4f%n",
                wheelTotal, reactorTotal, wheelTotal != 0.0 ? reactorTotal / wheelTotal : Double.NaN);
    }

    /** Axle-axis moment of the forces sitting on one wheel's hub ring and axle nodes. */
    private static double hubMoment(SoftBodyVehicle vehicle, int wheel) {
        int base = wheel * WheelContainer.MAX_RAYS;
        int n1 = vehicle.wheels.node1[wheel];
        int n2 = vehicle.wheels.node2[wheel];
        double ax = vehicle.nodes.posX[n2] - vehicle.nodes.posX[n1];
        double ay = vehicle.nodes.posY[n2] - vehicle.nodes.posY[n1];
        double az = vehicle.nodes.posZ[n2] - vehicle.nodes.posZ[n1];
        double len = Math.sqrt(ax * ax + ay * ay + az * az);
        if (len < 1.0e-9) return 0.0;
        ax /= len; ay /= len; az /= len;

        double mx = 0.0, my = 0.0, mz = 0.0;
        for (int i = 0; i < vehicle.wheels.numRays[wheel]; i++) {
            for (int node : new int[]{vehicle.wheels.hubInnerNodes[base + i],
                    vehicle.wheels.hubOuterNodes[base + i]}) {
                if (node < 0) continue;
                double fx = vehicle.nodes.forceX[node];
                double fy = vehicle.nodes.forceY[node];
                double fz = vehicle.nodes.forceZ[node];
                double px = vehicle.nodes.posX[node] - vehicle.nodes.posX[n2];
                double py = vehicle.nodes.posY[node] - vehicle.nodes.posY[n2];
                double pz = vehicle.nodes.posZ[node] - vehicle.nodes.posZ[n2];
                mx += py * fz - pz * fy;
                my += pz * fx - px * fz;
                mz += px * fy - py * fx;
            }
        }
        // Positive here is the torque the hub received; the audit only needs the ratio.
        return -(mx * ax + my * ay + mz * az);
    }
}
