package me.mzy.beamcraft.client.physics;
import java.util.ArrayList;
import java.util.List;

public class CouplerRegistry {
    public static class CouplerDef {
        public String nodeName;
        public String tag;
        public String couplerTag;
        public double startRadius;
        public double latchSpeed;
        public double strength;
        public boolean weld;
        public double lockRadius;

        public CouplerDef(String nodeName, String tag, String couplerTag, double startRadius, double latchSpeed, double strength, boolean weld, double lockRadius) {
            this.nodeName = nodeName;
            this.tag = tag;
            this.couplerTag = couplerTag;
            this.startRadius = startRadius;
            this.latchSpeed = latchSpeed;
            this.strength = strength;
            this.weld = weld;
            this.lockRadius = lockRadius;
        }
    }

    /** Spawn-time node pair declared by BeamNG's modern advancedCouplerControl. */
    public static class DirectCouplerDef {
        public final String controllerName;
        public final String node1;
        public final String node2;
        public final double startRadius;
        public final double latchSpeed;
        public final double strength;
        public final double lockRadius;
        public final String breakGroup;

        public DirectCouplerDef(String controllerName, String node1, String node2,
                                double startRadius, double latchSpeed, double strength,
                                double lockRadius, String breakGroup) {
            this.controllerName = controllerName;
            this.node1 = node1;
            this.node2 = node2;
            this.startRadius = startRadius;
            this.latchSpeed = latchSpeed;
            this.strength = strength;
            this.lockRadius = lockRadius;
            this.breakGroup = breakGroup;
        }
    }

    public final List<CouplerDef> definitions = new ArrayList<>();
    public final List<DirectCouplerDef> directDefinitions = new ArrayList<>();

    public void register(String nodeName, String tag, String couplerTag, double startRadius, double latchSpeed, double strength, boolean weld, double lockRadius) {
        if ((tag != null && !tag.isEmpty()) || (couplerTag != null && !couplerTag.isEmpty())) {
            definitions.add(new CouplerDef(nodeName, tag, couplerTag, startRadius, latchSpeed, strength, weld, lockRadius));
        }
    }

    public void registerDirect(String controllerName, String node1, String node2,
                               double startRadius, double latchSpeed, double strength,
                               double lockRadius, String breakGroup) {
        if (node1 == null || node1.isBlank() || node2 == null || node2.isBlank()
                || !Double.isFinite(startRadius) || startRadius <= 0.0) {
            return;
        }
        directDefinitions.add(new DirectCouplerDef(
                controllerName, node1, node2, startRadius, latchSpeed,
                strength, lockRadius, breakGroup));
    }
}
