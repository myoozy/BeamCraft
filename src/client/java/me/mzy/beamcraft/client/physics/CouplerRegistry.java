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

    public final List<CouplerDef> definitions = new ArrayList<>();

    public void register(String nodeName, String tag, String couplerTag, double startRadius, double latchSpeed, double strength, boolean weld, double lockRadius) {
        if ((tag != null && !tag.isEmpty()) || (couplerTag != null && !couplerTag.isEmpty())) {
            definitions.add(new CouplerDef(nodeName, tag, couplerTag, startRadius, latchSpeed, strength, weld, lockRadius));
        }
    }
}