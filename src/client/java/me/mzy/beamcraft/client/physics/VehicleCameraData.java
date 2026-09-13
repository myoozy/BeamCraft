package me.mzy.beamcraft.client.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/** Parsed BeamNG camera nodes and exterior-camera metadata for one vehicle. */
public final class VehicleCameraData {
    public record InternalCamera(String type, int nodeIndex, float fov) {}
    public record ExternalCamera(float offsetX, float offsetY, float offsetZ, float distance) {}
    public record RefNodes(int ref, int back, int left, int up) {}

    private final List<InternalCamera> internal = new ArrayList<>();
    private RefNodes refNodes;
    private ExternalCamera chase;
    private ExternalCamera external;

    public void addInternal(String type, int nodeIndex, float fov) {
        internal.add(new InternalCamera(type == null ? "" : type, nodeIndex, fov));
    }

    public InternalCamera preferredInternal() {
        return internal.stream()
                .min(Comparator.comparingInt(camera -> priority(camera.type())))
                .orElse(null);
    }

    private static int priority(String type) {
        return switch (type.toLowerCase(Locale.ROOT)) {
            case "driver" -> 0;
            case "dash" -> 1;
            case "rider" -> 2;
            case "hood" -> 3;
            case "bumper" -> 4;
            default -> 5;
        };
    }

    public RefNodes refNodes() {
        return refNodes;
    }

    public void setRefNodes(RefNodes refNodes) {
        if (this.refNodes == null) {
            this.refNodes = refNodes;
        }
    }

    public ExternalCamera exteriorFallback() {
        return chase != null ? chase : external;
    }

    public void setChase(ExternalCamera chase) {
        this.chase = chase;
    }

    public void setExternal(ExternalCamera external) {
        this.external = external;
    }

    public void clear() {
        internal.clear();
        refNodes = null;
        chase = null;
        external = null;
    }
}
