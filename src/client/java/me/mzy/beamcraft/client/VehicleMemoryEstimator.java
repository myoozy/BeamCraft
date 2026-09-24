package me.mzy.beamcraft.client;

import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;

/** Conservative, allocation-time-only estimate for retained vehicle memory. */
final class VehicleMemoryEstimator {
    private static final long OBJECT_HEADER_BYTES = 16L;
    private static final long ARRAY_HEADER_BYTES = 16L;
    private static final long REFERENCE_BYTES = 8L;
    private static final long COLLECTION_ENTRY_BYTES = 24L;
    private static final long MAP_ENTRY_BYTES = 40L;
    /** Rig streams, static/output VBOs and a conservative index-buffer allowance. */
    private static final long ESTIMATED_GPU_BYTES_PER_VERTEX = 128L;
    private static final long ESTIMATED_GPU_BYTES_PER_NODE = 16L;

    private VehicleMemoryEstimator() {
    }

    static long estimateRetainedBytes(SoftBodyVehicle vehicle) {
        if (vehicle == null) {
            return 0L;
        }
        long heapBytes = estimateObject(vehicle, new IdentityHashMap<>());
        long gpuBytes = saturatingAdd(
                saturatingMultiply(vehicle.flexbodies.totalVertexCount, ESTIMATED_GPU_BYTES_PER_VERTEX),
                saturatingMultiply(vehicle.nodes.count, ESTIMATED_GPU_BYTES_PER_NODE)
        );
        return Math.max(1L, saturatingAdd(heapBytes, gpuBytes));
    }

    private static long estimateObject(Object value, IdentityHashMap<Object, Boolean> seen) {
        if (value == null || seen.put(value, Boolean.TRUE) != null) {
            return 0L;
        }

        Class<?> type = value.getClass();
        if (type == String.class) {
            return align(OBJECT_HEADER_BYTES + 8L + ((String) value).length() * 2L);
        }
        if (type.isArray()) {
            return estimateArray(value, type.getComponentType(), seen);
        }
        if (value instanceof Map<?, ?> map) {
            long bytes = align(OBJECT_HEADER_BYTES + map.size() * MAP_ENTRY_BYTES);
            for (Map.Entry<?, ?> entry : map.entrySet()) {
                bytes = saturatingAdd(bytes, estimateObject(entry.getKey(), seen));
                bytes = saturatingAdd(bytes, estimateObject(entry.getValue(), seen));
            }
            return bytes;
        }
        if (value instanceof Collection<?> collection) {
            long bytes = align(OBJECT_HEADER_BYTES + collection.size() * COLLECTION_ENTRY_BYTES);
            for (Object element : collection) {
                bytes = saturatingAdd(bytes, estimateObject(element, seen));
            }
            return bytes;
        }

        // Minecraft anchors, JDK internals and renderer wrappers are shared or
        // externally owned. Count a shallow object but never traverse into a
        // ClientWorld through the retained parent entity.
        Package objectPackage = type.getPackage();
        if (objectPackage == null || !objectPackage.getName().startsWith("me.mzy.beamcraft")) {
            return OBJECT_HEADER_BYTES;
        }

        long bytes = OBJECT_HEADER_BYTES;
        for (Class<?> current = type;
             current != null && current.getName().startsWith("me.mzy.beamcraft");
             current = current.getSuperclass()) {
            for (Field field : current.getDeclaredFields()) {
                if (Modifier.isStatic(field.getModifiers())) {
                    continue;
                }
                Class<?> fieldType = field.getType();
                if (fieldType.isPrimitive()) {
                    bytes = saturatingAdd(bytes, primitiveBytes(fieldType));
                    continue;
                }
                bytes = saturatingAdd(bytes, REFERENCE_BYTES);
                if (field.trySetAccessible()) {
                    try {
                        bytes = saturatingAdd(bytes, estimateObject(field.get(value), seen));
                    } catch (IllegalAccessException ignored) {
                        // The reference itself is already counted.
                    }
                }
            }
        }
        return align(bytes);
    }

    private static long estimateArray(
            Object array,
            Class<?> componentType,
            IdentityHashMap<Object, Boolean> seen
    ) {
        int length = Array.getLength(array);
        if (componentType.isPrimitive()) {
            return align(ARRAY_HEADER_BYTES + saturatingMultiply(length, primitiveBytes(componentType)));
        }
        long bytes = align(ARRAY_HEADER_BYTES + saturatingMultiply(length, REFERENCE_BYTES));
        for (int index = 0; index < length; index++) {
            bytes = saturatingAdd(bytes, estimateObject(Array.get(array, index), seen));
        }
        return bytes;
    }

    private static int primitiveBytes(Class<?> type) {
        if (type == boolean.class || type == byte.class) return 1;
        if (type == char.class || type == short.class) return 2;
        if (type == int.class || type == float.class) return 4;
        return 8;
    }

    private static long align(long bytes) {
        if (bytes >= Long.MAX_VALUE - 7L) return Long.MAX_VALUE;
        return (bytes + 7L) & ~7L;
    }

    private static long saturatingMultiply(long left, long right) {
        if (left <= 0L || right <= 0L) return 0L;
        if (left > Long.MAX_VALUE / right) return Long.MAX_VALUE;
        return left * right;
    }

    private static long saturatingAdd(long left, long right) {
        if (left >= Long.MAX_VALUE - right) return Long.MAX_VALUE;
        return left + right;
    }
}
