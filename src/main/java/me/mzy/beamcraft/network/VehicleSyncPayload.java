package me.mzy.beamcraft.network;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.codec.PacketCodecs;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;

public record VehicleSyncPayload(int entityId, double x, double y, double z, float yaw) implements CustomPayload {

    public static final CustomPayload.Id<VehicleSyncPayload> ID = new CustomPayload.Id<>(Identifier.of("beamcraft", "vehicle_sync"));

    public static final PacketCodec<RegistryByteBuf, VehicleSyncPayload> CODEC = PacketCodec.tuple(
            PacketCodecs.INTEGER, VehicleSyncPayload::entityId,
            PacketCodecs.DOUBLE, VehicleSyncPayload::x,
            PacketCodecs.DOUBLE, VehicleSyncPayload::y,
            PacketCodecs.DOUBLE, VehicleSyncPayload::z,
            PacketCodecs.FLOAT, VehicleSyncPayload::yaw,
            VehicleSyncPayload::new
    );

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}