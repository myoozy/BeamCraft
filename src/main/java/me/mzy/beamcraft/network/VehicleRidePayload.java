package me.mzy.beamcraft.network;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.codec.PacketCodecs;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;

/** Server-authoritative request to enter or exit a BeamCraft vehicle. */
public record VehicleRidePayload(int entityId, boolean mount) implements CustomPayload {
    public static final Id<VehicleRidePayload> ID =
            new Id<>(Identifier.of("beamcraft", "vehicle_ride"));

    public static final PacketCodec<RegistryByteBuf, VehicleRidePayload> CODEC = PacketCodec.tuple(
            PacketCodecs.INTEGER, VehicleRidePayload::entityId,
            PacketCodecs.BOOL, VehicleRidePayload::mount,
            VehicleRidePayload::new
    );

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
