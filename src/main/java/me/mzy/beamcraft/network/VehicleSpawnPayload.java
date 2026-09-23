package me.mzy.beamcraft.network;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.codec.PacketCodecs;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;

/** Client request to create a server-side vehicle anchor using locally selected assets. */
public record VehicleSpawnPayload(String vehicleName, String pcFileName) implements CustomPayload {
    public static final Id<VehicleSpawnPayload> ID =
            new Id<>(Identifier.of("beamcraft", "vehicle_spawn"));

    public static final PacketCodec<RegistryByteBuf, VehicleSpawnPayload> CODEC = PacketCodec.tuple(
            PacketCodecs.STRING, VehicleSpawnPayload::vehicleName,
            PacketCodecs.STRING, VehicleSpawnPayload::pcFileName,
            VehicleSpawnPayload::new
    );

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
