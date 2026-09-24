package me.mzy.beamcraft.client.command;

import com.mojang.brigadier.arguments.StringArgumentType;
import com.mojang.brigadier.context.CommandContext;
import me.mzy.beamcraft.client.assets.VehicleCatalog;
import me.mzy.beamcraft.network.VehicleSpawnPayload;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback;
import net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.command.CommandSource;
import net.minecraft.text.Text;

/** Client-local vehicle command so suggestions can use this client's asset roots. */
public final class ClientVehicleCommands {
    private ClientVehicleCommands() {
    }

    public static void register(VehicleCatalog catalog) {
        ClientCommandRegistrationCallback.EVENT.register((dispatcher, registryAccess) -> {
            var pcArgument = ClientCommandManager.argument("pcFile", StringArgumentType.string())
                    .suggests((context, builder) -> CommandSource.suggestMatching(
                            catalog.pcFiles(StringArgumentType.getString(context, "name")), builder))
                    .executes(context -> requestSpawn(
                            context, StringArgumentType.getString(context, "pcFile")));
            var nameArgument = ClientCommandManager.argument("name", StringArgumentType.string())
                    .suggests((context, builder) ->
                            CommandSource.suggestMatching(catalog.vehicleNames(), builder))
                    .executes(context -> requestSpawn(context, ""))
                    .then(pcArgument);
            dispatcher.register(ClientCommandManager.literal("spawnvehicle").then(nameArgument));
        });
    }

    private static int requestSpawn(CommandContext<FabricClientCommandSource> context, String pcFile) {
        if (!ClientPlayNetworking.canSend(VehicleSpawnPayload.ID)) {
            context.getSource().sendError(Text.literal("The server does not support BeamCraft vehicle spawning"));
            return 0;
        }
        String vehicleName = StringArgumentType.getString(context, "name");
        ClientPlayNetworking.send(new VehicleSpawnPayload(vehicleName, pcFile));
        return 1;
    }
}
