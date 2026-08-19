package me.mzy.beamcraft.client.render;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pins the documented UV/pixel-row orientation contract for
 * {@link VehicleTextureUploader}. OpenGL 3.2 uploads memory row 0 at texture
 * coordinate {@code v = 0}. {@link me.mzy.beamcraft.texture.DecodedImage} is
 * top-row origin, and BeamCraft mesh UVs place {@code v = 0} at the top of the
 * image (the {@code DaeMeshLoader} flip against Assimp's bottom-left UV
 * convention), exactly like vanilla Minecraft's NativeImage path. So no vertical
 * flip is performed on upload. If either half of that convention ever changes,
 * these tests force the change to be deliberate.
 */
class TextureUploadOrientationTest {

    @Test
    void beamCraftConventionRequiresNoUploadFlip() {
        assertFalse(VehicleTextureUploader.requiresUploadFlip(
                /* imageTopRowOrigin= */ true,
                /* meshUvTopOrigin= */ true));
    }

    @Test
    void flipIsRequiredWhenRowsOrUvsAreNotTopOrigin() {
        assertTrue(VehicleTextureUploader.requiresUploadFlip(false, true));
        assertTrue(VehicleTextureUploader.requiresUploadFlip(true, false));
        assertTrue(VehicleTextureUploader.requiresUploadFlip(false, false));
    }
}
