#version 150

// Fragment program for BeamCraft's opaque diffuse pass. Identical to vanilla
// rendertype_entity_cutout.fsh except that the base colour comes from the
// dedicated BeamcraftDiffuse sampler when BeamcraftUseTexture != 0, multiplied
// by the BeamcraftDiffuseColor factor; otherwise a colour-only white is used
// (the deterministic fallback for materials without a usable texture). Sampler0
// is deliberately absent: nothing samples it, so it would be optimised out and
// misalign the sampler units assigned by ShaderProgram. Blending is deliberately
// not enabled: this is the opaque stage. Baked texture alpha may still reach the
// output (and the vanilla 0.1 cutout discard applies), but opacity maps and
// translucent flags are deferred to a later stage.

#moj_import <fog.glsl>

uniform sampler2D BeamcraftDiffuse;

uniform vec4 ColorModulator;
uniform vec4 BeamcraftDiffuseColor;
uniform int BeamcraftUseTexture;
uniform float FogStart;
uniform float FogEnd;
uniform vec4 FogColor;

in float vertexDistance;
in vec4 vertexColor;
in vec4 lightMapColor;
in vec4 overlayColor;
in vec2 texCoord0;

out vec4 fragColor;

void main() {
    vec4 color = BeamcraftUseTexture != 0 ? texture(BeamcraftDiffuse, texCoord0) : vec4(1.0);
    color *= BeamcraftDiffuseColor;
    if (color.a < 0.1) {
        discard;
    }
    color *= vertexColor * ColorModulator;
    color.rgb = mix(overlayColor.rgb, color.rgb, overlayColor.a);
    color *= lightMapColor;
    fragColor = linear_fog(color, vertexDistance, FogStart, FogEnd, FogColor);
}
