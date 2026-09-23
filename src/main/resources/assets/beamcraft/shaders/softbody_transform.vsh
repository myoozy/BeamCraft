#version 150 core

// Rig data is split into texture buffers so each stream preserves the OpenGL
// 3.2 minimum capacity of at least 65,536 render vertices.
uniform samplerBuffer uRigWeights;
uniform samplerBuffer uRigNormals;
uniform samplerBuffer uRigOffsets;
uniform samplerBuffer uRigVz;
uniform samplerBuffer uPhysicsNodes;

out vec3 tfPosition;
out vec3 tfNormal;

const float MIN_DIRECTION_LENGTH_SQUARED = 1e-10;
const float MIN_REST_BASIS_AREA = 1e-6;
const float CROSS_COLLAPSE_FADE_START = 0.05;
const float CROSS_COLLAPSE_FADE_END = 0.25;

void main() {
    int id = gl_VertexID;
    vec4 weightsAndCenter = texelFetch(uRigWeights, id);
    vec4 normalWeightsAndVx = texelFetch(uRigNormals, id);
    vec4 staticOffsetAndVy = texelFetch(uRigOffsets, id);
    float vzNodeValue = texelFetch(uRigVz, id).x;
    int vzNode = int(vzNodeValue + 0.5);

    vec3 weights = weightsAndCenter.xyz;
    int centerNode = int(weightsAndCenter.w + 0.5);
    vec3 normalWeights = normalWeightsAndVx.xyz;
    int vxNode = int(normalWeightsAndVx.w + 0.5);
    vec3 rigAux = staticOffsetAndVy.xyz;
    int vyNode = int(staticOffsetAndVy.w + 0.5);

    vec3 centerPosition = texelFetch(uPhysicsNodes, centerNode).xyz;
    bool usesDeformBasis = normalWeightsAndVx.w >= 0.0;

    if (usesDeformBasis) {
        vec3 vx = texelFetch(uPhysicsNodes, vxNode).xyz - centerPosition;
        vec3 vy = texelFetch(uPhysicsNodes, vyNode).xyz - centerPosition;
        vec3 basisNormal = cross(vx, vy);
        float basisLengthSquared = dot(basisNormal, basisNormal);
        float basisLength = sqrt(basisLengthSquared);

        if (basisLengthSquared > MIN_DIRECTION_LENGTH_SQUARED) {
            basisNormal /= basisLength;
        } else {
            basisNormal = vec3(0.0);
        }

        // A cross-derived thickness direction becomes undefined as its node
        // triangle collapses. Fade the offset relative to the rest-pose area,
        // so an inversion passes continuously through a flat surface instead
        // of launching vertices along an arbitrary world-space normal.
        float restBasisLength = max(rigAux.x, MIN_REST_BASIS_AREA);
        float crossThicknessScale = smoothstep(
                CROSS_COLLAPSE_FADE_START, CROSS_COLLAPSE_FADE_END,
                basisLength / restBasisLength);
        vec3 vz = vzNodeValue >= 0.0
                ? texelFetch(uPhysicsNodes, vzNode).xyz - centerPosition
                : basisNormal * crossThicknessScale;

        tfPosition = centerPosition
                + vx * weights.x
                + vy * weights.y
                + vz * weights.z;

        vec3 reconstructedNormal = vx * normalWeights.x
                + vy * normalWeights.y
                + vz * normalWeights.z;
        float normalLengthSquared = dot(reconstructedNormal, reconstructedNormal);
        vec3 survivingAxis = dot(vx, vx) >= dot(vy, vy) ? vx : vy;
        float survivingAxisLengthSquared = dot(survivingAxis, survivingAxis);
        vec3 finiteFallbackNormal = survivingAxisLengthSquared > MIN_DIRECTION_LENGTH_SQUARED
                ? survivingAxis * inversesqrt(survivingAxisLengthSquared)
                : vec3(0.0, 1.0, 0.0);
        tfNormal = normalLengthSquared > MIN_DIRECTION_LENGTH_SQUARED
                ? reconstructedNormal * inversesqrt(normalLengthSquared)
                : finiteFallbackNormal;
    } else {
        tfPosition = centerPosition + rigAux;
        float normalLengthSquared = dot(normalWeights, normalWeights);
        tfNormal = normalLengthSquared > MIN_DIRECTION_LENGTH_SQUARED
                ? normalWeights * inversesqrt(normalLengthSquared)
                : vec3(0.0, 1.0, 0.0);
    }

    // Rasterizer discard is enabled, but core-profile vertex shaders still
    // define gl_Position to keep validation consistent across drivers.
    gl_Position = vec4(0.0);
}
