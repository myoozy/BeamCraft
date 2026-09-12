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
    vec3 staticOffset = staticOffsetAndVy.xyz;
    int vyNode = int(staticOffsetAndVy.w + 0.5);

    vec3 centerPosition = texelFetch(uPhysicsNodes, centerNode).xyz;
    bool usesDeformBasis = normalWeightsAndVx.w >= 0.0;

    if (usesDeformBasis) {
        vec3 vx = texelFetch(uPhysicsNodes, vxNode).xyz - centerPosition;
        vec3 vy = texelFetch(uPhysicsNodes, vyNode).xyz - centerPosition;
        vec3 basisNormal = cross(vx, vy);
        float basisLengthSquared = dot(basisNormal, basisNormal);

        if (basisLengthSquared > 1e-10) {
            basisNormal *= inversesqrt(basisLengthSquared);
        } else {
            // A collapsed node basis has no well-defined normal. Keep the
            // output finite so a damaged vehicle cannot poison later draws.
            basisNormal = vec3(0.0, 1.0, 0.0);
        }

        vec3 vz = vzNodeValue >= 0.0
                ? texelFetch(uPhysicsNodes, vzNode).xyz - centerPosition
                : basisNormal;

        tfPosition = centerPosition
                + vx * weights.x
                + vy * weights.y
                + vz * weights.z;

        vec3 reconstructedNormal = vx * normalWeights.x
                + vy * normalWeights.y
                + vz * normalWeights.z;
        float normalLengthSquared = dot(reconstructedNormal, reconstructedNormal);
        tfNormal = normalLengthSquared > 1e-10
                ? reconstructedNormal * inversesqrt(normalLengthSquared)
                : basisNormal;
    } else {
        tfPosition = centerPosition + staticOffset;
        float normalLengthSquared = dot(normalWeights, normalWeights);
        tfNormal = normalLengthSquared > 1e-10
                ? normalWeights * inversesqrt(normalLengthSquared)
                : vec3(0.0, 1.0, 0.0);
    }

    // Rasterizer discard is enabled, but core-profile vertex shaders still
    // define gl_Position to keep validation consistent across drivers.
    gl_Position = vec4(0.0);
}
