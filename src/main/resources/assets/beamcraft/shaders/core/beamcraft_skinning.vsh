#version 150

in vec3 Position;
in vec4 Color;
in vec2 UV0;
in ivec2 UV1;
in ivec2 UV2;
in vec3 Normal;

uniform samplerBuffer u_Rigging;
uniform samplerBuffer u_PhysicsNodes;

uniform mat4 ModelViewMat;
uniform mat4 ProjMat;

out vec2 texCoord0;
out vec4 vertexColor;

void main() {
    // 核心解密：无视 Minecraft 的切块，从 X 坐标里读出绝对真实的顶点 ID！
    int true_vid = int(Position.x + 0.5);

    // 使用真实 ID 去查 TBO
    vec4 p0 = texelFetch(u_Rigging, true_vid * 3 + 0);
    vec4 p1 = texelFetch(u_Rigging, true_vid * 3 + 1);
    vec4 p2 = texelFetch(u_Rigging, true_vid * 3 + 2);

    vec3 w = p0.xyz;
    int cNode = int(p0.w + 0.5);

    int vxNode = int(p1.x + 0.5);
    int vyNode = int(p1.y + 0.5);
    bool useCross = bool(p1.z > 0.5); // 显式转换防止 IDE 报错

    vec3 skPos = p2.xyz;

    vec3 cPos = texelFetch(u_PhysicsNodes, cNode).xyz;
    vec3 finalPos;

    if (useCross) {
        vec3 vxPos = texelFetch(u_PhysicsNodes, vxNode).xyz;
        vec3 vyPos = texelFetch(u_PhysicsNodes, vyNode).xyz;

        vec3 vx = vxPos - cPos;
        vec3 vy = vyPos - cPos;

        vec3 baseN = cross(vx, vy);
        if (dot(baseN, baseN) > 1e-10) {
            baseN = normalize(baseN);
        }

        finalPos = cPos + (vx * w.x) + (vy * w.y) + (baseN * w.z);
    } else {
        finalPos = cPos + skPos;
    }

    gl_Position = ProjMat * ModelViewMat * vec4(finalPos, 1.0);

    vertexColor = vec4(1.0);
    texCoord0 = UV0;
}