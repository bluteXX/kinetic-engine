#version 450

// Lit, textured mesh. Stream 0: position, normal, tangent (xyz + handedness); stream 1: uv.
// SDL_GPU binding rules: vertex-stage uniform buffers live in set 1.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aTangent;
layout(location = 3) in vec2 aUv;

layout(set = 1, binding = 0) uniform DrawUniforms {
    mat4 mvp;
    mat4 normalMatrix; // inverse-transpose of the model matrix (upper 3x3 used)
    mat4 model;
};

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec4 vTangent;
layout(location = 2) out vec2 vUv;

void main()
{
    gl_Position = mvp * vec4(aPosition, 1.0);
    vNormal = mat3(normalMatrix) * aNormal;
    vTangent = vec4(mat3(model) * aTangent.xyz, aTangent.w);
    vUv = aUv;
}
