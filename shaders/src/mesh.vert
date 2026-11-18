#version 450

// Lit mesh: position + normal, per-draw MVP and normal matrix.
// SDL_GPU binding rules: vertex-stage uniform buffers live in set 1.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

layout(set = 1, binding = 0) uniform DrawUniforms {
    mat4 mvp;
    mat4 normalMatrix; // inverse-transpose of the model matrix (upper 3x3 used)
};

layout(location = 0) out vec3 vNormal;

void main()
{
    gl_Position = mvp * vec4(aPosition, 1.0);
    vNormal = mat3(normalMatrix) * aNormal;
}
