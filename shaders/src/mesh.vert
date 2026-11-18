#version 450

// Unlit mesh: position + vertex colour, transformed by a per-draw MVP.
// SDL_GPU binding rules: vertex-stage uniform buffers live in set 1.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;

layout(set = 1, binding = 0) uniform DrawUniforms {
    mat4 mvp;
};

layout(location = 0) out vec3 vColor;

void main()
{
    gl_Position = mvp * vec4(aPosition, 1.0);
    vColor = aColor;
}
