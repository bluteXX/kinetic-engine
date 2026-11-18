#version 450

// SDL_GPU binding rules: fragment-stage uniform buffers live in set 3.
layout(set = 3, binding = 0) uniform MaterialUniforms {
    vec4 tint;
};

layout(location = 0) in vec3 vColor;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = vec4(vColor, 1.0) * tint;
}
