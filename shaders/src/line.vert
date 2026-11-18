#version 450

// Coloured debug lines (grid, axes) in world space.
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;

layout(set = 1, binding = 0) uniform LineUniforms {
    mat4 viewProjection;
};

layout(location = 0) out vec3 vColor;

void main()
{
    gl_Position = viewProjection * vec4(aPosition, 1.0);
    vColor = aColor;
}
