#version 450

// Hard-coded triangle (no vertex buffer) - first sanity check of the GPU pipeline.
layout(location = 0) out vec3 vColor;

const vec2 kPositions[3] = vec2[](vec2(0.0, 0.6), vec2(0.6, -0.5), vec2(-0.6, -0.5));
const vec3 kColors[3] = vec3[](vec3(0.95, 0.35, 0.30), vec3(0.30, 0.85, 0.45), vec3(0.30, 0.50, 0.95));

void main()
{
    gl_Position = vec4(kPositions[gl_VertexIndex], 0.0, 1.0);
    vColor = kColors[gl_VertexIndex];
}
