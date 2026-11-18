#version 450

// Simple two-sided lighting: ambient + a fixed key light + a headlight from the camera,
// so every face is readable from any angle.
// SDL_GPU binding rules: fragment-stage uniform buffers live in set 3.
layout(set = 3, binding = 0) uniform MaterialUniforms {
    vec4 tint;
    vec4 keyLightDir; // world space, points towards the light
    vec4 cameraDir;   // world space, points from the camera into the scene
};

layout(location = 0) in vec3 vNormal;
layout(location = 0) out vec4 outColor;

void main()
{
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) {
        n = -n;
    }
    float key = max(dot(n, keyLightDir.xyz), 0.0);
    float head = max(dot(n, -cameraDir.xyz), 0.0);
    float light = 0.25 + 0.5 * key + 0.35 * head;
    outColor = vec4(tint.rgb * light, 1.0);
}
