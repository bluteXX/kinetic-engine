#version 450

// Base colour texture x tint, tangent-space normal map, and the editor's simple two-sided
// lighting: ambient + a fixed key light + a headlight from the camera, so every face is readable
// from any angle. Meshes without textures get the built-in white / flat-normal textures.
// SDL_GPU binding rules: fragment-stage samplers live in set 2, uniform buffers in set 3.
layout(set = 2, binding = 0) uniform sampler2D baseColorMap;
layout(set = 2, binding = 1) uniform sampler2D normalMap;

layout(set = 3, binding = 0) uniform MaterialUniforms {
    vec4 tint;
    vec4 keyLightDir; // world space, points towards the light
    vec4 cameraDir;   // world space, points from the camera into the scene
    vec4 params;      // x = normal strength, y = green sign (+1 OpenGL maps, -1 DirectX maps)
};

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec4 vTangent;
layout(location = 2) in vec2 vUv;
layout(location = 0) out vec4 outColor;

void main()
{
    vec3 n = normalize(vNormal);

    // Tangent frame (re-orthogonalised after interpolation); w = bitangent handedness.
    vec3 t = vTangent.xyz - n * dot(n, vTangent.xyz);
    float tl = length(t);
    if (tl > 1e-5 && params.x > 0.0) {
        t /= tl;
        vec3 b = cross(n, t) * vTangent.w;
        vec3 m = texture(normalMap, vUv).xyz * 2.0 - 1.0;
        m.y *= params.y;
        m.xy *= params.x;
        n = normalize(t * m.x + b * m.y + n * max(m.z, 1e-3));
    }
    if (!gl_FrontFacing) {
        n = -n;
    }

    vec4 base = texture(baseColorMap, vUv) * tint;
    float key = max(dot(n, keyLightDir.xyz), 0.0);
    float head = max(dot(n, -cameraDir.xyz), 0.0);
    float light = 0.25 + 0.5 * key + 0.35 * head;
    outColor = vec4(base.rgb * light, 1.0);
}
