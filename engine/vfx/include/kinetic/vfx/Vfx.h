#pragma once

#include <kinetic/core/Json.h>
#include <kinetic/core/Types.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinetic::vfx {

// Simple particle effects (hit sparks, wood splinters, dust, blood), described in small JSON files
// (assets/vfx/*.vfx): one or more emitters, each a burst of particles of one colour.
//
//   {
//     "emitters": [
//       { "shape": "spark", "count": 24, "color": [1.0, 0.75, 0.3],
//         "size": [0.004, 0.008], "speed": [3, 7], "spread": 70, "lifetime": [0.15, 0.35],
//         "gravity": 1.0, "drag": 1.0, "stretch": 0.02 }
//     ]
//   }
//
// Shapes: spark (a thin streak along its motion), shard (a tumbling flat chip), puff (a ball that
// grows: dust, smoke), drop (a droplet stretched along its motion). Directions are spread in a
// cone (`spread`, degrees) around the surface normal, bent towards where the striker was sliding
// (`slide`, 0..1); `inherit` adds a share of the striker's velocity reflected off the surface.
enum class Shape : u8 { Spark, Shard, Puff, Drop };

struct Range {
    f32 min = 0.0f;
    f32 max = 0.0f;
};

struct EmitterDesc {
    std::string name;
    Shape shape = Shape::Spark;
    u32 count = 12;
    glm::vec3 color{1.0f};
    Range size{0.005f, 0.01f};   // m (half size)
    Range speed{1.0f, 3.0f};     // m/s
    f32 spread = 60.0f;          // degrees
    f32 slide = 0.3f;
    f32 inherit = 0.0f;
    Range lifetime{0.3f, 0.6f};  // s
    f32 gravity = 1.0f;          // x 9.81 m/s^2 down (negative: rises)
    f32 drag = 0.5f;             // 1/s
    f32 spin = 0.0f;             // rad/s (shards)
    f32 grow = 1.0f;             // size at the end of life (puffs), x start
    f32 stretch = 0.02f;         // sparks / drops: extra length per m/s, s
    f32 bounce = 0.3f;           // off the ground
    bool scaleWithSpeed = true;  // more particles for harder hits
};

struct VfxDesc {
    std::string name;
    std::vector<EmitterDesc> emitters;
};

[[nodiscard]] std::optional<VfxDesc> parseVfx(const Json& json, std::string* error = nullptr);
[[nodiscard]] Json vfxToJson(const VfxDesc& desc);

// Effects by name (file stem: assets/vfx/hit_wood.vfx -> "hit_wood").
class VfxLibrary {
public:
    // Loads every *.vfx in `directory`; returns how many loaded (errors are logged).
    usize loadDirectory(const std::filesystem::path& directory);
    void add(std::string name, VfxDesc desc);
    [[nodiscard]] const VfxDesc* find(std::string_view name) const;
    [[nodiscard]] std::vector<std::string> names() const;
    [[nodiscard]] bool empty() const { return m_effects.empty(); }

private:
    std::unordered_map<std::string, VfxDesc> m_effects;
};

struct Particle {
    glm::vec3 position{0.0f};
    glm::vec3 velocity{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 spinAxis{0.0f, 1.0f, 0.0f};
    f32 size = 0.01f;
    f32 age = 0.0f;
    f32 lifetime = 0.5f;
    [[nodiscard]] bool alive() const { return age < lifetime; }
};

// One burst of one emitter.
struct Burst {
    u32 id = 0;
    EmitterDesc desc;
    std::vector<Particle> particles;
    [[nodiscard]] bool alive() const;
};

// Runs bursts. Rendering: buildMesh() gives each burst's particles as one triangle mesh (a fixed
// vertex count per burst, dead particles collapse), drawn with the emitter's colour.
class VfxSystem {
public:
    // `strikerVelocity`: of what hit (bends and feeds the particles); `intensity` ~ hit strength
    // (1 = a normal hit; scales the count of emitters with scaleWithSpeed).
    void spawn(const VfxDesc& desc, const glm::vec3& point, const glm::vec3& normal, const glm::vec3& strikerVelocity = glm::vec3(0.0f),
               f32 intensity = 1.0f);
    void update(f32 dt);
    void clear() { m_bursts.clear(); }
    // Particles bounce off this height (the floor).
    void setGround(f32 y) { m_ground = y; }

    [[nodiscard]] const std::vector<Burst>& bursts() const { return m_bursts; }
    [[nodiscard]] usize particleCount() const;

    static constexpr usize kVerticesPerParticle = 24; // octahedron, flat shaded
    [[nodiscard]] static usize vertexCount(const Burst& burst) { return burst.particles.size() * kVerticesPerParticle; }
    // Positions / normals for the burst (sizes: vertexCount(burst)); indices are 0..n-1.
    static void buildMesh(const Burst& burst, std::span<glm::vec3> positions, std::span<glm::vec3> normals);

private:
    std::vector<Burst> m_bursts;
    u32 m_nextId = 1;
    u32 m_seed = 0x9E3779B9u;
    f32 m_ground = -1e9f;
};

} // namespace kinetic::vfx
