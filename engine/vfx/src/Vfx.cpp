#include "kinetic/vfx/Vfx.h"

#include <kinetic/core/Log.h>

#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace kinetic::vfx {

namespace {

constexpr f32 kPi = 3.14159265f;

const char* shapeName(Shape s)
{
    switch (s) {
    case Shape::Spark: return "spark";
    case Shape::Shard: return "shard";
    case Shape::Puff: return "puff";
    case Shape::Drop: return "drop";
    }
    return "spark";
}

Range rangeOf(const Json& j, Range fallback)
{
    if (j.isNumber()) return {j.asFloat(), j.asFloat()};
    if (j.isArray() && j.size() >= 2) return {j[0].asFloat(fallback.min), j[1].asFloat(fallback.max)};
    return fallback;
}

Json rangeJson(Range r) { return Json(Json::Array{r.min, r.max}); }

// xorshift: cheap, deterministic.
f32 random01(u32& state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return static_cast<f32>(state & 0xFFFFFFu) / static_cast<f32>(0x1000000u);
}

f32 pick(Range r, u32& state) { return r.min + (r.max - r.min) * random01(state); }

glm::vec3 anyPerpendicular(const glm::vec3& v)
{
    const glm::vec3 other = std::abs(v.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    return glm::normalize(glm::cross(v, other));
}

} // namespace

std::optional<VfxDesc> parseVfx(const Json& json, std::string* error)
{
    if (!json["emitters"].isArray() || json["emitters"].size() == 0) {
        if (error) *error = "no \"emitters\"";
        return std::nullopt;
    }
    VfxDesc desc;
    desc.name = json["name"].asString();
    for (const Json& e : json["emitters"].items()) {
        EmitterDesc d;
        d.name = e["name"].asString();
        const std::string shape = e["shape"].asString();
        d.shape = shape == "shard" ? Shape::Shard : shape == "puff" ? Shape::Puff : shape == "drop" ? Shape::Drop : Shape::Spark;
        d.count = static_cast<u32>(std::clamp(e["count"].asInt(12), 1, 400));
        d.color = e["color"].asVec3(d.color);
        d.size = rangeOf(e["size"], d.size);
        d.speed = rangeOf(e["speed"], d.speed);
        d.spread = e["spread"].asFloat(d.spread);
        d.slide = e["slide"].asFloat(d.slide);
        d.inherit = e["inherit"].asFloat(d.inherit);
        d.lifetime = rangeOf(e["lifetime"], d.lifetime);
        d.gravity = e["gravity"].asFloat(d.gravity);
        d.drag = e["drag"].asFloat(d.drag);
        d.spin = e["spin"].asFloat(d.spin);
        d.grow = e["grow"].asFloat(d.grow);
        d.stretch = e["stretch"].asFloat(d.stretch);
        d.bounce = e["bounce"].asFloat(d.bounce);
        d.scaleWithSpeed = e["scaleWithSpeed"].asBool(d.scaleWithSpeed);
        desc.emitters.push_back(std::move(d));
    }
    return desc;
}

Json vfxToJson(const VfxDesc& desc)
{
    Json out = Json::object();
    if (!desc.name.empty()) out.set("name", desc.name);
    Json emitters = Json::array();
    for (const EmitterDesc& d : desc.emitters) {
        Json e = Json::object();
        if (!d.name.empty()) e.set("name", d.name);
        e.set("shape", shapeName(d.shape));
        e.set("count", static_cast<int>(d.count));
        e.set("color", d.color);
        e.set("size", rangeJson(d.size));
        e.set("speed", rangeJson(d.speed));
        e.set("spread", d.spread);
        e.set("slide", d.slide);
        e.set("inherit", d.inherit);
        e.set("lifetime", rangeJson(d.lifetime));
        e.set("gravity", d.gravity);
        e.set("drag", d.drag);
        e.set("spin", d.spin);
        e.set("grow", d.grow);
        e.set("stretch", d.stretch);
        e.set("bounce", d.bounce);
        e.set("scaleWithSpeed", d.scaleWithSpeed);
        emitters.push(std::move(e));
    }
    out.set("emitters", std::move(emitters));
    return out;
}

// ---------------------------------------------------------------------------------------------- //

usize VfxLibrary::loadDirectory(const std::filesystem::path& directory)
{
    std::error_code ec;
    usize loaded = 0;
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".vfx") continue;
        std::string error;
        const auto json = loadJsonFile(entry.path().string(), &error);
        std::optional<VfxDesc> desc = json ? parseVfx(*json, &error) : std::nullopt;
        if (!desc) {
            KN_WARN("VFX '{}': {}", entry.path().filename().string(), error);
            continue;
        }
        add(entry.path().stem().string(), std::move(*desc));
        ++loaded;
    }
    return loaded;
}

void VfxLibrary::add(std::string name, VfxDesc desc)
{
    if (desc.name.empty()) desc.name = name;
    m_effects[std::move(name)] = std::move(desc);
}

const VfxDesc* VfxLibrary::find(std::string_view name) const
{
    const auto it = m_effects.find(std::string(name));
    return it == m_effects.end() ? nullptr : &it->second;
}

std::vector<std::string> VfxLibrary::names() const
{
    std::vector<std::string> out;
    for (const auto& [name, desc] : m_effects) out.push_back(name);
    std::ranges::sort(out);
    return out;
}

// ---------------------------------------------------------------------------------------------- //

bool Burst::alive() const
{
    return std::ranges::any_of(particles, [](const Particle& p) { return p.alive(); });
}

void VfxSystem::spawn(const VfxDesc& desc, const glm::vec3& point, const glm::vec3& normalIn, const glm::vec3& strikerVelocity,
                      f32 intensity)
{
    const glm::vec3 normal = glm::length(normalIn) > 1e-5f ? glm::normalize(normalIn) : glm::vec3(0.0f, 1.0f, 0.0f);
    // Where the striker slid along the surface, and its velocity reflected off it.
    const glm::vec3 into = normal * glm::dot(strikerVelocity, normal);
    const glm::vec3 slideVelocity = strikerVelocity - into;
    const glm::vec3 slideDir = glm::length(slideVelocity) > 0.1f ? glm::normalize(slideVelocity) : glm::vec3(0.0f);
    const glm::vec3 reflected = slideVelocity - into;
    for (const EmitterDesc& e : desc.emitters) {
        Burst burst;
        burst.id = m_nextId++;
        burst.desc = e;
        const f32 scale = e.scaleWithSpeed ? std::clamp(intensity, 0.3f, 2.0f) : 1.0f;
        const usize count = std::max<usize>(1, static_cast<usize>(std::lround(static_cast<f32>(e.count) * scale)));
        burst.particles.resize(count);
        const glm::vec3 axis = glm::normalize(normal + slideDir * e.slide);
        const glm::vec3 u = anyPerpendicular(axis);
        const glm::vec3 v = glm::cross(axis, u);
        const f32 cone = glm::radians(std::clamp(e.spread, 0.0f, 180.0f));
        for (Particle& p : burst.particles) {
            // Uniform in the cone's solid angle.
            const f32 cosT = 1.0f - random01(m_seed) * (1.0f - std::cos(cone));
            const f32 sinT = std::sqrt(std::max(0.0f, 1.0f - cosT * cosT));
            const f32 phi = 2.0f * kPi * random01(m_seed);
            const glm::vec3 dir = axis * cosT + (u * std::cos(phi) + v * std::sin(phi)) * sinT;
            p.position = point + normal * 0.005f;
            p.velocity = dir * pick(e.speed, m_seed) * std::sqrt(scale) + reflected * e.inherit;
            p.size = pick(e.size, m_seed);
            p.lifetime = std::max(pick(e.lifetime, m_seed), 0.02f);
            p.spinAxis = glm::normalize(glm::vec3(random01(m_seed) - 0.5f, random01(m_seed) - 0.5f, random01(m_seed) - 0.5f) + glm::vec3(1e-3f));
            p.rotation = glm::angleAxis(random01(m_seed) * 2.0f * kPi, p.spinAxis);
        }
        m_bursts.push_back(std::move(burst));
    }
}

void VfxSystem::update(f32 dt)
{
    if (dt <= 0.0f) return;
    for (Burst& b : m_bursts) {
        const EmitterDesc& e = b.desc;
        for (Particle& p : b.particles) {
            if (!p.alive()) continue;
            p.age += dt;
            p.velocity += glm::vec3(0.0f, -9.81f * e.gravity, 0.0f) * dt;
            p.velocity *= std::exp(-e.drag * dt);
            p.position += p.velocity * dt;
            if (p.position.y < m_ground) { // bounce off the floor, losing most of the speed
                p.position.y = m_ground;
                if (p.velocity.y < 0.0f) p.velocity.y = -p.velocity.y * e.bounce;
                p.velocity.x *= 0.6f;
                p.velocity.z *= 0.6f;
            }
            if (e.spin > 0.0f) {
                p.rotation = glm::normalize(glm::angleAxis(e.spin * dt, p.spinAxis) * p.rotation);
            }
        }
    }
    std::erase_if(m_bursts, [](const Burst& b) { return !b.alive(); });
}

usize VfxSystem::particleCount() const
{
    usize n = 0;
    for (const Burst& b : m_bursts) {
        for (const Particle& p : b.particles) n += p.alive() ? 1 : 0;
    }
    return n;
}

void VfxSystem::buildMesh(const Burst& burst, std::span<glm::vec3> positions, std::span<glm::vec3> normals)
{
    // Octahedron: 8 faces x 3 vertices (flat normals).
    static const glm::vec3 kCorners[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    static const int kFaces[8][3] = {{0, 2, 4}, {4, 2, 1}, {1, 2, 5}, {5, 2, 0}, {4, 3, 0}, {1, 3, 4}, {5, 3, 1}, {0, 3, 5}};
    const EmitterDesc& e = burst.desc;
    usize v = 0;
    for (const Particle& p : burst.particles) {
        if (v + kVerticesPerParticle > positions.size()) break;
        if (!p.alive()) {
            for (usize k = 0; k < kVerticesPerParticle; ++k) {
                positions[v + k] = p.position;
                normals[v + k] = glm::vec3(0.0f, 1.0f, 0.0f);
            }
            v += kVerticesPerParticle;
            continue;
        }
        const f32 t = std::clamp(p.age / p.lifetime, 0.0f, 1.0f);
        f32 size = p.size;
        glm::vec3 scale(1.0f);
        glm::quat rotation = p.rotation;
        const f32 speed = glm::length(p.velocity);
        switch (e.shape) {
        case Shape::Spark:
        case Shape::Drop: {
            size *= e.shape == Shape::Spark ? (1.0f - t * t) : (1.0f - t * t * t);
            const f32 length = 1.0f + speed * e.stretch / std::max(size, 1e-4f);
            scale = glm::vec3(1.0f, std::min(length, e.shape == Shape::Spark ? 40.0f : 3.0f), 1.0f);
            if (speed > 1e-3f) rotation = glm::rotation(glm::vec3(0.0f, 1.0f, 0.0f), p.velocity / speed);
            break;
        }
        case Shape::Shard:
            size *= 1.0f - std::pow(t, 4.0f);
            scale = glm::vec3(1.0f, 0.25f, 0.5f);
            break;
        case Shape::Puff:
            size *= (1.0f + (e.grow - 1.0f) * std::sqrt(t)) * (1.0f - std::pow(t, 3.0f));
            break;
        }
        const glm::mat3 basis = glm::mat3_cast(rotation);
        for (int f = 0; f < 8; ++f) {
            glm::vec3 c[3];
            for (int k = 0; k < 3; ++k) c[k] = p.position + basis * (kCorners[kFaces[f][k]] * scale * size);
            glm::vec3 n = glm::cross(c[1] - c[0], c[2] - c[0]);
            n = glm::length(n) > 1e-12f ? glm::normalize(n) : glm::vec3(0.0f, 1.0f, 0.0f);
            for (int k = 0; k < 3; ++k) {
                positions[v] = c[k];
                normals[v] = n;
                ++v;
            }
        }
    }
    for (; v < positions.size(); ++v) { // unused tail
        positions[v] = glm::vec3(0.0f);
        normals[v] = glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

} // namespace kinetic::vfx
