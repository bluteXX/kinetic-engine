#include "kinetic/core/Sword.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace kinetic {

namespace {

constexpr f32 kPi = 3.14159265f;

// Texture atlas bands (u ranges). Each part samples inside its band with a margin, so linear
// filtering never bleeds a neighbour in.
struct Band {
    f32 u0, u1;
    [[nodiscard]] f32 at(f32 t) const { return u0 + (u1 - u0) * (0.15f + 0.7f * std::clamp(t, 0.0f, 1.0f)); }
};
constexpr Band kEdge{0.0f, 0.25f};
constexpr Band kFlat{0.25f, 0.5f};
constexpr Band kFuller{0.5f, 0.625f};
constexpr Band kDark{0.625f, 0.75f};
constexpr Band kLeather{0.75f, 1.0f};

constexpr f32 kBottom = -0.175f; // v = 0 here (y), v = 1 at the tip
constexpr f32 kTip = 1.05f;
f32 vOf(f32 y) { return std::clamp((y - kBottom) / (kTip - kBottom), 0.0f, 1.0f); }

// One triangle whose winding faces `n`.
void triangle(MeshData& m, u32 a, u32 b, u32 c, const glm::vec3& n)
{
    const glm::vec3 face = glm::cross(m.positions[b] - m.positions[a], m.positions[c] - m.positions[a]);
    if (glm::dot(face, n) < 0.0f) std::swap(b, c);
    m.indices.insert(m.indices.end(), {a, b, c});
}

// A flat-shaded quad (a b c d around it, either way); `out` is any point it faces away from.
void quad(MeshData& m, const std::array<glm::vec3, 4>& p, const std::array<glm::vec2, 4>& uv, const glm::vec3& inside)
{
    glm::vec3 n = glm::cross(p[2] - p[0], p[3] - p[1]); // diagonals: fine for degenerate quads too
    if (glm::length(n) < 1e-12f) return;
    n = glm::normalize(n);
    const glm::vec3 center = (p[0] + p[1] + p[2] + p[3]) * 0.25f;
    if (glm::dot(n, center - inside) < 0.0f) n = -n;
    const auto base = static_cast<u32>(m.positions.size());
    for (int i = 0; i < 4; ++i) {
        m.positions.push_back(p[static_cast<usize>(i)]);
        m.normals.push_back(n);
        m.uvs.push_back(uv[static_cast<usize>(i)]);
    }
    triangle(m, base, base + 1, base + 2, n);
    triangle(m, base, base + 2, base + 3, n);
}

// A smooth surface p(s, t), s around (closed when `closed`), t along; normals from the surface's
// derivatives, facing away from `inside(p)`.
void surface(MeshData& m, u32 ns, u32 nt, bool closed, const std::function<glm::vec3(f32, f32)>& p,
             const std::function<glm::vec2(f32, f32)>& uv, const std::function<glm::vec3(const glm::vec3&)>& inside)
{
    const auto base = static_cast<u32>(m.positions.size());
    constexpr f32 h = 1e-3f;
    for (u32 j = 0; j <= nt; ++j) {
        const f32 t = static_cast<f32>(j) / static_cast<f32>(nt);
        for (u32 i = 0; i <= ns; ++i) {
            const f32 s = static_cast<f32>(i) / static_cast<f32>(ns);
            const glm::vec3 pos = p(s, t);
            const f32 s0 = closed ? s - h : std::max(s - h, 0.0f);
            const f32 s1 = closed ? s + h : std::min(s + h, 1.0f);
            const glm::vec3 ds = p(s1, t) - p(s0, t);
            const glm::vec3 dt = p(s, std::min(t + h, 1.0f)) - p(s, std::max(t - h, 0.0f));
            glm::vec3 n = glm::cross(ds, dt);
            n = glm::length(n) > 1e-14f ? glm::normalize(n) : glm::normalize(pos - inside(pos) + glm::vec3(1e-6f));
            if (glm::dot(n, pos - inside(pos)) < 0.0f) n = -n;
            m.positions.push_back(pos);
            m.normals.push_back(n);
            m.uvs.push_back(uv(s, t));
        }
    }
    const u32 stride = ns + 1;
    for (u32 j = 0; j < nt; ++j) {
        for (u32 i = 0; i < ns; ++i) {
            const u32 a = base + j * stride + i;
            const u32 b = a + 1;
            const u32 c = a + stride + 1;
            const u32 d = a + stride;
            const glm::vec3 n = m.normals[a] + m.normals[c];
            triangle(m, a, b, c, n);
            triangle(m, a, c, d, n);
        }
    }
}

// --- Parts ----------------------------------------------------------------------------------------

void addBlade(MeshData& m)
{
    constexpr f32 y0 = 0.118f; // buried in the guard
    constexpr u32 kStations = 48;
    const auto halfWidth = [](f32 s) { // s: 0 at the guard, 1 at the tip
        constexpr f32 w0 = 0.025f;
        constexpr f32 knee = 0.82f;
        if (s < knee) return w0 * (1.0f - 0.38f * s);
        const f32 u = (s - knee) / (1.0f - knee);
        return w0 * (1.0f - 0.38f * knee) * (1.0f - u) * (1.0f + 0.8f * u); // convex point
    };
    // Cross-section (x, z) at s: edge, bevel, fuller rim, fuller bottom ... around both flats.
    struct Section {
        std::array<glm::vec2, 12> p;
    };
    const auto section = [&](f32 s) {
        const f32 w = halfWidth(s);
        const f32 t = 0.0046f * (1.0f - 0.5f * s); // half thickness at the ridge
        const f32 bevel = std::min(0.0055f, 0.55f * w);
        const f32 fullerEnd = 0.6f;
        const f32 fade = s < fullerEnd - 0.1f ? 1.0f : std::max(0.0f, (fullerEnd - s) / 0.1f);
        const f32 f = std::min(0.32f * w, 0.5f * (w - bevel)) * fade;
        const f32 d = 0.0018f * fade;
        Section sec;
        sec.p = {glm::vec2(w, 0.0f),   glm::vec2(w - bevel, t), glm::vec2(f, t),    glm::vec2(0.0f, t - d),
                 glm::vec2(-f, t),     glm::vec2(-(w - bevel), t), glm::vec2(-w, 0.0f), glm::vec2(-(w - bevel), -t),
                 glm::vec2(-f, -t),    glm::vec2(0.0f, -(t - d)), glm::vec2(f, -t),  glm::vec2(w - bevel, -t)};
        return sec;
    };
    // Facet k runs from point k to k+1: which band it samples.
    const std::array<Band, 12> bands = {kEdge, kFlat, kFuller, kFuller, kFlat, kEdge, kEdge, kFlat, kFuller, kFuller, kFlat, kEdge};
    for (u32 i = 0; i < kStations; ++i) {
        const f32 sa = static_cast<f32>(i) / kStations;
        const f32 sb = static_cast<f32>(i + 1) / kStations;
        const f32 ya = y0 + (kTip - y0) * sa;
        const f32 yb = y0 + (kTip - y0) * sb;
        const Section a = section(sa);
        const Section b = section(sb);
        for (usize k = 0; k < 12; ++k) {
            const usize l = (k + 1) % 12;
            const auto P = [](const glm::vec2& q, f32 y) { return glm::vec3(q.x, y, q.y); };
            quad(m, {P(a.p[k], ya), P(a.p[l], ya), P(b.p[l], yb), P(b.p[k], yb)},
                 {glm::vec2(bands[k].at(0.0f), vOf(ya)), glm::vec2(bands[k].at(1.0f), vOf(ya)), glm::vec2(bands[k].at(1.0f), vOf(yb)),
                  glm::vec2(bands[k].at(0.0f), vOf(yb))},
                 glm::vec3(0.0f, (ya + yb) * 0.5f, 0.0f));
        }
    }
}

void addGuard(MeshData& m)
{
    constexpr f32 L = 0.112f;
    constexpr u32 kStations = 40;
    // Along x: centre line height and half sizes (y, z), flaring at the ends.
    const auto profile = [](f32 x) {
        const f32 a = std::abs(x) / L;
        const f32 flare = a > 0.82f ? (a - 0.82f) / 0.18f : 0.0f;
        const f32 bulb = std::sin(std::min(flare, 1.0f) * kPi * 0.5f);
        struct P {
            f32 cy, hy, hz;
        };
        // The centre swells into a block where the blade enters (the ecusson).
        const f32 block = std::exp(-(x / 0.022f) * (x / 0.022f));
        return P{0.112f + 0.016f * a * a, 0.0115f - 0.0045f * std::min(a / 0.82f, 1.0f) + 0.0045f * bulb + 0.005f * block,
                 0.0145f - 0.005f * std::min(a / 0.82f, 1.0f) + 0.004f * bulb + 0.004f * block};
    };
    const auto octagon = [](f32 hy, f32 hz) {
        return std::array<glm::vec2, 8>{glm::vec2(hy, 0.45f * hz), glm::vec2(0.45f * hy, hz), glm::vec2(-0.45f * hy, hz), glm::vec2(-hy, 0.45f * hz),
                                        glm::vec2(-hy, -0.45f * hz), glm::vec2(-0.45f * hy, -hz), glm::vec2(0.45f * hy, -hz), glm::vec2(hy, -0.45f * hz)};
    };
    const auto at = [&](f32 x, const glm::vec2& q) { return glm::vec3(x, profile(x).cy + q.x, q.y); };
    for (u32 i = 0; i < kStations; ++i) {
        const f32 xa = -L + 2.0f * L * static_cast<f32>(i) / kStations;
        const f32 xb = -L + 2.0f * L * static_cast<f32>(i + 1) / kStations;
        const auto pa = profile(xa);
        const auto pb = profile(xb);
        const auto a = octagon(pa.hy, pa.hz);
        const auto b = octagon(pb.hy, pb.hz);
        for (usize k = 0; k < 8; ++k) {
            const usize l = (k + 1) % 8;
            const f32 u0 = static_cast<f32>(k) / 8.0f;
            const f32 u1 = static_cast<f32>(k + 1) / 8.0f;
            const f32 va = 0.02f + 0.3f * (xa + L) / (2.0f * L);
            const f32 vb = 0.02f + 0.3f * (xb + L) / (2.0f * L);
            quad(m, {at(xa, a[k]), at(xa, a[l]), at(xb, b[l]), at(xb, b[k])},
                 {glm::vec2(kDark.at(u0), va), glm::vec2(kDark.at(u1), va), glm::vec2(kDark.at(u1), vb), glm::vec2(kDark.at(u0), vb)},
                 glm::vec3((xa + xb) * 0.5f, profile((xa + xb) * 0.5f).cy, 0.0f));
        }
    }
    // End caps: a fan of flat quads (pairs of rim points to the centre).
    for (const f32 x : {-L, L}) {
        const auto p = profile(x);
        const auto o = octagon(p.hy, p.hz);
        const glm::vec3 c(x, p.cy, 0.0f);
        const glm::vec3 inside(x - (x > 0.0f ? 1.0f : -1.0f), p.cy, 0.0f);
        for (usize k = 0; k < 8; k += 2) {
            quad(m, {c, at(x, o[k]), at(x, o[(k + 1) % 8]), at(x, o[(k + 2) % 8])},
                 {glm::vec2(kDark.at(0.5f), 0.5f), glm::vec2(kDark.at(0.0f), 0.48f), glm::vec2(kDark.at(0.5f), 0.46f), glm::vec2(kDark.at(1.0f), 0.48f)},
                 inside);
        }
    }
}

void addGrip(MeshData& m)
{
    constexpr f32 y0 = -0.105f;
    constexpr f32 y1 = 0.104f;
    constexpr f32 kTurns = 9.0f;
    const auto radius = [&](f32 s, f32 t) { // s around (0..1), t along (0..1)
        const f32 barrel = 1.0f + 0.07f * std::sin(kPi * t);
        const f32 phase = t * kTurns + s;
        const f32 ridge = std::pow(0.5f + 0.5f * std::cos(2.0f * kPi * phase), 6.0f); // cord wrap
        return barrel * (1.0f + 0.05f * ridge);
    };
    surface(
        m, 32, 96, true,
        [&](f32 s, f32 t) {
            const f32 a = 2.0f * kPi * s;
            const f32 r = radius(s, t);
            return glm::vec3(0.0155f * r * std::cos(a), y0 + (y1 - y0) * t, 0.0135f * r * std::sin(a));
        },
        [&](f32 s, f32 t) { return glm::vec2(kLeather.at(s), 0.05f + 0.9f * t); },
        [](const glm::vec3& p) { return glm::vec3(0.0f, p.y, 0.0f); });
    // Steel collars at both ends of the grip.
    for (const auto& [ya, yb, r] : {std::array<f32, 3>{0.098f, 0.108f, 0.0185f}, std::array<f32, 3>{-0.114f, -0.1f, 0.0175f}}) {
        surface(
            m, 24, 3, true,
            [&](f32 s, f32 t) {
                const f32 a = 2.0f * kPi * s;
                const f32 bulge = 1.0f + 0.08f * std::sin(kPi * t);
                return glm::vec3(r * bulge * std::cos(a), ya + (yb - ya) * t, 0.86f * r * bulge * std::sin(a));
            },
            [&](f32 s, f32 t) { return glm::vec2(kDark.at(s), 0.6f + 0.05f * t); },
            [](const glm::vec3& p) { return glm::vec3(0.0f, p.y, 0.0f); });
    }
}

void addPommel(MeshData& m)
{
    // A wheel pommel turned about Z: lathe profile (radius, z) from the centre of one face around
    // the rim to the other.
    const glm::vec3 c(0.0f, -0.142f, 0.0f);
    const std::array<glm::vec2, 7> profile = {glm::vec2(0.0f, 0.0125f), glm::vec2(0.012f, 0.0135f), glm::vec2(0.021f, 0.0115f),
                                              glm::vec2(0.03f, 0.0055f),  glm::vec2(0.03f, -0.0055f), glm::vec2(0.021f, -0.0115f),
                                              glm::vec2(0.0f, -0.0125f)};
    // Faces bulge slightly (a boss in the middle); the rim is bevelled.
    for (usize k = 0; k + 1 < profile.size(); ++k) {
        const glm::vec2 a = profile[k];
        const glm::vec2 b = profile[k + 1];
        surface(
            m, 32, 2, true,
            [&](f32 s, f32 t) {
                const f32 ang = 2.0f * kPi * s;
                const glm::vec2 q = glm::mix(a, b, t);
                return c + glm::vec3(q.x * std::cos(ang), q.x * std::sin(ang), q.y);
            },
            [&](f32 s, f32 t) { return glm::vec2(kDark.at(s), 0.7f + 0.1f * (static_cast<f32>(k) + t) / 6.0f); },
            [&](const glm::vec3&) { return c; });
    }
}

// Cheap value noise (hash on integer lattice) for the texture.
f32 hash(i32 x, i32 y)
{
    u32 h = static_cast<u32>(x) * 374761393u + static_cast<u32>(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return static_cast<f32>((h ^ (h >> 16)) & 0xffffu) / 65535.0f;
}

} // namespace

MeshData makeSwordMesh()
{
    MeshData m;
    addBlade(m);
    addGuard(m);
    addGrip(m);
    addPommel(m);
    m.material.name = "Sword";
    computeTangents(m);
    return m;
}

ImageData makeSwordTexture()
{
    ImageData img;
    img.width = 128;
    img.height = 256;
    img.rgba.resize(static_cast<usize>(img.width) * img.height * 4);
    for (u32 y = 0; y < img.height; ++y) {
        for (u32 x = 0; x < img.width; ++x) {
            const f32 u = (static_cast<f32>(x) + 0.5f) / static_cast<f32>(img.width);
            const f32 v = (static_cast<f32>(y) + 0.5f) / static_cast<f32>(img.height); // row = v * height (v = 0 at the top)
            // Brushed metal: streaks along the blade (vary across u, slowly along v).
            const f32 streak = 0.6f * hash(static_cast<i32>(x), static_cast<i32>(y) / 24) + 0.4f * hash(static_cast<i32>(x), 1000 + static_cast<i32>(y) / 7);
            glm::vec3 c;
            if (u < kEdge.u1) {
                // Honed edge: bright, a little brighter towards the cutting edge.
                const f32 a = (u - kEdge.u0) / (kEdge.u1 - kEdge.u0);
                c = glm::mix(glm::vec3(0.80f, 0.83f, 0.87f), glm::vec3(0.93f, 0.95f, 0.97f), a < 0.5f ? 1.0f - a : a) * (0.96f + 0.06f * streak);
            } else if (u < kFlat.u1) {
                c = glm::vec3(0.66f, 0.69f, 0.74f) * (0.93f + 0.12f * streak);
            } else if (u < kFuller.u1) {
                c = glm::vec3(0.50f, 0.53f, 0.58f) * (0.92f + 0.12f * streak);
            } else if (u < kDark.u1) {
                // Blackened steel, worn a little lighter in spots.
                const f32 n = 0.5f * hash(static_cast<i32>(x), static_cast<i32>(y)) + 0.5f * hash(static_cast<i32>(x) / 2, 500 + static_cast<i32>(y) / 2);
                c = glm::vec3(0.24f, 0.26f, 0.30f) * (0.92f + 0.14f * n);
            } else {
                // Leather with a dark cord spiral (matches the grip's 9 turns: phase = t * 9 + s).
                const f32 s = (u - kLeather.u0) / (kLeather.u1 - kLeather.u0);
                const f32 t = (v - 0.05f) / 0.9f;
                const f32 phase = t * 9.0f + (s - 0.15f) / 0.7f;
                const f32 f = phase - std::floor(phase);
                const f32 cord = std::pow(0.5f + 0.5f * std::cos(2.0f * kPi * f), 6.0f);
                const f32 n = hash(static_cast<i32>(x) / 2, static_cast<i32>(y) / 2);
                const glm::vec3 leather = glm::vec3(0.42f, 0.20f, 0.12f) * (0.85f + 0.25f * n);
                const glm::vec3 wrap = glm::vec3(0.12f, 0.08f, 0.06f);
                c = glm::mix(leather, wrap, std::clamp(cord * 1.4f, 0.0f, 1.0f));
            }
            u8* px = &img.rgba[(static_cast<usize>(y) * img.width + x) * 4];
            for (int k = 0; k < 3; ++k) px[k] = static_cast<u8>(std::clamp(c[k], 0.0f, 1.0f) * 255.0f + 0.5f);
            px[3] = 255;
        }
    }
    return img;
}

} // namespace kinetic
