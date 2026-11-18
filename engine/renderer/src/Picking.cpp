#include "kinetic/renderer/Picking.h"

#include "kinetic/scene/AnimatorComponent.h"

#include <span>

namespace kinetic::renderer {

namespace {

// Vertices an entity currently shows: posed vertices for animated entities, else the mesh's own.
std::span<const glm::vec3> currentPositions(const scene::Scene& scene, entt::entity entity, const GpuMesh& mesh)
{
    if (const auto* animator = scene.registry().try_get<scene::AnimatorComponent>(entity)) {
        if (animator->skinnedPositions.size() == mesh.positions.size()) {
            return animator->skinnedPositions;
        }
    }
    return mesh.positions;
}

} // namespace

Ray cameraRay(const scene::CameraComponent& camera, const scene::TransformComponent& transform, f32 aspect,
              const glm::vec2& ndc)
{
    const glm::mat4 inverseViewProjection = glm::inverse(camera.projection(aspect) * scene::CameraComponent::view(transform));
    const glm::vec4 nearPoint = inverseViewProjection * glm::vec4(ndc, 0.0f, 1.0f);
    const glm::vec4 farPoint = inverseViewProjection * glm::vec4(ndc, 1.0f, 1.0f);
    const glm::vec3 origin = glm::vec3(nearPoint) / nearPoint.w;
    return {origin, glm::normalize(glm::vec3(farPoint) / farPoint.w - origin)};
}

std::optional<PickHit> pickEntity(const scene::Scene& scene, const Renderer& renderer, const Ray& ray)
{
    std::optional<PickHit> best;
    const auto view = scene.registry().view<const scene::TransformComponent, const scene::MeshRendererComponent>();
    for (const entt::entity entity : view) {
        const auto& [transform, meshRenderer] = view.get(entity);
        const GpuMesh* mesh = renderer.mesh(meshRenderer.mesh);
        if (!mesh || !meshRenderer.visible || mesh->indices.empty()) {
            continue;
        }

        // Test in the mesh's local space; t stays comparable because the transform is affine
        // and the local direction is not renormalized.
        const glm::mat4 toLocal = glm::inverse(transform.matrix());
        const Ray local{glm::vec3(toLocal * glm::vec4(ray.origin, 1.0f)), glm::mat3(toLocal) * ray.direction};
        const std::span<const glm::vec3> positions = currentPositions(scene, entity, *mesh);

        const bool posed = positions.data() != mesh->positions.data();
        if (!posed) { // the stored bounds only fit the unposed mesh
            const auto boxHit = intersectAabb(local, mesh->boundsMin, mesh->boundsMax);
            if (!boxHit || (best && *boxHit > best->distance)) {
                continue;
            }
        }
        for (usize i = 0; i + 2 < mesh->indices.size(); i += 3) {
            const auto t = intersectTriangle(local, positions[mesh->indices[i]], positions[mesh->indices[i + 1]],
                                             positions[mesh->indices[i + 2]]);
            if (t && (!best || *t < best->distance)) {
                best = PickHit{entity, *t};
            }
        }
    }
    return best;
}

bool worldBounds(const scene::Scene& scene, const Renderer& renderer, entt::entity entity, glm::vec3& outMin,
                 glm::vec3& outMax)
{
    const auto& registry = scene.registry();
    if (!registry.valid(entity)) {
        return false;
    }
    const auto* meshRenderer = registry.try_get<scene::MeshRendererComponent>(entity);
    const auto* transform = registry.try_get<scene::TransformComponent>(entity);
    const GpuMesh* mesh = meshRenderer ? renderer.mesh(meshRenderer->mesh) : nullptr;
    if (!mesh || !transform || !meshRenderer->visible || mesh->positions.empty()) {
        return false;
    }
    const glm::mat4 model = transform->matrix();
    outMin = glm::vec3(INFINITY);
    outMax = glm::vec3(-INFINITY);
    for (const glm::vec3& p : currentPositions(scene, entity, *mesh)) {
        const glm::vec3 w(model * glm::vec4(p, 1.0f));
        outMin = glm::min(outMin, w);
        outMax = glm::max(outMax, w);
    }
    return true;
}

void appendBoxLines(std::vector<DebugVertex>& out, const glm::vec3& boxMin, const glm::vec3& boxMax,
                    const glm::vec3& color)
{
    const auto corner = [&](int i) {
        return glm::vec3(i & 1 ? boxMax.x : boxMin.x, i & 2 ? boxMax.y : boxMin.y, i & 4 ? boxMax.z : boxMin.z);
    };
    // Corners differing in exactly one bit share an edge.
    for (int a = 0; a < 8; ++a) {
        for (int bit = 1; bit < 8; bit <<= 1) {
            if (!(a & bit)) {
                out.push_back({corner(a), color});
                out.push_back({corner(a | bit), color});
            }
        }
    }
}

} // namespace kinetic::renderer
