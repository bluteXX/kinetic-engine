#include "kinetic/renderer/AnimationSystem.h"

#include <kinetic/animation/PoseOps.h>

namespace kinetic::renderer {

namespace {

constexpr glm::vec3 kBoneColor{0.95f, 0.80f, 0.30f};
constexpr glm::vec3 kJointColor{1.00f, 0.95f, 0.75f};
constexpr glm::vec3 kHighlightColor{0.30f, 0.90f, 1.00f};

void addCross(std::vector<DebugVertex>& out, const glm::vec3& p, f32 size, const glm::vec3& color)
{
    for (int axis = 0; axis < 3; ++axis) {
        glm::vec3 d(0.0f);
        d[axis] = size;
        out.push_back({p - d, color});
        out.push_back({p + d, color});
    }
}

} // namespace

bool enableAnimation(scene::Scene& scene, Renderer& renderer, entt::entity entity,
                     std::shared_ptr<const anim::SkinnedModel> model)
{
    auto& registry = scene.registry();
    auto* meshRenderer = registry.valid(entity) ? registry.try_get<scene::MeshRendererComponent>(entity) : nullptr;
    if (!model || !meshRenderer) {
        return false;
    }
    disableAnimation(scene, renderer, entity); // switching models: drop the old instance first

    const scene::MeshId instance = renderer.addMesh(model->name, model->skin.mesh, MeshUsage::Dynamic);
    if (instance == Renderer::kInvalidMesh) {
        return false;
    }
    auto& animator = registry.emplace<scene::AnimatorComponent>(entity);
    animator.restMesh = meshRenderer->mesh;
    meshRenderer->mesh = instance;
    animator.model = std::move(model);
    if (!animator.model->clips.empty()) {
        animator.player.play(0);
    }
    return true;
}

void disableAnimation(scene::Scene& scene, Renderer& renderer, entt::entity entity)
{
    auto& registry = scene.registry();
    if (!registry.valid(entity)) {
        return;
    }
    const auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator) {
        return;
    }
    if (auto* meshRenderer = registry.try_get<scene::MeshRendererComponent>(entity)) {
        renderer.removeMesh(meshRenderer->mesh);
        meshRenderer->mesh = animator->restMesh;
    }
    registry.remove<scene::AnimatorComponent>(entity);
}

void removeMesh(scene::Scene& scene, Renderer& renderer, entt::entity entity)
{
    disableAnimation(scene, renderer, entity);
    if (scene.registry().valid(entity)) {
        scene.registry().remove<scene::MeshRendererComponent>(entity);
    }
}

void destroyEntity(scene::Scene& scene, Renderer& renderer, entt::entity entity)
{
    if (!scene.registry().valid(entity)) {
        return;
    }
    disableAnimation(scene, renderer, entity);
    scene.destroyEntity(entity);
}

void updateAnimations(scene::Scene& scene, Renderer& renderer, f32 dt)
{
    auto view = scene.registry().view<scene::AnimatorComponent>();
    for (const entt::entity entity : view) {
        auto& a = view.get<scene::AnimatorComponent>(entity);
        if (!a.model) {
            continue;
        }
        const anim::SkinnedModel& model = *a.model;
        const usize jointCount = model.skeleton.jointCount();

        if (a.bindPose) {
            a.localPose = model.skeleton.restPose();
        } else if (a.externalPose && a.localPose.jointCount() == jointCount) {
            // pose provided externally (physics)
        } else {
            a.player.update(dt, model.clips);
            a.player.evaluate(model.skeleton, model.clips, a.localPose, a.scratchPose);
        }

        a.jointModel.resize(jointCount);
        a.skinMatrices.resize(jointCount);
        anim::localToModel(model.skeleton, a.localPose, a.jointModel);
        anim::computeSkinningMatrices(model.skeleton, a.jointModel, a.skinMatrices);

        const auto* meshRenderer = scene.registry().try_get<scene::MeshRendererComponent>(entity);
        if (!meshRenderer) {
            continue;
        }
        const usize vertexCount = model.skin.mesh.positions.size();
        a.skinnedPositions.resize(vertexCount);
        a.skinnedNormals.resize(vertexCount);
        anim::skinVertices(model.skin, a.skinMatrices, a.skinnedPositions, a.skinnedNormals);
        renderer.updateMeshVertices(meshRenderer->mesh, a.skinnedPositions, a.skinnedNormals);
    }
}

void appendSkeletonLines(const scene::Scene& scene, std::vector<DebugVertex>& out, entt::entity highlightEntity,
                         i32 highlightJoint)
{
    const auto view = scene.registry().view<const scene::AnimatorComponent, const scene::TransformComponent>();
    for (const entt::entity entity : view) {
        const auto& [animator, transform] = view.get(entity);
        appendSkeletonLines(animator, transform.matrix(), out, entity == highlightEntity ? highlightJoint : -1);
    }
}

void appendSkeletonLines(const scene::AnimatorComponent& a, const glm::mat4& world, std::vector<DebugVertex>& out,
                         i32 highlightJoint)
{
    if (!a.model || a.jointModel.size() != a.model->skeleton.jointCount()) {
        return;
    }
    const anim::SkinnedModel& model = *a.model;
    const auto& parents = model.skeleton.parents();

    // Marker size relative to the model, so tiny and huge rigs both stay readable.
    const glm::vec3 scale(glm::length(glm::vec3(world[0])), glm::length(glm::vec3(world[1])), glm::length(glm::vec3(world[2])));
    const glm::vec3 extent = model.restBoundsMax - model.restBoundsMin;
    const f32 markerSize = std::max(glm::length(extent * scale) * 0.012f, 0.004f);

    for (usize j = 0; j < a.jointModel.size(); ++j) {
        if (!model.jointIsBone(j)) {
            continue;
        }
        const glm::vec3 p = glm::vec3(world * a.jointModel[j][3]);
        const bool highlighted = static_cast<i32>(j) == highlightJoint;
        addCross(out, p, highlighted ? markerSize * 2.5f : markerSize, highlighted ? kHighlightColor : kJointColor);

        const i32 parent = parents[j];
        if (parent != anim::kNoParent && model.jointIsBone(static_cast<usize>(parent))) {
            const glm::vec3 q = glm::vec3(world * a.jointModel[static_cast<usize>(parent)][3]);
            const glm::vec3 color = highlighted || parent == highlightJoint ? kHighlightColor : kBoneColor;
            out.push_back({q, color});
            out.push_back({p, color});
        }
    }
}

} // namespace kinetic::renderer
