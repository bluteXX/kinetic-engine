#include "kinetic/scene/AnimatorSystem.h"

#include "kinetic/scene/SceneQueries.h"

#include <kinetic/animation/PoseOps.h>

#include <algorithm>

namespace kinetic::scene {

namespace {

// Replaces overridden joints (model space, or relative to the anchor joint) and re-derives their
// descendants from the local pose. Joints are ordered parents first. With overrideWeight < 1 each
// overridden joint is blended with its animated matrix.
void applyBoneOverrides(const anim::Skeleton& skeleton, AnimatorComponent& a)
{
    const usize count = skeleton.jointCount();
    if (a.boneOverrides.empty() || a.overrideWeight <= 0.0f || static_cast<usize>(std::max(a.overrideAnchor, -1) + 1) > count) {
        return;
    }
    std::vector<const glm::mat4*> overridden(count, nullptr);
    for (const auto& o : a.boneOverrides) {
        if (o.joint >= 0 && static_cast<usize>(o.joint) < count) overridden[static_cast<usize>(o.joint)] = &o.relative;
    }
    const glm::mat4 anchor = a.overrideAnchor >= 0 ? a.jointModel[static_cast<usize>(a.overrideAnchor)] : glm::mat4(1.0f);
    const f32 w = std::min(a.overrideWeight, 1.0f);
    std::vector<u8> moved(count, 0);
    for (usize j = 0; j < count; ++j) {
        const i32 parent = skeleton.parents()[j];
        const bool parentMoved = parent >= 0 && moved[static_cast<usize>(parent)];
        if (parentMoved) {
            a.jointModel[j] = a.jointModel[static_cast<usize>(parent)] *
                              anim::toMatrix(a.localPose.translations[j], a.localPose.rotations[j], a.localPose.scales[j]);
            moved[j] = 1;
        }
        if (overridden[j]) {
            const glm::mat4 target = anchor * *overridden[j];
            if (w >= 1.0f) {
                a.jointModel[j] = target;
            } else {
                // Blend translation, rotation, scale separately (a matrix lerp would shear).
                const auto split = [](const glm::mat4& m, glm::vec3& t, glm::quat& r, glm::vec3& s) {
                    t = glm::vec3(m[3]);
                    glm::mat3 b(m);
                    for (int i = 0; i < 3; ++i) {
                        s[i] = glm::length(b[i]);
                        if (s[i] > 1e-8f) b[i] /= s[i];
                    }
                    r = glm::normalize(glm::quat_cast(b));
                };
                glm::vec3 t0, t1, s0, s1;
                glm::quat r0, r1;
                split(a.jointModel[j], t0, r0, s0);
                split(target, t1, r1, s1);
                a.jointModel[j] = anim::toMatrix(glm::mix(t0, t1, w), glm::slerp(r0, r1, w), glm::mix(s0, s1, w));
            }
            moved[j] = 1;
        }
    }
}

} // namespace

void updateAnimatorPoses(Scene& scene, f32 dt)
{
    evaluateAnimatorPoses(scene, dt);
    finishAnimatorPoses(scene);
}

void evaluateAnimatorPoses(Scene& scene, f32 frameDt)
{
    auto view = scene.registry().view<AnimatorComponent>();
    for (const entt::entity entity : view) {
        auto& a = view.get<AnimatorComponent>(entity);
        if (!a.model) {
            continue;
        }
        const anim::SkinnedModel& model = *a.model;
        const usize jointCount = model.skeleton.jointCount();
        const f32 dt = frameDt * std::max(a.timeScale, 0.0f);

        if (a.bindPose) {
            a.localPose = model.skeleton.restPose();
        } else if (a.externalPose && a.localPose.jointCount() == jointCount) {
            // pose provided externally (physics)
        } else if (a.blendTreeActive()) {
            anim::updateBlendTree(*a.blendTree, model.clips, a.treeState, dt);
            anim::evaluateBlendTree(*a.blendTree, model.skeleton, model.clips, a.treeState, a.localPose);
        } else {
            a.player.update(dt, model.clips);
            a.player.evaluate(model.skeleton, model.clips, a.localPose, a.scratchPose);
        }

        // Foot IK on animated poses (not on the bind pose or poses written by physics).
        if (a.footIK.enabled && !a.bindPose && !a.externalPose) {
            if (a.legs.empty()) {
                a.legs = anim::findLegChains(model.skeleton);
            }
            if (const auto* transform = scene.registry().try_get<TransformComponent>(entity)) {
                // The actor's own collider (if any) is not ground for its feet.
                anim::applyFootIK(model.skeleton, a.legs, transform->matrix(), colliderGroundQuery(scene, entity), a.footIK,
                                  a.footIKState, dt, a.localPose);
            }
        }

        a.animatedModel.resize(jointCount);
        anim::localToModel(model.skeleton, a.localPose, a.animatedModel);
    }
}

void finishAnimatorPoses(Scene& scene)
{
    auto view = scene.registry().view<AnimatorComponent>();
    for (const entt::entity entity : view) {
        auto& a = view.get<AnimatorComponent>(entity);
        if (!a.model) {
            continue;
        }
        const anim::Skeleton& skeleton = a.model->skeleton;
        if (a.externalPose && a.localPose.jointCount() == skeleton.jointCount()) {
            // Written after the evaluation (e.g. by a physics character): take the latest.
            a.animatedModel.resize(skeleton.jointCount());
            anim::localToModel(skeleton, a.localPose, a.animatedModel);
        }
        if (a.animatedModel.size() != skeleton.jointCount()) {
            continue;
        }
        a.jointModel = a.animatedModel;
        a.skinMatrices.resize(skeleton.jointCount());
        applyBoneOverrides(skeleton, a);
        anim::computeSkinningMatrices(skeleton, a.jointModel, a.skinMatrices);
    }
}

void updateAttachments(Scene& scene)
{
    auto& registry = scene.registry();
    for (const entt::entity e : registry.view<AttachmentComponent, TransformComponent>()) {
        const auto& a = registry.get<AttachmentComponent>(e);
        if (!registry.valid(a.parent) || a.parent == e) {
            continue;
        }
        const auto* parentTransform = registry.try_get<TransformComponent>(a.parent);
        if (!parentTransform) {
            continue;
        }
        glm::mat4 world = parentTransform->matrix();
        if (const auto* animator = registry.try_get<AnimatorComponent>(a.parent);
            animator && a.joint >= 0 && static_cast<usize>(a.joint) < animator->jointModel.size()) {
            world = world * animator->jointModel[static_cast<usize>(a.joint)];
        }
        world = world * a.offset;
        auto& t = registry.get<TransformComponent>(e);
        t.position = glm::vec3(world[3]);
        glm::mat3 r(world);
        glm::vec3 scale(glm::length(r[0]), glm::length(r[1]), glm::length(r[2]));
        for (int i = 0; i < 3; ++i) {
            if (scale[i] > 1e-8f) r[i] /= scale[i];
        }
        t.rotation = glm::normalize(glm::quat_cast(r));
        if (a.inheritScale) {
            t.scale = scale;
        }
    }
}

} // namespace kinetic::scene
