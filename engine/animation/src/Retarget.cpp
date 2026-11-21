#include "kinetic/animation/Retarget.h"

#include "kinetic/animation/IK.h"
#include "kinetic/animation/PoseOps.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <unordered_map>

namespace kinetic::anim {

namespace {

std::string normalizedName(const std::string& name)
{
    // Strip "namespace:" / "Armature|" prefixes.
    usize start = 0;
    for (const char sep : {':', '|'}) {
        if (const usize at = name.rfind(sep); at != std::string::npos) {
            start = std::max(start, at + 1);
        }
    }
    std::string r = name.substr(start);
    std::ranges::transform(r, r.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return r;
}

// The joint the legs hang from (by name "hips" / "pelvis" first, else the first joint with two
// or more children).
i32 findHips(const Skeleton& s)
{
    for (usize j = 0; j < s.jointCount(); ++j) {
        const std::string n = normalizedName(s.names()[j]);
        if (n == "hips" || n == "pelvis" || n == "root_hips") {
            return static_cast<i32>(j);
        }
    }
    const auto legs = findLegChains(s);
    return findPelvis(s, legs);
}

} // namespace

std::vector<i32> mapJointsByName(const Skeleton& source, const Skeleton& target)
{
    std::unordered_map<std::string, i32> byName;
    for (usize j = 0; j < source.jointCount(); ++j) {
        byName.emplace(normalizedName(source.names()[j]), static_cast<i32>(j));
    }
    std::vector<i32> map(target.jointCount(), -1);
    for (usize j = 0; j < target.jointCount(); ++j) {
        if (const auto it = byName.find(normalizedName(target.names()[j])); it != byName.end()) {
            map[j] = it->second;
        }
    }
    return map;
}

AnimationClip retargetClip(const Skeleton& source, const AnimationClip& clip, const Skeleton& target, std::string name,
                           const RetargetOptions& options)
{
    const usize nt = target.jointCount();
    const std::vector<i32> map = mapJointsByName(source, target);
    const auto& tParents = target.parents();

    // Rest poses in model space.
    std::vector<glm::mat4> sRest(source.jointCount());
    std::vector<glm::mat4> tRest(nt);
    localToModel(source, source.restPose(), sRest);
    localToModel(target, target.restPose(), tRest);
    std::vector<glm::quat> sRestRot(source.jointCount());
    std::vector<glm::quat> tRestRot(nt);
    for (usize j = 0; j < source.jointCount(); ++j) sRestRot[j] = rotationOf(sRest[j]);
    for (usize j = 0; j < nt; ++j) tRestRot[j] = rotationOf(tRest[j]);

    const i32 sHips = findHips(source);
    const i32 tHips = findHips(target);
    f32 hipScale = 1.0f;
    if (sHips >= 0 && tHips >= 0) {
        const f32 sh = sRest[static_cast<usize>(sHips)][3].y;
        const f32 th = tRest[static_cast<usize>(tHips)][3].y;
        if (std::abs(sh) > 1e-5f) hipScale = th / sh;
    }

    // Only joints that the source animates (or whose ancestors it animates) get tracks.
    std::vector<JointTracks> tracks(nt);
    const usize samples = std::max<usize>(2, static_cast<usize>(std::ceil(clip.duration() * options.sampleRate)) + 1);
    Pose sPose;
    std::vector<glm::mat4> sModel(source.jointCount());
    std::vector<glm::quat> tModelRot(nt);
    std::vector<glm::mat4> tModel(nt);
    for (usize i = 0; i < samples; ++i) {
        const f32 time = clip.duration() * static_cast<f32>(i) / static_cast<f32>(samples - 1);
        sPose = source.restPose();
        clip.sample(time, false, sPose);
        localToModel(source, sPose, sModel);

        Pose tPose = target.restPose();
        for (usize j = 0; j < nt; ++j) {
            const i32 parent = tParents[j];
            const glm::quat parentRot = parent >= 0 ? tModelRot[static_cast<usize>(parent)] : glm::quat(1, 0, 0, 0);
            if (map[j] >= 0) {
                const auto s = static_cast<usize>(map[j]);
                // Model-space change from the source rest pose, applied to the target's rest pose.
                const glm::quat delta = rotationOf(sModel[s]) * glm::inverse(sRestRot[s]);
                tModelRot[j] = glm::normalize(delta * tRestRot[j]);
                tPose.rotations[j] = glm::normalize(glm::inverse(parentRot) * tModelRot[j]);
            } else {
                tModelRot[j] = glm::normalize(parentRot * tPose.rotations[j]);
            }
        }
        // Hips position: rest + the source's offset from its rest, scaled to the target.
        if (options.translateRoot && sHips >= 0 && tHips >= 0) {
            const auto th = static_cast<usize>(tHips);
            const glm::vec3 offset = (glm::vec3(sModel[static_cast<usize>(sHips)][3]) - glm::vec3(sRest[static_cast<usize>(sHips)][3])) * hipScale;
            const glm::vec3 wanted = glm::vec3(tRest[th][3]) + offset;
            const i32 parent = tParents[th];
            localToModel(target, tPose, tModel);
            const glm::mat4 parentModel = parent >= 0 ? tModel[static_cast<usize>(parent)] : glm::mat4(1.0f);
            tPose.translations[th] = glm::vec3(glm::inverse(parentModel) * glm::vec4(wanted, 1.0f));
        }

        for (usize j = 0; j < nt; ++j) {
            if (map[j] < 0) {
                continue;
            }
            auto& r = tracks[j].rotation;
            glm::quat q = tPose.rotations[j];
            if (!r.values.empty() && glm::dot(r.values.back(), q) < 0.0f) q = -q;
            r.times.push_back(time);
            r.values.push_back(q);
        }
        if (options.translateRoot && tHips >= 0) {
            auto& t = tracks[static_cast<usize>(tHips)].translation;
            t.times.push_back(time);
            t.values.push_back(tPose.translations[static_cast<usize>(tHips)]);
        }
    }
    AnimationClip result(std::move(name), clip.duration(), std::move(tracks));
    result.setKind(clip.kind());
    return result;
}

} // namespace kinetic::anim
