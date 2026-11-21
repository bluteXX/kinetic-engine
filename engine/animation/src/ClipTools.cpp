#include "kinetic/animation/ClipTools.h"

#include "kinetic/animation/Combat.h"
#include "kinetic/animation/Locomotion.h"
#include "kinetic/animation/PoseOps.h"

#include <glm/gtx/matrix_decompose.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace kinetic::anim {

namespace {

constexpr f32 kPi = 3.14159265f;
constexpr f32 kResampleRate = 60.0f; // keys per second when a clip has to be resampled

glm::quat sameHemisphere(const glm::quat& reference, glm::quat q) { return glm::dot(reference, q) < 0.0f ? -q : q; }

void modelPose(const Skeleton& skeleton, const AnimationClip& clip, f32 time, Pose& pose, std::vector<glm::mat4>& model)
{
    pose = skeleton.restPose();
    clip.sample(time, false, pose);
    model.resize(skeleton.jointCount());
    localToModel(skeleton, pose, model);
}

glm::quat rotationOfMatrix(const glm::mat4& m) { return rotationOf(m); }

f32 uniformScale(const glm::mat4& m) { return glm::length(glm::vec3(m[0])); }

// Yaw (about +Y) of a model-space rotation relative to the rest rotation, radians.
f32 yawOf(const glm::quat& rotation, const glm::quat& rest)
{
    const glm::vec3 v = (rotation * glm::inverse(rest)) * glm::vec3(0.0f, 0.0f, 1.0f);
    return std::atan2(v.x, v.z);
}

std::vector<f32> sampleTimes(f32 start, f32 end)
{
    const usize n = std::max<usize>(2, static_cast<usize>(std::ceil((end - start) * kResampleRate)) + 1);
    std::vector<f32> times(n);
    for (usize i = 0; i < n; ++i) {
        times[i] = start + (end - start) * static_cast<f32>(i) / static_cast<f32>(n - 1);
    }
    return times;
}

// Rest-pose model matrices.
std::vector<glm::mat4> restModel(const Skeleton& skeleton)
{
    std::vector<glm::mat4> model(skeleton.jointCount());
    localToModel(skeleton, skeleton.restPose(), model);
    return model;
}

std::string replaceAll(std::string s, const std::string& from, const std::string& to)
{
    for (usize at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size())) {
        s.replace(at, from.size(), to);
    }
    return s;
}

} // namespace

f32 centerRootXZ(const Skeleton& skeleton, AnimationClip& clip, usize joint)
{
    if (joint >= clip.jointCount() || clip.tracks()[joint].translation.empty()) {
        return 0.0f;
    }
    Pose pose;
    std::vector<glm::mat4> model;
    glm::vec3 mean(0.0f);
    const std::vector<f32> times = sampleTimes(0.0f, clip.duration());
    for (const f32 t : times) {
        modelPose(skeleton, clip, t, pose, model);
        mean += glm::vec3(model[joint][3]) / static_cast<f32>(times.size());
    }
    const std::vector<glm::mat4> rest = restModel(skeleton);
    const glm::vec3 shift(rest[joint][3].x - mean.x, 0.0f, rest[joint][3].z - mean.z);
    // Into the parent's space (the parent is not animated for roots / pelvises).
    const i32 parent = skeleton.parents()[joint];
    const glm::mat3 parentModel = parent >= 0 ? glm::mat3(rest[static_cast<usize>(parent)]) : glm::mat3(1.0f);
    const glm::vec3 local = glm::inverse(parentModel) * shift;
    for (glm::vec3& v : clip.tracks()[joint].translation.values) {
        v += local;
    }
    return glm::length(shift);
}

f32 makeLoopPose(const Skeleton& skeleton, AnimationClip& clip)
{
    Pose pose;
    std::vector<glm::mat4> first;
    std::vector<glm::mat4> last;
    modelPose(skeleton, clip, 0.0f, pose, first);
    modelPose(skeleton, clip, clip.duration(), pose, last);
    f32 gap = 0.0f;
    for (usize j = 0; j < first.size(); ++j) {
        gap = std::max(gap, glm::length(glm::vec3(first[j][3] - last[j][3])));
    }

    for (JointTracks& jt : clip.tracks()) {
        if (jt.translation.times.size() > 1) {
            auto& t = jt.translation;
            const f32 span = t.times.back() - t.times.front();
            const glm::vec3 diff = t.values.front() - t.values.back();
            for (usize k = 0; k < t.values.size() && span > 0.0f; ++k) {
                t.values[k] += diff * ((t.times[k] - t.times.front()) / span);
            }
        }
        if (jt.rotation.times.size() > 1) {
            auto& r = jt.rotation;
            const f32 span = r.times.back() - r.times.front();
            const glm::quat end = sameHemisphere(r.values.front(), r.values.back());
            const glm::quat error = glm::normalize(r.values.front() * glm::inverse(end));
            for (usize k = 0; k < r.values.size() && span > 0.0f; ++k) {
                const f32 w = (r.times[k] - r.times.front()) / span;
                r.values[k] = glm::normalize(glm::slerp(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), error, w) * r.values[k]);
            }
        }
        if (jt.scale.times.size() > 1) {
            auto& s = jt.scale;
            const f32 span = s.times.back() - s.times.front();
            const glm::vec3 diff = s.values.front() - s.values.back();
            for (usize k = 0; k < s.values.size() && span > 0.0f; ++k) {
                s.values[k] += diff * ((s.times[k] - s.times.front()) / span);
            }
        }
    }
    return gap;
}

AnimationClip makeSubClip(const AnimationClip& clip, f32 start, f32 end, std::string name)
{
    start = std::clamp(start, 0.0f, clip.duration());
    end = std::clamp(end, start + 1e-3f, std::max(clip.duration(), start + 1e-3f));
    std::vector<JointTracks> tracks(clip.jointCount());
    Pose pose(clip.jointCount());
    const std::vector<f32> times = sampleTimes(start, end);
    for (const f32 t : times) {
        pose.setIdentity();
        clip.sample(t, false, pose);
        for (usize j = 0; j < clip.jointCount(); ++j) {
            const JointTracks& src = clip.tracks()[j];
            JointTracks& dst = tracks[j];
            if (!src.translation.empty()) {
                dst.translation.times.push_back(t - start);
                dst.translation.values.push_back(pose.translations[j]);
            }
            if (!src.rotation.empty()) {
                dst.rotation.times.push_back(t - start);
                dst.rotation.values.push_back(pose.rotations[j]);
            }
            if (!src.scale.empty()) {
                dst.scale.times.push_back(t - start);
                dst.scale.values.push_back(pose.scales[j]);
            }
        }
    }
    AnimationClip result(std::move(name), end - start, std::move(tracks));
    result.setKind(clip.kind());
    return result;
}

f32 extractRootMotion(const Skeleton& skeleton, AnimationClip& clip, usize joint)
{
    if (joint >= clip.jointCount() || clip.tracks()[joint].translation.empty()) {
        return 0.0f;
    }
    const std::vector<glm::mat4> rest = restModel(skeleton);
    const i32 parent = skeleton.parents()[joint];
    const glm::mat3 parentModel = parent >= 0 ? glm::mat3(rest[static_cast<usize>(parent)]) : glm::mat3(1.0f);
    const glm::mat3 toLocal = glm::inverse(parentModel);

    auto& track = clip.tracks()[joint].translation;
    Pose pose;
    std::vector<glm::mat4> model;
    Track<glm::vec3> motion;
    glm::vec3 start(0.0f);
    for (usize k = 0; k < track.times.size(); ++k) {
        modelPose(skeleton, clip, track.times[k], pose, model);
        const glm::vec3 p(model[joint][3]);
        if (k == 0) start = p;
        motion.times.push_back(track.times[k]);
        motion.values.push_back(glm::vec3(p.x - start.x, 0.0f, p.z - start.z));
    }
    for (usize k = 0; k < track.values.size(); ++k) {
        track.values[k] -= toLocal * motion.values[k];
    }
    const f32 distance = glm::length(motion.values.back());
    clip.setRootMotion(std::move(motion));
    return distance;
}

std::vector<f32> rootYawCurve(const Skeleton& skeleton, const AnimationClip& clip, usize joint, usize samples)
{
    const std::vector<glm::mat4> rest = restModel(skeleton);
    const glm::quat restRotation = rotationOfMatrix(rest[joint]);
    std::vector<f32> yaw(samples + 1);
    Pose pose;
    std::vector<glm::mat4> model;
    for (usize i = 0; i <= samples; ++i) {
        modelPose(skeleton, clip, clip.duration() * static_cast<f32>(i) / static_cast<f32>(samples), pose, model);
        yaw[i] = yawOf(rotationOfMatrix(model[joint]), restRotation);
        if (i > 0) { // unwrap
            while (yaw[i] - yaw[i - 1] > kPi) yaw[i] -= 2.0f * kPi;
            while (yaw[i] - yaw[i - 1] < -kPi) yaw[i] += 2.0f * kPi;
        }
    }
    return yaw;
}

f32 extractRootYaw(const Skeleton& skeleton, AnimationClip& clip, usize joint)
{
    if (joint >= clip.jointCount() || clip.tracks()[joint].rotation.empty() || clip.duration() <= 0.0f) {
        return 0.0f;
    }
    const std::vector<glm::mat4> rest = restModel(skeleton);
    const glm::quat restRotation = rotationOfMatrix(rest[joint]);
    const i32 parent = skeleton.parents()[joint];
    const glm::quat parentRotation = parent >= 0 ? rotationOfMatrix(rest[static_cast<usize>(parent)]) : glm::quat(1, 0, 0, 0);

    // Yaw at every rotation key (unwrapped), relative to the first one.
    auto& track = clip.tracks()[joint].rotation;
    Pose pose;
    std::vector<glm::mat4> model;
    std::vector<f32> yaw(track.times.size());
    for (usize k = 0; k < track.times.size(); ++k) {
        modelPose(skeleton, clip, track.times[k], pose, model);
        yaw[k] = yawOf(rotationOfMatrix(model[joint]), restRotation);
        if (k > 0) {
            while (yaw[k] - yaw[k - 1] > kPi) yaw[k] -= 2.0f * kPi;
            while (yaw[k] - yaw[k - 1] < -kPi) yaw[k] += 2.0f * kPi;
        }
    }
    for (usize k = 0; k < track.values.size(); ++k) {
        // model' = Ry(-delta) * model  ->  local' = parent^-1 * Ry(-delta) * parent * local
        const glm::quat undo = glm::angleAxis(-(yaw[k] - yaw[0]), glm::vec3(0.0f, 1.0f, 0.0f));
        track.values[k] = glm::normalize(glm::inverse(parentRotation) * undo * parentRotation * track.values[k]);
    }
    const f32 span = track.times.back() - track.times.front();
    const f32 rate = span > 0.0f ? (yaw.back() - yaw.front()) / span : 0.0f;
    clip.setRootYawRate(rate);
    return rate;
}

std::optional<std::pair<f32, f32>> findTurnSegment(const Skeleton& skeleton, const AnimationClip& clip, usize joint,
                                                   f32 minAngle)
{
    constexpr usize kSamples = 120;
    const std::vector<f32> yaw = rootYawCurve(skeleton, clip, joint, kSamples);
    if (std::abs(yaw.back() - yaw.front()) < minAngle) {
        return std::nullopt;
    }
    std::vector<f32> rate(kSamples);
    f32 peak = 0.0f;
    for (usize i = 0; i < kSamples; ++i) {
        rate[i] = std::abs(yaw[i + 1] - yaw[i]);
        peak = std::max(peak, rate[i]);
    }
    usize first = kSamples;
    usize last = 0;
    for (usize i = 0; i < kSamples; ++i) {
        if (rate[i] > peak * 0.25f) {
            first = std::min(first, i);
            last = std::max(last, i + 1);
        }
    }
    if (first >= last) {
        return std::nullopt;
    }
    const f32 dt = clip.duration() / static_cast<f32>(kSamples);
    return std::pair{static_cast<f32>(first) * dt, static_cast<f32>(last) * dt};
}

std::vector<usize> findMirrorJoints(const Skeleton& skeleton)
{
    static const std::pair<std::string, std::string> kPairs[] = {
        {"Left", "Right"}, {"left", "right"}, {"LEFT", "RIGHT"}, {"_l", "_r"}, {"_L", "_R"},
        {".l", ".r"},      {".L", ".R"},      {"L_", "R_"},      {"l_", "r_"},
    };
    std::vector<usize> mirror(skeleton.jointCount());
    for (usize j = 0; j < skeleton.jointCount(); ++j) {
        mirror[j] = j;
        const std::string& name = skeleton.names()[j];
        for (const auto& [a, b] : kPairs) {
            for (const auto& [from, to] : {std::pair{a, b}, std::pair{b, a}}) {
                if (name.find(from) == std::string::npos) {
                    continue;
                }
                if (const auto other = skeleton.findJoint(replaceAll(name, from, to)); other && *other != j) {
                    mirror[j] = *other;
                }
            }
            if (mirror[j] != j) {
                break;
            }
        }
    }
    return mirror;
}

AnimationClip mirrorClip(const Skeleton& skeleton, const AnimationClip& clip, std::string name)
{
    const usize n = skeleton.jointCount();
    const std::vector<usize> mirror = findMirrorJoints(skeleton);
    const std::vector<glm::mat4> rest = restModel(skeleton);
    const glm::mat3 s(glm::vec3(-1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
    const auto reflect = [&](const glm::quat& q) { return glm::normalize(glm::quat_cast(s * glm::mat3_cast(q) * s)); };

    // Per joint: correction so the mirrored rest rotation equals the joint's own rest rotation.
    std::vector<glm::quat> correction(n);
    std::vector<glm::quat> restRot(n);
    for (usize j = 0; j < n; ++j) {
        restRot[j] = rotationOfMatrix(rest[j]);
    }
    for (usize j = 0; j < n; ++j) {
        correction[j] = glm::normalize(glm::inverse(reflect(restRot[mirror[j]])) * restRot[j]);
    }

    std::vector<JointTracks> tracks(n);
    Pose pose;
    std::vector<glm::mat4> model;
    std::vector<glm::mat4> mirrored(n);
    for (const f32 t : sampleTimes(0.0f, clip.duration())) {
        modelPose(skeleton, clip, t, pose, model);
        for (usize j = 0; j < n; ++j) {
            const usize m = mirror[j];
            const glm::quat r = glm::normalize(reflect(rotationOfMatrix(model[m])) * correction[j]);
            const glm::vec3 p = s * glm::vec3(model[m][3]);
            const f32 scale = uniformScale(model[m]);
            mirrored[j] = glm::translate(glm::mat4(1.0f), p) * glm::mat4_cast(r) * glm::scale(glm::mat4(1.0f), glm::vec3(scale));
        }
        for (usize j = 0; j < n; ++j) {
            const JointTracks& a = clip.tracks()[j];
            const JointTracks& b = clip.tracks()[mirror[j]];
            const bool animated = !a.rotation.empty() || !a.translation.empty() || !b.rotation.empty() || !b.translation.empty();
            if (!animated) {
                continue;
            }
            const i32 parent = skeleton.parents()[j];
            const glm::mat4 local = parent >= 0 ? glm::inverse(mirrored[static_cast<usize>(parent)]) * mirrored[j] : mirrored[j];
            glm::vec3 scale;
            glm::quat rotation;
            glm::vec3 translation;
            glm::vec3 skew;
            glm::vec4 perspective;
            glm::decompose(local, scale, rotation, translation, skew, perspective);
            JointTracks& out = tracks[j];
            if (!out.rotation.values.empty()) {
                rotation = sameHemisphere(out.rotation.values.back(), rotation);
            }
            out.rotation.times.push_back(t);
            out.rotation.values.push_back(glm::normalize(rotation));
            out.translation.times.push_back(t);
            out.translation.values.push_back(translation);
        }
    }
    AnimationClip result(std::move(name), clip.duration(), std::move(tracks));
    result.setKind(clip.kind());
    result.setRootYawRate(-clip.rootYawRate()); // mirrored: turns the other way
    return result;
}

LocomotionPrepReport prepareLocomotionClips(SkinnedModel& model)
{
    LocomotionPrepReport report;
    const auto legs = findLegChains(model.skeleton);
    const i32 pelvis = findPelvis(model.skeleton, legs);
    if (pelvis < 0) {
        report.lines.push_back("no legs / pelvis found: clips left as they are");
        return report;
    }
    const auto p = static_cast<usize>(pelvis);
    char line[160];
    for (const LocomotionClip& c : findLocomotionClips(model.skeleton, model.clips, legs, 1.0f)) {
        AnimationClip& clip = model.clips[static_cast<usize>(c.clip)];
        const f32 shift = centerRootXZ(model.skeleton, clip, p);
        const f32 gap = makeLoopPose(model.skeleton, clip);
        std::snprintf(line, sizeof(line), "%s: root centred (%.2f moved), loop closed (%.2f gap)", clip.name().c_str(),
                      static_cast<double>(shift), static_cast<double>(gap));
        report.lines.emplace_back(line);
    }

    // Attacks: their step forward becomes root motion (the controller moves the character), so
    // chaining them does not snap the body back. Combat idles: centred and looped like an idle.
    for (const i32 a : findAttackClips(model.clips)) {
        AnimationClip& clip = model.clips[static_cast<usize>(a)];
        if (clip.hasRootMotion()) {
            continue;
        }
        const f32 travel = extractRootMotion(model.skeleton, clip, p);
        std::snprintf(line, sizeof(line), "%s: root motion extracted (%.2f travel)", clip.name().c_str(), static_cast<double>(travel));
        report.lines.emplace_back(line);
    }
    if (const i32 idle = findCombatIdle(model.clips); idle >= 0) {
        AnimationClip& clip = model.clips[static_cast<usize>(idle)];
        centerRootXZ(model.skeleton, clip, p);
        makeLoopPose(model.skeleton, clip);
    }

    // Turn: the first clip named *turn* (skipped if it was already prepared).
    const bool prepared = std::ranges::any_of(model.clips, [](const AnimationClip& c) {
        return c.name().find("(in place)") != std::string::npos;
    });
    i32 turn = -1;
    for (usize i = 0; i < model.clips.size(); ++i) {
        std::string lowerName = model.clips[i].name();
        std::ranges::transform(lowerName, lowerName.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (lowerName.find("turn") != std::string::npos && lowerName.find("(in place)") == std::string::npos &&
            lowerName.find("(mirror)") == std::string::npos) {
            turn = static_cast<i32>(i);
            break;
        }
    }
    if (turn >= 0 && !prepared) {
        const AnimationClip source = model.clips[static_cast<usize>(turn)];
        if (const auto segment = findTurnSegment(model.skeleton, source, p)) {
            AnimationClip inPlace = makeSubClip(source, segment->first, segment->second, source.name() + " (in place)");
            report.turnRate = extractRootYaw(model.skeleton, inPlace, p);
            centerRootXZ(model.skeleton, inPlace, p);
            makeLoopPose(model.skeleton, inPlace);
            AnimationClip mirrored = mirrorClip(model.skeleton, inPlace, source.name() + " (mirror)");
            model.clips.push_back(std::move(inPlace));
            report.turnClip = static_cast<i32>(model.clips.size() - 1);
            model.clips.push_back(std::move(mirrored));
            report.turnMirrorClip = static_cast<i32>(model.clips.size() - 1);
            std::snprintf(line, sizeof(line), "%s: turning part %.2f-%.2f s cut out, root yaw extracted (%.0f deg/s), mirrored",
                          source.name().c_str(), static_cast<double>(segment->first), static_cast<double>(segment->second),
                          static_cast<double>(glm::degrees(report.turnRate)));
            report.lines.emplace_back(line);
        }
    }
    return report;
}

} // namespace kinetic::anim
