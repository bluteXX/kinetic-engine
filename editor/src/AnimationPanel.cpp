#include "AnimationPanel.h"

#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kinetic::editor {

namespace {

constexpr ImU32 kTimelineBg = IM_COL32(28, 30, 36, 255);
constexpr ImU32 kTickMinor = IM_COL32(70, 74, 84, 255);
constexpr ImU32 kTickMajor = IM_COL32(120, 125, 138, 255);
constexpr ImU32 kKeyColor = IM_COL32(235, 200, 90, 255);
constexpr ImU32 kPlayhead = IM_COL32(240, 80, 70, 255);

void tooltip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

} // namespace

void AnimationPanel::draw(scene::Scene& scene, entt::entity selected)
{
    ImGui::Begin("Animation");
    auto& registry = scene.registry();
    auto* animator = registry.valid(selected) ? registry.try_get<scene::AnimatorComponent>(selected) : nullptr;

    if (selected != m_jointOwner) {
        m_joint = -1;
        m_jointOwner = selected;
    }
    if (!animator || !animator->model) {
        ImGui::Spacing();
        ImGui::TextDisabled("Select an animated object.");
        ImGui::TextDisabled("Drag an ANIM tile from Assets into the Viewport to add one.");
        ImGui::End();
        return;
    }

    const anim::SkinnedModel& model = *animator->model;
    ImGui::Text("%s", model.name.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("  %zu joints  |  %zu clip(s)  |  %zu triangles", model.skeleton.jointCount(), model.clips.size(),
                        model.skin.mesh.triangleCount());

    if (ImGui::BeginTable("##animation", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("Clips", ImGuiTableColumnFlags_WidthFixed, 210.0f);
        ImGui::TableSetupColumn("Playback", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow();

        // --- Clip list ---------------------------------------------------------------------------
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Clips");
        ImGui::BeginChild("##clips", ImVec2(0, 0), ImGuiChildFlags_None);
        if (model.clips.empty()) {
            ImGui::TextDisabled("(no clips in this file)");
        }
        for (usize i = 0; i < model.clips.size(); ++i) {
            const anim::AnimationClip& clip = model.clips[i];
            const bool current = animator->player.clip() == static_cast<i32>(i);
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(clip.name().c_str(), current, ImGuiSelectableFlags_AllowOverlap)) {
                animator->bindPose = false;
                animator->player.play(static_cast<i32>(i), current ? 0.0f : m_crossfade);
            }
            char duration[32];
            std::snprintf(duration, sizeof(duration), "%.2fs", static_cast<double>(clip.duration()));
            ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - ImGui::CalcTextSize(duration).x);
            ImGui::TextDisabled("%s", duration);
            ImGui::PopID();
        }
        ImGui::EndChild();

        // --- Playback ----------------------------------------------------------------------------
        ImGui::TableSetColumnIndex(1);
        ImGui::BeginChild("##playback", ImVec2(0, 0), ImGuiChildFlags_None);
        drawTransport(*animator);
        drawTimeline(*animator);

        ImGui::Spacing();
        ImGui::Checkbox("Skeleton", &showSkeleton);
        ImGui::SameLine();
        if (auto* mesh = registry.try_get<scene::MeshRendererComponent>(selected)) {
            ImGui::Checkbox("Mesh", &mesh->visible);
            ImGui::SameLine();
        }
        ImGui::Checkbox("Bind pose", &animator->bindPose);
        tooltip("Show the skeleton's rest pose instead of the animation");

        drawJoints(*animator);
        ImGui::EndChild();
        ImGui::EndTable();
    }
    ImGui::End();
}

void AnimationPanel::drawTransport(scene::AnimatorComponent& animator)
{
    anim::AnimationPlayer& player = animator.player;
    const auto clips = animator.clips();
    const anim::AnimationClip* clip =
        player.clip() >= 0 && static_cast<usize>(player.clip()) < clips.size() ? &clips[static_cast<usize>(player.clip())] : nullptr;
    const f32 duration = clip ? clip->duration() : 0.0f;
    const f32 frame = 1.0f / kFrameRate;

    ImGui::BeginDisabled(clip == nullptr);
    const ImVec2 button(34.0f, 0.0f);
    if (ImGui::Button("|<", button)) {
        player.setTime(0.0f, clips);
    }
    tooltip("Go to start");
    ImGui::SameLine();
    if (ImGui::Button("<", button)) {
        player.playing = false;
        player.setTime(player.time() - frame, clips);
    }
    tooltip("Previous frame");
    ImGui::SameLine();
    if (ImGui::Button(player.playing ? "Pause" : "Play", ImVec2(64.0f, 0.0f))) {
        if (!player.playing && !player.loop && player.time() >= duration && player.speed > 0.0f) {
            player.setTime(0.0f, clips); // replay a finished one-shot clip
        }
        player.playing = !player.playing;
        animator.bindPose = false;
    }
    ImGui::SameLine();
    if (ImGui::Button(">", button)) {
        player.playing = false;
        player.setTime(player.time() + frame, clips);
    }
    tooltip("Next frame");
    ImGui::SameLine();
    if (ImGui::Button(">|", button)) {
        player.playing = false;
        player.setTime(duration, clips);
    }
    tooltip("Go to end");
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        player.stop();
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    ImGui::Checkbox("Loop", &player.loop);

    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("Speed", &player.speed, -2.0f, 3.0f, "x%.2f");
    tooltip("Playback rate; negative plays backwards. Ctrl+click to type a value.");
    ImGui::SameLine();
    if (ImGui::SmallButton("1x")) {
        player.speed = 1.0f;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("Crossfade", &m_crossfade, 0.0f, 1.0f, "%.2f s");
    tooltip("Blend time when switching clips");
}

void AnimationPanel::drawTimeline(scene::AnimatorComponent& animator)
{
    anim::AnimationPlayer& player = animator.player;
    const auto clips = animator.clips();
    if (player.clip() < 0 || static_cast<usize>(player.clip()) >= clips.size()) {
        ImGui::TextDisabled("No clip selected.");
        return;
    }
    const anim::AnimationClip& clip = clips[static_cast<usize>(player.clip())];
    const f32 duration = std::max(clip.duration(), 1e-3f);

    ImGui::Spacing();
    constexpr f32 kHeight = 52.0f;
    constexpr f32 kPad = 8.0f; // keeps the 0 s and end markers inside the widget
    const f32 width = std::max(ImGui::GetContentRegionAvail().x, 100.0f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + width, p0.y + kHeight);
    const f32 x0 = p0.x + kPad;
    const f32 x1 = p1.x - kPad;
    const auto timeToX = [&](f32 t) { return x0 + (x1 - x0) * (t / duration); };

    ImGui::InvisibleButton("##timeline", ImVec2(width, kHeight));
    if (ImGui::IsItemActivated()) {
        m_scrubbing = true;
        m_playingBeforeScrub = player.playing;
        animator.bindPose = false;
    }
    if (m_scrubbing && ImGui::IsItemActive()) {
        player.playing = false;
        const f32 t = (ImGui::GetIO().MousePos.x - x0) / (x1 - x0) * duration;
        player.setTime(std::clamp(t, 0.0f, duration), clips);
    }
    if (m_scrubbing && ImGui::IsItemDeactivated()) {
        m_scrubbing = false;
        player.playing = m_playingBeforeScrub;
    }

    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(p0, p1, kTimelineBg, 4.0f);

    // Ruler: pick the smallest "nice" step that leaves room between ticks.
    const f32 pixelsPerSecond = (x1 - x0) / duration;
    constexpr f32 kSteps[] = {1.0f / 30.0f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 30.0f, 60.0f};
    f32 minor = kSteps[std::size(kSteps) - 1];
    for (const f32 step : kSteps) {
        if (step * pixelsPerSecond >= 7.0f) {
            minor = step;
            break;
        }
    }
    f32 major = minor;
    for (const f32 step : kSteps) {
        if (step >= minor && step * pixelsPerSecond >= 60.0f) {
            major = step;
            break;
        }
    }
    const f32 rulerY = p0.y + 16.0f;
    const auto tickCount = static_cast<int>(std::floor(duration / minor + 1e-3f));
    for (int i = 0; i <= tickCount; ++i) {
        const f32 t = static_cast<f32>(i) * minor;
        const f32 x = timeToX(t);
        const f32 ratio = t / major;
        const bool isMajor = std::abs(ratio - std::round(ratio)) < 1e-3f;
        draw->AddLine(ImVec2(x, rulerY - (isMajor ? 7.0f : 3.0f)), ImVec2(x, rulerY), isMajor ? kTickMajor : kTickMinor);
        if (isMajor) {
            char label[16];
            std::snprintf(label, sizeof(label), major < 1.0f ? "%.2f" : "%.0fs", static_cast<double>(t));
            draw->AddText(ImVec2(x + 3.0f, p0.y + 1.0f), kTickMajor, label);
        }
    }
    draw->AddLine(ImVec2(x0, rulerY), ImVec2(x1, rulerY), kTickMinor);

    // Keyframes (union over every joint and channel).
    const f32 keyY = p0.y + 34.0f;
    for (const f32 t : keyTimes(clip)) {
        const f32 x = timeToX(t);
        draw->AddQuadFilled(ImVec2(x, keyY - 4.0f), ImVec2(x + 4.0f, keyY), ImVec2(x, keyY + 4.0f), ImVec2(x - 4.0f, keyY),
                            kKeyColor);
    }

    // Playhead.
    const f32 px = timeToX(std::clamp(player.time(), 0.0f, duration));
    draw->AddLine(ImVec2(px, p0.y), ImVec2(px, p1.y), kPlayhead, 2.0f);
    draw->AddTriangleFilled(ImVec2(px - 5.0f, p0.y), ImVec2(px + 5.0f, p0.y), ImVec2(px, p0.y + 7.0f), kPlayhead);

    if (ImGui::IsItemHovered() && !m_scrubbing) {
        ImGui::SetTooltip("Click or drag to scrub");
    }

    const int frameNumber = static_cast<int>(std::round(player.time() * kFrameRate));
    const int frameCount = static_cast<int>(std::round(duration * kFrameRate));
    ImGui::Text("%.2f / %.2f s", static_cast<double>(player.time()), static_cast<double>(clip.duration()));
    ImGui::SameLine();
    ImGui::TextDisabled("frame %d / %d @ %.0f fps  |  %zu keys", frameNumber, frameCount,
                        static_cast<double>(kFrameRate), keyTimes(clip).size());
    if (player.isFading()) {
        ImGui::SameLine();
        ImGui::TextDisabled("|  crossfade %.0f%%", static_cast<double>(player.fadeWeight() * 100.0f));
    }
}

const std::vector<f32>& AnimationPanel::keyTimes(const anim::AnimationClip& clip)
{
    auto [it, inserted] = m_keyTimes.try_emplace(&clip);
    if (inserted) {
        std::vector<f32>& times = it->second;
        for (const anim::JointTracks& jt : clip.tracks()) {
            times.insert(times.end(), jt.translation.times.begin(), jt.translation.times.end());
            times.insert(times.end(), jt.rotation.times.begin(), jt.rotation.times.end());
            times.insert(times.end(), jt.scale.times.begin(), jt.scale.times.end());
        }
        std::ranges::sort(times);
        // Merge keys closer than half a frame: they would draw on top of each other anyway.
        const auto last = std::unique(times.begin(), times.end(),
                                      [](f32 a, f32 b) { return std::abs(a - b) < 0.5f / kFrameRate; });
        times.erase(last, times.end());
    }
    return it->second;
}

void AnimationPanel::drawJoints(const scene::AnimatorComponent& animator)
{
    const anim::Skeleton& skeleton = animator.model->skeleton;
    const usize n = skeleton.jointCount();
    if (!ImGui::CollapsingHeader("Joints")) {
        return;
    }

    m_children.assign(n, {});
    for (usize j = 0; j < n; ++j) {
        if (const i32 p = skeleton.parents()[j]; p != anim::kNoParent) {
            m_children[static_cast<usize>(p)].push_back(j);
        }
    }

    // Selected joint: its current local transform and position on the model.
    if (m_joint >= 0 && static_cast<usize>(m_joint) < n && animator.localPose.jointCount() == n) {
        const auto j = static_cast<usize>(m_joint);
        const anim::JointTransform t = animator.localPose.get(j);
        const glm::vec3 euler = glm::degrees(glm::eulerAngles(t.rotation));
        ImGui::Text("%s", skeleton.names()[j].c_str());
        ImGui::TextDisabled("local  T %.3f %.3f %.3f   R %.1f %.1f %.1f   S %.2f %.2f %.2f",
                            static_cast<double>(t.translation.x), static_cast<double>(t.translation.y),
                            static_cast<double>(t.translation.z), static_cast<double>(euler.x),
                            static_cast<double>(euler.y), static_cast<double>(euler.z), static_cast<double>(t.scale.x),
                            static_cast<double>(t.scale.y), static_cast<double>(t.scale.z));
        if (j < animator.jointModel.size()) {
            const glm::vec3 p(animator.jointModel[j][3]);
            ImGui::TextDisabled("model  %.3f %.3f %.3f", static_cast<double>(p.x), static_cast<double>(p.y),
                                static_cast<double>(p.z));
        }
        ImGui::Separator();
    } else {
        ImGui::TextDisabled("Click a joint to highlight it in the Viewport.");
    }

    ImGui::BeginChild("##joints", ImVec2(0, std::max(ImGui::GetContentRegionAvail().y, 120.0f)), ImGuiChildFlags_Borders);
    for (usize j = 0; j < n; ++j) {
        if (skeleton.parents()[j] == anim::kNoParent) {
            drawJointNode(animator, j);
        }
    }
    ImGui::EndChild();
}

void AnimationPanel::drawJointNode(const scene::AnimatorComponent& animator, usize joint)
{
    const anim::SkinnedModel& model = *animator.model;
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_DefaultOpen |
                               ImGuiTreeNodeFlags_SpanAvailWidth;
    if (m_children[joint].empty()) {
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    }
    if (static_cast<i32>(joint) == m_joint) {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    if (!model.jointIsBone(joint)) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    }
    const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(joint), flags, "%s", model.skeleton.names()[joint].c_str());
    if (!model.jointIsBone(joint)) {
        ImGui::PopStyleColor();
    }
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
        m_joint = m_joint == static_cast<i32>(joint) ? -1 : static_cast<i32>(joint);
    }
    if (open && !m_children[joint].empty()) {
        for (const usize child : m_children[joint]) {
            drawJointNode(animator, child);
        }
        ImGui::TreePop();
    }
}

} // namespace kinetic::editor
