#include "CharacterSetup.h"

#include <kinetic/animation/BlendTreeIO.h>
#include <kinetic/animation/ClipTools.h>
#include <kinetic/animation/Combat.h>
#include <kinetic/core/Log.h>
#include <kinetic/animation/Locomotion.h>

#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <filesystem>
#include <optional>
#include <cmath>
#include <cstdio>

namespace kinetic::editor {

namespace {

bool beginProperties(const char* id)
{
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX)) {
        return false;
    }
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 84.0f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

void property(const char* label)
{
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ImGui::TableSetColumnIndex(1);
    ImGui::SetNextItemWidth(-FLT_MIN);
}

void tooltip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

std::string textureLabel(const renderer::Renderer& renderer, scene::TextureId id)
{
    const renderer::GpuTexture* t = renderer.texture(id);
    if (!t) {
        return "<missing>";
    }
    if (id == scene::kWhiteTexture || id == scene::kFlatNormalTexture) {
        return "none";
    }
    const std::string file = std::filesystem::path(t->name).filename().string();
    char size[32];
    std::snprintf(size, sizeof(size), "  (%u x %u)", t->width, t->height);
    return file + size;
}

f32 uniformScale(const scene::TransformComponent& t) { return std::abs(t.scale.y) > 1e-6f ? std::abs(t.scale.y) : 1.0f; }

} // namespace

f32 restHeight(const anim::SkinnedModel& model) { return std::max(model.restBoundsMax.y - model.restBoundsMin.y, 1e-4f); }

HeroSetup rebuildLocomotionTree(scene::Scene& scene, entt::entity entity)
{
    auto& registry = scene.registry();
    auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) {
        return {false, "Needs an Animator."};
    }
    const anim::SkinnedModel& model = *animator->model;
    if (animator->legs.empty()) {
        animator->legs = anim::findLegChains(model.skeleton);
    }
    const f32 scale = uniformScale(registry.get<scene::TransformComponent>(entity));
    auto tree = anim::makeLocomotionTree(model.skeleton, model.clips, animator->legs, scale);
    if (!tree) {
        return {false, "No walk / run clip found (clip names should contain idle, walk, run / jog)."};
    }
    // Melee: a Dark-Souls-style combo from the attack clips (if the model has any).
    std::string comboMessage;
    if (const std::vector<i32> attacks = anim::findAttackClips(model.clips); !attacks.empty()) {
        const anim::ComboReport combo = anim::addAttackCombo(*tree, model.skeleton, model.clips, attacks,
                                                             anim::findCombatIdle(model.clips), tree->stateMachine.defaultState);
        comboMessage = "; combo: " + std::to_string(combo.attackStates.size()) + " attacks (LMB)";
        for (const std::string& line : combo.lines) {
            KN_INFO("{}", line);
        }
    }
    animator->blendTree = std::make_shared<anim::BlendTree>(std::move(*tree));
    animator->treeState = {};
    animator->useBlendTree = true;
    animator->bindPose = false;

    const anim::BlendTree& graph = *animator->blendTree;
    const anim::BlendNode& blend = graph.nodes[static_cast<usize>(graph.root)];
    const bool twoD = blend.type == anim::BlendNodeType::Blend2D;
    std::string message = twoD ? "Locomotion (2D: turn x speed):" : "Locomotion:";
    auto* controller = registry.try_get<gameplay::CharacterControllerComponent>(entity);
    f32 maxTurn = 0.0f;
    for (const anim::BlendChild& c : blend.children) {
        const std::string& name = graph.nodes[static_cast<usize>(c.node)].name;
        if (name.find("(turning") != std::string::npos) {
            continue; // the walk / run copies at the ends of the turn axis
        }
        const f32 speed = twoD ? c.thresholdY : c.threshold;
        char part[96];
        if (twoD && std::abs(c.threshold) > 1e-3f) {
            std::snprintf(part, sizeof(part), " %s @ %.0f deg/s", name.c_str(), static_cast<double>(glm::degrees(c.threshold)));
            maxTurn = std::max(maxTurn, std::abs(c.threshold));
        } else {
            std::snprintf(part, sizeof(part), " %s @ %.2f m/s", name.c_str(), static_cast<double>(speed));
        }
        message += part;
        if (controller && name == "Walk") controller->walkSpeed = speed;
        if (controller && name == "Run") controller->runSpeed = speed;
    }
    if (controller && maxTurn > 0.0f) {
        // Turn as fast as the turn clip steps: faster makes the feet slide.
        controller->turnSpeed = std::max(maxTurn, 2.0f);
    }
    if (graph.stateMachine.states.size() > 1) {
        message += "; states:";
        for (const anim::AnimState& st : graph.stateMachine.states) {
            message += " " + st.name;
        }
    }
    return {true, message + comboMessage};
}

HeroSetup makePlayableHero(scene::Scene& scene, entt::entity entity, f32 height)
{
    auto& registry = scene.registry();
    auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) {
        return {false, "Needs an Animator with an animated model."};
    }
    // Locomotion clips need preparing once (centred root, closed loops, in-place turn clips). The
    // editor's asset drawer does it on import; models from elsewhere get a prepared copy here.
    if (!animator->model->locomotionPrepared) {
        auto prepared = std::make_shared<anim::SkinnedModel>(*animator->model);
        anim::prepareLocomotionClips(*prepared);
        prepared->locomotionPrepared = true;
        animator->model = std::move(prepared);
        animator->treeState = {};
    }
    const anim::SkinnedModel& model = *animator->model;
    animator->legs = anim::findLegChains(model.skeleton);
    if (animator->legs.size() < 2) {
        return {false, "Could not find two legs in the skeleton (joints named *Foot / *Ankle)."};
    }

    auto& transform = registry.get<scene::TransformComponent>(entity);
    const f32 oldScale = uniformScale(transform);
    const f32 scale = height / restHeight(model);
    transform.scale = glm::vec3(scale);
    // Keep the feet where they were.
    transform.position.y += model.restBoundsMin.y * (oldScale - scale);

    auto& controller = registry.emplace_or_replace<gameplay::CharacterControllerComponent>(entity);
    controller.height = height;
    controller.radius = std::clamp(height * 0.17f, 0.1f, 1.0f);
    controller.stepHeight = std::clamp(height * 0.22f, 0.1f, 1.0f);

    HeroSetup result = rebuildLocomotionTree(scene, entity);
    animator->footIK.enabled = true;
    animator->footIKState = {};
    if (result.ok) {
        std::string loaded;
        if (loadActorGraph(scene, entity, &loaded)) {
            result.message = "Hero ready. " + loaded + ". Press Play (F5).";
        } else {
            result.message = "Hero ready. " + result.message + ". Press Play (F5).";
        }
    }
    return result;
}

std::string actorFilePath(const anim::SkinnedModel& model)
{
    if (model.sourcePath.empty()) return {};
    std::filesystem::path p(model.sourcePath);
    p.replace_extension(".actor");
    return p.string();
}

Json actorToJson(const scene::Scene& scene, entt::entity entity)
{
    const auto& registry = scene.registry();
    Json out = Json::object();
    const auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) return out;
    const anim::SkinnedModel& model = *animator->model;
    out.set("version", 1);
    out.set("model", model.name);
    if (animator->blendTree) {
        out.set("graph", anim::blendTreeToJson(*animator->blendTree, model.skeleton, model.clips));
    }
    Json weapons = Json::array();
    for (const entt::entity e : registry.view<scene::AttachmentComponent, scene::WeaponComponent>()) {
        const auto& a = registry.get<scene::AttachmentComponent>(e);
        if (a.parent != entity) continue;
        const auto& w = registry.get<scene::WeaponComponent>(e);
        Json j = Json::object();
        j.set("bone", a.joint >= 0 && static_cast<usize>(a.joint) < model.skeleton.jointCount() ? Json(model.skeleton.names()[static_cast<usize>(a.joint)]) : Json());
        j.set("offset", Json(a.offset));
        j.set("halfExtents", w.halfExtents);
        j.set("center", w.center);
        j.set("balance", w.balance);
        j.set("mass", w.mass);
        j.set("sharp", w.sharp);
        weapons.push(std::move(j));
    }
    out.set("weapons", std::move(weapons));
    Json aligned = Json::array();
    for (const anim::AnimationClip& clip : model.clips) {
        if (clip.edgeAligned()) aligned.push(clip.name());
    }
    if (aligned.size() > 0) out.set("edgeAlignedClips", std::move(aligned));
    return out;
}

bool saveActor(const scene::Scene& scene, entt::entity entity, std::string* message)
{
    const auto* animator = scene.registry().try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) return false;
    const std::string path = actorFilePath(*animator->model);
    if (path.empty()) return false;
    std::string error;
    if (!saveJsonFile(path, actorToJson(scene, entity), &error)) {
        if (message) *message = error;
        KN_WARN("Saving actor: {}", error);
        return false;
    }
    if (message) *message = "Saved " + std::filesystem::path(path).filename().string();
    return true;
}

namespace {

std::optional<Json> readActorFile(const anim::SkinnedModel& model)
{
    const std::string path = actorFilePath(model);
    std::error_code ec;
    if (path.empty() || !std::filesystem::exists(path, ec)) return std::nullopt;
    std::string error;
    auto json = loadJsonFile(path, &error);
    if (!json) KN_WARN("Actor file {}: {}", path, error);
    return json;
}

} // namespace

bool loadActorGraph(scene::Scene& scene, entt::entity entity, std::string* message)
{
    auto* animator = scene.registry().try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) return false;
    const auto json = readActorFile(*animator->model);
    if (!json || !(*json)["graph"].isObject()) return false;
    std::string warnings;
    auto tree = anim::blendTreeFromJson((*json)["graph"], animator->model->skeleton, animator->model->clips, &warnings);
    if (!warnings.empty()) KN_WARN("Actor graph: {}", warnings);
    if (!tree) return false;
    animator->blendTree = std::make_shared<anim::BlendTree>(std::move(*tree));
    animator->treeState = {};
    animator->useBlendTree = true;
    animator->bindPose = false;
    if (message) *message = "Graph loaded from " + std::filesystem::path(actorFilePath(*animator->model)).filename().string();
    return true;
}

void loadActorWeapons(scene::Scene& scene, entt::entity entity)
{
    auto& registry = scene.registry();
    const auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model) return;
    const auto json = readActorFile(*animator->model);
    if (!json) return;
    const Json& weapons = (*json)["weapons"];
    usize i = 0;
    for (const entt::entity e : registry.view<scene::AttachmentComponent, scene::WeaponComponent>()) {
        auto& a = registry.get<scene::AttachmentComponent>(e);
        if (a.parent != entity || i >= weapons.size()) continue;
        const Json& j = weapons[i++];
        if (const auto joint = animator->model->skeleton.findJoint(j["bone"].asString())) a.joint = static_cast<i32>(*joint);
        a.offset = j["offset"].asMat4(a.offset);
        auto& w = registry.get<scene::WeaponComponent>(e);
        w.halfExtents = j["halfExtents"].asVec3(w.halfExtents);
        w.center = j["center"].asVec3(w.center);
        w.balance = j["balance"].asVec3(w.balance);
        w.mass = j["mass"].asFloat(w.mass);
        w.sharp = j["sharp"].asBool(w.sharp);
    }
    for (const Json& name : (*json)["edgeAlignedClips"].items()) {
        const auto& clips = registry.get<scene::AnimatorComponent>(entity).model->clips;
        for (usize c = 0; c < clips.size(); ++c) {
            if (clips[c].name() == name.asString()) (void)alignWeaponEdge(scene, entity, static_cast<i32>(c));
        }
    }
}

std::optional<anim::EdgeAlignment> alignWeaponEdge(scene::Scene& scene, entt::entity entity, i32 clip)
{
    auto& registry = scene.registry();
    auto* animator = registry.try_get<scene::AnimatorComponent>(entity);
    if (!animator || !animator->model || clip < 0 || static_cast<usize>(clip) >= animator->model->clips.size() ||
        animator->model->clips[static_cast<usize>(clip)].edgeAligned()) {
        return std::nullopt;
    }
    const scene::AttachmentComponent* weapon = nullptr;
    for (const entt::entity e : registry.view<scene::AttachmentComponent, scene::WeaponComponent>()) {
        const auto& a = registry.get<scene::AttachmentComponent>(e);
        if (a.parent == entity && a.joint >= 0) weapon = &a;
    }
    if (!weapon) return std::nullopt;
    // The middle of the blade, 0.55 m from the grip: the sword is drawn unscaled (metres) while the
    // clip is in model units (the actor's scale).
    const auto& t = registry.get<scene::TransformComponent>(entity);
    const f32 scale = std::abs(t.scale.y) > 1e-6f ? std::abs(t.scale.y) : 1.0f;
    auto copy = std::make_shared<anim::SkinnedModel>(*animator->model);
    anim::AnimationClip& c = copy->clips[static_cast<usize>(clip)];
    const anim::EdgeAlignment report = anim::alignBladeEdge(copy->skeleton, c, weapon->joint, weapon->offset, 0.55f / scale);
    c.setEdgeAligned(true);
    animator->model = std::move(copy);
    return report;
}

void drawMaterialSection(scene::MeshRendererComponent& mesh, const renderer::Renderer& renderer)
{
    if (!beginProperties("##material")) {
        return;
    }
    property("Base colour");
    ImGui::TextUnformatted(textureLabel(renderer, mesh.baseColorTexture).c_str());
    property("Normal map");
    ImGui::TextUnformatted(textureLabel(renderer, mesh.normalTexture).c_str());
    if (mesh.normalTexture != scene::kFlatNormalTexture) {
        property("Normal str.");
        ImGui::SliderFloat("##normalStrength", &mesh.normalStrength, 0.0f, 2.0f, "%.2f");
        tooltip("How strongly the normal map bends the lighting (0 = off)");
        property("DirectX map");
        ImGui::Checkbox("##flipY", &mesh.flipNormalY);
        tooltip("Tick if the normal map is authored for DirectX (green channel points down): bumps look inverted");
    }
    ImGui::EndTable();
}

void drawFootIKSettings(scene::Scene& scene, entt::entity entity)
{
    auto& animator = scene.registry().get<scene::AnimatorComponent>(entity);
    ImGui::SeparatorText("Foot IK");
    anim::FootIKSettings& ik = animator.footIK;
    if (!beginProperties("##footik")) {
        return;
    }
    property("Enabled");
    if (ImGui::Checkbox("##ikEnabled", &ik.enabled)) {
        animator.footIKState = {};
    }
    tooltip("Feet follow the colliders under them (steps, stones, slopes); the pelvis lowers when needed");
    if (ik.enabled) {
        property("Legs");
        if (animator.legs.empty() && animator.model) {
            animator.legs = anim::findLegChains(animator.model->skeleton);
        }
        if (animator.legs.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "none found (needs joints named *Foot)");
        } else {
            std::string names;
            for (const anim::LegChain& leg : animator.legs) {
                names += (names.empty() ? "" : ", ") + animator.model->skeleton.names()[leg.foot];
            }
            ImGui::TextDisabled("%zu: %s", animator.legs.size(), names.c_str());
        }
        property("Probe up");
        ImGui::SliderFloat("##probe", &ik.probeHeight, 0.05f, 1.5f, "%.2f m");
        tooltip("Highest ground (above the character's level) a foot will step onto");
        property("Reach down");
        ImGui::SliderFloat("##down", &ik.maxStepDown, 0.05f, 1.5f, "%.2f m");
        property("Pelvis drop");
        ImGui::SliderFloat("##pelvis", &ik.maxPelvisDrop, 0.0f, 1.0f, "%.2f m");
        property("Foot align");
        ImGui::SliderFloat("##align", &ik.footAlign, 0.0f, 1.0f, "%.2f");
        tooltip("How much planted feet tilt to the slope under them");
        property("Smoothing");
        ImGui::SliderFloat("##smooth", &ik.smoothing, 1.0f, 40.0f, "%.0f /s");
    }
    ImGui::EndTable();
}

void drawCharacterControllerDetails(scene::Scene& scene, entt::entity entity)
{
    auto& registry = scene.registry();
    auto& c = registry.get<gameplay::CharacterControllerComponent>(entity);
    if (beginProperties("##controller")) {
        property("Walk speed");
        ImGui::SliderFloat("##walk", &c.walkSpeed, 0.2f, 4.0f, "%.2f m/s");
        property("Run speed");
        ImGui::SliderFloat("##run", &c.runSpeed, 0.5f, 10.0f, "%.2f m/s");
        property("Accel.");
        ImGui::SliderFloat("##accel", &c.acceleration, 1.0f, 40.0f, "%.1f m/s2");
        property("Turn speed");
        ImGui::SliderFloat("##turn", &c.turnSpeed, 1.0f, 30.0f, "%.1f rad/s");
        tooltip("Faster than the turn clips' rate makes the feet slide while turning");
        property("Jump speed");
        ImGui::SliderFloat("##jump", &c.jumpSpeed, 0.0f, 12.0f, "%.1f m/s");
        property("Radius");
        ImGui::SliderFloat("##radius", &c.radius, 0.05f, 1.5f, "%.2f m");
        property("Height");
        ImGui::SliderFloat("##height", &c.height, 0.3f, 4.0f, "%.2f m");
        property("Step height");
        ImGui::SliderFloat("##step", &c.stepHeight, 0.0f, 1.0f, "%.2f m");
        ImGui::EndTable();
    }
    if (registry.all_of<scene::AnimatorComponent>(entity) && ImGui::SmallButton("Match speeds to clips")) {
        rebuildLocomotionTree(scene, entity);
    }
    tooltip("Rebuild the locomotion graph at the current scale and use its walk / run / turn speeds (no foot sliding)");
    ImGui::TextDisabled("Play (F5): WASD move, Shift run, Space jump. Feeds the graph's\nspeed / turn / grounded / jump parameters.");
}

void appendFootIKLines(const scene::AnimatorComponent& animator, std::vector<renderer::DebugVertex>& out)
{
    if (!animator.footIK.enabled) {
        return;
    }
    const anim::FootIKState& s = animator.footIKState;
    for (usize i = 0; i < s.targets.size() && i < s.grounded.size(); ++i) {
        const glm::vec3 color = s.grounded[i] ? glm::vec3(0.3f, 1.0f, 0.5f) : glm::vec3(1.0f, 0.4f, 0.3f);
        const glm::vec3 t = s.targets[i];
        const f32 r = 0.03f;
        out.push_back({t - glm::vec3(r, 0, 0), color});
        out.push_back({t + glm::vec3(r, 0, 0), color});
        out.push_back({t - glm::vec3(0, 0, r), color});
        out.push_back({t + glm::vec3(0, 0, r), color});
        if (s.grounded[i]) {
            const glm::vec3 p = s.hits[i].point;
            out.push_back({p, color});
            out.push_back({p + s.hits[i].normal * 0.15f, color});
        }
    }
}

} // namespace kinetic::editor
