// Animation Graph tab of the Actor Editor: a Unity-Animator-like state machine canvas, the
// parameter list and the details of the selected state (its motion: clip / 1D / 2D blend with a
// plot) or transition (conditions). Everything edits the actor's own graph live, so the preview
// and the Viewport react immediately.

#include "AnimationEditor.h"
#include "CharacterSetup.h"

#include <kinetic/animation/Combat.h>
#include <kinetic/animation/IK.h>

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace kinetic::editor {

namespace {

constexpr glm::vec2 kStateSize{170.0f, 46.0f};
constexpr glm::vec2 kSmallSize{120.0f, 34.0f};
constexpr i32 kEntry = -1;
constexpr i32 kAny = -3;
constexpr i32 kNone = -2;

constexpr ImU32 kCanvasBg = IM_COL32(32, 34, 39, 255);
constexpr ImU32 kGridMinor = IM_COL32(42, 44, 50, 255);
constexpr ImU32 kGridMajor = IM_COL32(52, 55, 62, 255);
constexpr ImU32 kStateFill = IM_COL32(72, 76, 86, 255);
constexpr ImU32 kDefaultFill = IM_COL32(196, 112, 40, 255);
constexpr ImU32 kEntryFill = IM_COL32(52, 132, 66, 255);
constexpr ImU32 kAnyFill = IM_COL32(38, 124, 136, 255);
constexpr ImU32 kActiveBorder = IM_COL32(90, 175, 255, 255);
constexpr ImU32 kSelectBorder = IM_COL32(255, 255, 255, 255);
constexpr ImU32 kLink = IM_COL32(200, 202, 210, 255);
constexpr ImU32 kLinkSelected = IM_COL32(255, 210, 80, 255);
constexpr ImU32 kLinkActive = IM_COL32(90, 175, 255, 255);
constexpr ImU32 kPlotBg = IM_COL32(28, 30, 36, 255);
constexpr ImU32 kPlotAxis = IM_COL32(80, 84, 96, 255);
constexpr ImU32 kMarker = IM_COL32(235, 200, 90, 255);
constexpr ImU32 kValue = IM_COL32(240, 80, 70, 255);
constexpr ImU32 kWeight = IM_COL32(90, 170, 255, 255);

// Context-menu state of the canvas (one Actor Editor at a time).
struct CanvasMenu {
    i32 target = kNone;     // state / kEntry / kAny the node menu is for
    glm::vec2 position{0};  // graph position for "Create State"
};
CanvasMenu g_menu;

void tooltip(const char* text)
{
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("%s", text);
    }
}

const char* clipName(const scene::AnimatorComponent& animator, i32 clip)
{
    const auto clips = animator.clips();
    return clip >= 0 && static_cast<usize>(clip) < clips.size() ? clips[static_cast<usize>(clip)].name().c_str() : "<no clip>";
}

void inputText(const char* id, std::string& value, f32 width = -FLT_MIN)
{
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), "%s", value.c_str());
    ImGui::SetNextItemWidth(width);
    if (ImGui::InputText(id, buffer, sizeof(buffer))) {
        value = buffer;
    }
}

const char* typeLetter(anim::ParameterType t)
{
    switch (t) {
    case anim::ParameterType::Float: return "F";
    case anim::ParameterType::Bool: return "B";
    case anim::ParameterType::Trigger: return "T";
    }
    return "?";
}

ImVec4 typeColor(anim::ParameterType t)
{
    switch (t) {
    case anim::ParameterType::Float: return {0.45f, 0.72f, 1.0f, 1.0f};
    case anim::ParameterType::Bool: return {0.45f, 0.9f, 0.5f, 1.0f};
    case anim::ParameterType::Trigger: return {1.0f, 0.7f, 0.35f, 1.0f};
    }
    return {1, 1, 1, 1};
}

// Combo of float parameters; returns true when changed.
bool parameterCombo(const char* id, const anim::BlendTree& tree, i32& parameter, bool floatsOnly)
{
    const char* current = parameter >= 0 && static_cast<usize>(parameter) < tree.parameters.size()
                              ? tree.parameters[static_cast<usize>(parameter)].name.c_str()
                              : "<parameter>";
    bool changed = false;
    if (ImGui::BeginCombo(id, current)) {
        for (usize i = 0; i < tree.parameters.size(); ++i) {
            if (floatsOnly && tree.parameters[i].type != anim::ParameterType::Float) {
                continue;
            }
            if (ImGui::Selectable(tree.parameters[i].name.c_str(), parameter == static_cast<i32>(i))) {
                parameter = static_cast<i32>(i);
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

bool clipCombo(const char* id, const scene::AnimatorComponent& animator, i32& clip)
{
    bool changed = false;
    if (ImGui::BeginCombo(id, clipName(animator, clip))) {
        const auto clips = animator.clips();
        for (usize i = 0; i < clips.size(); ++i) {
            if (ImGui::Selectable(clips[i].name().c_str(), clip == static_cast<i32>(i))) {
                clip = static_cast<i32>(i);
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    return changed;
}

// First float parameter (creating `fallback` if there is none other than `except`).
i32 floatParameter(anim::BlendTree& tree, const char* fallback, i32 except = -1)
{
    for (usize i = 0; i < tree.parameters.size(); ++i) {
        if (tree.parameters[i].type == anim::ParameterType::Float && static_cast<i32>(i) != except) {
            return static_cast<i32>(i);
        }
    }
    return tree.addParameter(fallback, 0.0f, 0.0f, 1.0f);
}

void removeParameter(anim::BlendTree& tree, anim::BlendTreeInstance& inst, i32 p)
{
    tree.parameters.erase(tree.parameters.begin() + p);
    if (static_cast<usize>(p) < inst.parameters.size()) {
        inst.parameters.erase(inst.parameters.begin() + p);
    }
    const auto fix = [p](i32& index) { index = index == p ? -1 : (index > p ? index - 1 : index); };
    for (anim::BlendNode& n : tree.nodes) {
        fix(n.parameter);
        fix(n.parameterY);
    }
    for (anim::AnimTransition& t : tree.stateMachine.transitions) {
        std::erase_if(t.conditions, [p](const anim::TransitionCondition& c) { return c.parameter == p; });
        for (anim::TransitionCondition& c : t.conditions) {
            fix(c.parameter);
        }
    }
}

f32 distanceToSegment(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b)
{
    const glm::vec2 ab = b - a;
    const f32 len2 = glm::dot(ab, ab);
    const f32 t = len2 > 0.0f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
    return glm::length(p - (a + ab * t));
}

ImVec2 iv(const glm::vec2& v) { return {v.x, v.y}; }
glm::vec2 gv(const ImVec2& v) { return {v.x, v.y}; }

void arrowLine(ImDrawList* draw, const glm::vec2& a, const glm::vec2& b, ImU32 color, f32 thickness)
{
    draw->AddLine(iv(a), iv(b), color, thickness);
    const glm::vec2 d = b - a;
    const f32 len = glm::length(d);
    if (len < 1e-3f) {
        return;
    }
    const glm::vec2 dir = d / len;
    const glm::vec2 side(-dir.y, dir.x);
    const glm::vec2 mid = (a + b) * 0.5f + dir * 5.0f;
    const f32 s = 7.0f;
    draw->AddTriangleFilled(iv(mid), iv(mid - dir * s * 1.4f + side * s), iv(mid - dir * s * 1.4f - side * s), color);
}

std::string motionLabel(const anim::BlendTree& tree, const scene::AnimatorComponent& animator, i32 node)
{
    if (!tree.validNode(node)) {
        return "no motion";
    }
    const anim::BlendNode& n = tree.nodes[static_cast<usize>(node)];
    switch (n.type) {
    case anim::BlendNodeType::Clip: return clipName(animator, n.clip);
    case anim::BlendNodeType::Blend1D: return "Blend Tree 1D (" + std::to_string(n.children.size()) + ")";
    case anim::BlendNodeType::Blend2D: return "Blend Tree 2D (" + std::to_string(n.children.size()) + ")";
    }
    return "";
}

} // namespace

// ---------------------------------------------------------------------------------------------- //
// Tab

void AnimationEditor::drawGraphTab(scene::Scene& scene, scene::AnimatorComponent& animator)
{
    if (!animator.blendTree) {
        ImGui::Spacing();
        ImGui::TextDisabled("This actor has no animation graph yet.");
        if (ImGui::Button("Create Locomotion Graph")) {
            rebuildLocomotionTree(scene, m_actor);
        }
        tooltip("Idle / walk / run (+ turns as a 2D blend, + jump state) built from the model's clips");
        ImGui::SameLine();
        if (ImGui::Button("Create Empty Graph")) {
            auto graph = std::make_shared<anim::BlendTree>();
            const i32 motion = graph->addClip("Idle", animator.clips().empty() ? -1 : 0);
            graph->addState("Idle", motion, {260.0f, 120.0f});
            animator.blendTree = std::move(graph);
            animator.treeState = {};
            animator.useBlendTree = true;
        }
        return;
    }
    anim::BlendTree& tree = *animator.blendTree;
    anim::BlendTreeInstance& inst = animator.treeState;
    inst.prepare(tree);

    // Toolbar: playing, runtime state.
    if (ImGui::Checkbox("Play graph", &animator.useBlendTree)) {
        animator.bindPose = false;
    }
    tooltip("The actor plays this graph (Play mode feeds speed / turn / grounded / jump from the character controller)");
    ImGui::SameLine();
    if (tree.stateMachine.validState(inst.state)) {
        const anim::AnimState& s = tree.stateMachine.states[static_cast<usize>(inst.state)];
        const f32 duration = anim::blendNodeDuration(tree, animator.clips(), inst, s.motion);
        ImGui::TextDisabled("|  state: %s  %.0f%%", s.name.c_str(),
                            static_cast<double>(std::fmod(inst.stateTime / duration, 1.0f) * 100.0f));
        if (inst.inTransition()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.35f, 0.7f, 1.0f, 1.0f), "<- %s %.0f%%",
                               tree.stateMachine.states[static_cast<usize>(inst.previousState)].name.c_str(),
                               static_cast<double>((1.0f - inst.transitionWeight()) * 100.0f));
        }
    }
    if (!tree.isValid(animator.clips().size())) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "|  incomplete: a state has no clip / parameter");
    }
    ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 150.0f);
    if (ImGui::SmallButton("Rebuild Locomotion")) {
        rebuildLocomotionTree(scene, m_actor);
        m_item = GraphItem::None;
        return;
    }
    tooltip("Replace this graph with a fresh locomotion graph built from the clips");

    // Graph above, details below, with a splitter between them (drag it to give either more room).
    constexpr f32 kSplitter = 10.0f;
    const f32 total = ImGui::GetContentRegionAvail().y;
    const f32 maxDetails = std::max(total - 90.0f, 80.0f);
    const f32 detailsHeight = std::clamp(m_detailsHeight, 80.0f, maxDetails);
    // The graph lives in a child of fixed height: a table alone only treats its height as a
    // minimum, and the canvas (which fills what is left) would push the details out of the window.
    const f32 graphHeight = std::max(total - detailsHeight - kSplitter, 60.0f);
    if (ImGui::BeginChild("##graphArea", ImVec2(0, graphHeight), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
        if (ImGui::BeginTable("##graphLayout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("params", ImGuiTableColumnFlags_WidthFixed, 200.0f);
            ImGui::TableSetupColumn("canvas", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("##params", ImVec2(0, graphHeight - 4.0f));
            drawParameters(animator);
            ImGui::EndChild();
            ImGui::TableSetColumnIndex(1);
            drawCanvas(animator);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
    {
        const ImVec2 a = ImGui::GetCursorScreenPos();
        const f32 width = ImGui::GetContentRegionAvail().x;
        ImGui::InvisibleButton("##splitter", ImVec2(width, kSplitter));
        const bool active = ImGui::IsItemActive();
        if (ImGui::IsItemHovered() || active) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        }
        if (active) {
            m_detailsHeight = std::clamp(detailsHeight - ImGui::GetIO().MouseDelta.y, 80.0f, maxDetails);
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            // Double-click: details large <-> graph large.
            m_detailsHeight = detailsHeight < total * 0.5f ? maxDetails : total * 0.3f;
        }
        const bool hot = active || ImGui::IsItemHovered();
        const f32 y = a.y + kSplitter * 0.5f;
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(ImVec2(a.x, a.y + 1.0f), ImVec2(a.x + width, a.y + kSplitter - 1.0f),
                            hot ? IM_COL32(70, 100, 150, 255) : IM_COL32(48, 51, 58, 255), 2.0f);
        draw->AddRectFilled(ImVec2(a.x + width * 0.5f - 24.0f, y - 1.5f), ImVec2(a.x + width * 0.5f + 24.0f, y + 1.5f),
                            hot ? IM_COL32(200, 220, 255, 255) : IM_COL32(150, 155, 165, 255), 2.0f);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && !active) {
            ImGui::SetTooltip("Drag: resize graph / details\nDouble-click: enlarge the details");
        }
    }
    // Scrolls when the details are taller than the panel.
    ImGui::BeginChild("##graphDetails", ImVec2(0, 0), ImGuiChildFlags_None, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    drawGraphDetails(scene, animator);
    ImGui::EndChild();
}

// ---------------------------------------------------------------------------------------------- //
// Parameters

void AnimationEditor::drawParameters(scene::AnimatorComponent& animator)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::BlendTreeInstance& inst = animator.treeState;
    ImGui::SeparatorText("Parameters");
    i32 remove = -1;
    for (usize i = 0; i < tree.parameters.size(); ++i) {
        anim::BlendParameter& p = tree.parameters[i];
        f32& value = inst.parameters[i];
        ImGui::PushID(static_cast<int>(i));
        ImGui::TextColored(typeColor(p.type), "%s", typeLetter(p.type));
        ImGui::SameLine();
        inputText("##name", p.name, 78.0f);
        if (ImGui::BeginPopupContextItem("##paramMenu")) {
            if (p.type == anim::ParameterType::Trigger) {
                ImGui::SetNextItemWidth(140.0f);
                ImGui::SliderFloat("Buffer", &p.bufferTime, 0.0f, 2.0f, p.bufferTime > 0.0f ? "%.2f s" : "until used");
                ImGui::TextDisabled("How long a press waits for a transition\n(the combo input buffer). 0 = forever.");
            }
            if (p.type == anim::ParameterType::Float) {
                ImGui::SetNextItemWidth(140.0f);
                ImGui::DragFloatRange2("Range", &p.min, &p.max, 0.05f);
                ImGui::SetNextItemWidth(140.0f);
                ImGui::DragFloat("Default", &p.defaultValue, 0.05f);
            }
            if (ImGui::MenuItem("Delete")) {
                remove = static_cast<i32>(i);
            }
            ImGui::EndPopup();
        }
        tooltip("Right-click: range / delete");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        switch (p.type) {
        case anim::ParameterType::Float: ImGui::SliderFloat("##v", &value, p.min, p.max, "%.2f"); break;
        case anim::ParameterType::Bool: {
            bool on = value > 0.5f;
            if (ImGui::Checkbox("##v", &on)) value = on ? 1.0f : 0.0f;
            break;
        }
        case anim::ParameterType::Trigger:
            ImGui::PushStyleColor(ImGuiCol_Button, value > 0.5f ? ImVec4(0.8f, 0.5f, 0.2f, 1.0f) : ImGui::GetStyleColorVec4(ImGuiCol_Button));
            if (ImGui::SmallButton(value > 0.5f ? "set" : "fire")) value = 1.0f;
            ImGui::PopStyleColor();
            break;
        }
        ImGui::PopID();
    }
    if (remove >= 0) {
        removeParameter(tree, inst, remove);
    }
    if (ImGui::SmallButton("+ Parameter")) {
        ImGui::OpenPopup("##newParam");
    }
    if (ImGui::BeginPopup("##newParam")) {
        const auto add = [&](const char* label, anim::ParameterType type) {
            if (ImGui::MenuItem(label)) {
                tree.addParameter(std::string("New ") + label + std::to_string(tree.parameters.size()), 0.0f, 0.0f, 1.0f, type);
            }
        };
        add("Float", anim::ParameterType::Float);
        add("Bool", anim::ParameterType::Bool);
        add("Trigger", anim::ParameterType::Trigger);
        ImGui::EndPopup();
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Values are live: drag them to\npreview the graph. In Play the\ncontroller drives speed / turn /\ngrounded / jump.");
}

// ---------------------------------------------------------------------------------------------- //
// Canvas

void AnimationEditor::drawCanvas(scene::AnimatorComponent& animator)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::StateMachine& sm = tree.stateMachine;
    const anim::BlendTreeInstance& inst = animator.treeState;

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 size(std::max(avail.x, 100.0f), std::max(avail.y, 100.0f));
    const glm::vec2 p0 = gv(ImGui::GetCursorScreenPos());
    ImGui::InvisibleButton("##canvas", ImVec2(size.x, size.y),
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const ImGuiIO& io = ImGui::GetIO();
    const auto toScreen = [&](const glm::vec2& g) { return p0 + (g + m_pan) * m_zoom; };
    const auto toGraph = [&](const glm::vec2& s) { return (s - p0) / m_zoom - m_pan; };
    const glm::vec2 mouse = gv(io.MousePos);
    const glm::vec2 mouseGraph = toGraph(mouse);

    // Pan (MMB, or RMB drag) and zoom around the cursor.
    if (active && (ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f) || ImGui::IsMouseDragging(ImGuiMouseButton_Right, 4.0f))) {
        m_pan += gv(io.MouseDelta) / m_zoom;
    }
    if (hovered && io.MouseWheel != 0.0f) {
        const f32 zoom = std::clamp(m_zoom * std::pow(1.12f, io.MouseWheel), 0.4f, 2.0f);
        m_pan = (mouse - p0) / zoom - mouseGraph;
        m_zoom = zoom;
    }

    // Geometry.
    const auto stateRect = [&](i32 s) -> std::pair<glm::vec2, glm::vec2> {
        if (s == kEntry) return {sm.entryPosition, kSmallSize};
        if (s == kAny) return {sm.anyStatePosition, kSmallSize};
        return {sm.states[static_cast<usize>(s)].position, kStateSize};
    };
    const auto center = [&](i32 s) {
        const auto [pos, sz] = stateRect(s);
        return toScreen(pos + sz * 0.5f);
    };
    const auto hitNode = [&](const glm::vec2& g) -> i32 {
        for (i32 s = static_cast<i32>(sm.states.size()) - 1; s >= 0; --s) {
            const auto [pos, sz] = stateRect(s);
            if (g.x >= pos.x && g.y >= pos.y && g.x <= pos.x + sz.x && g.y <= pos.y + sz.y) return s;
        }
        for (const i32 s : {kEntry, kAny}) {
            const auto [pos, sz] = stateRect(s);
            if (g.x >= pos.x && g.y >= pos.y && g.x <= pos.x + sz.x && g.y <= pos.y + sz.y) return s;
        }
        return kNone;
    };
    // Screen segment of a transition (offset sideways when the reverse one exists too).
    const auto segment = [&](const anim::AnimTransition& t) -> std::pair<glm::vec2, glm::vec2> {
        glm::vec2 a = center(t.from < 0 ? kAny : t.from);
        glm::vec2 b = center(t.to);
        const bool reverse = std::ranges::any_of(sm.transitions, [&](const anim::AnimTransition& o) {
            return o.from == t.to && o.to == t.from;
        });
        if (reverse) {
            const glm::vec2 d = b - a;
            const f32 len = glm::length(d);
            if (len > 1e-3f) {
                const glm::vec2 side = glm::vec2(-d.y, d.x) / len * 7.0f;
                a += side;
                b += side;
            }
        }
        return {a, b};
    };
    const auto hitTransition = [&](const glm::vec2& s) -> i32 {
        for (usize t = 0; t < sm.transitions.size(); ++t) {
            if (!sm.validState(sm.transitions[t].to)) continue;
            const auto [a, b] = segment(sm.transitions[t]);
            if (distanceToSegment(s, a, b) < 6.0f) return static_cast<i32>(t);
        }
        return -1;
    };

    // Mouse.
    if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const i32 node = hitNode(mouseGraph);
        if (m_linkFrom != kNone) {
            if (node >= 0 && node != m_linkFrom) {
                const bool exists = std::ranges::any_of(sm.transitions, [&](const anim::AnimTransition& t) {
                    return t.from == m_linkFrom && t.to == node;
                });
                if (!exists) {
                    m_itemIndex = tree.addTransition(m_linkFrom == kAny ? -1 : m_linkFrom, node, 0.2f);
                    m_item = GraphItem::Transition;
                }
            }
            m_linkFrom = kNone;
        } else if (node != kNone) {
            m_item = node >= 0 ? GraphItem::State : (node == kEntry ? GraphItem::Entry : GraphItem::AnyState);
            m_itemIndex = node;
            m_dragState = node;
            m_selectedChild = -1;
        } else if (const i32 t = hitTransition(mouse); t >= 0) {
            m_item = GraphItem::Transition;
            m_itemIndex = t;
        } else {
            m_item = GraphItem::None;
            m_itemIndex = -1;
        }
    }
    if (m_dragState != kNone && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const glm::vec2 delta = gv(io.MouseDelta) / m_zoom;
        if (m_dragState == kEntry) sm.entryPosition += delta;
        else if (m_dragState == kAny) sm.anyStatePosition += delta;
        else if (sm.validState(m_dragState)) sm.states[static_cast<usize>(m_dragState)].position += delta;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        m_dragState = kNone;
    }
    // Right click (without dragging): node or canvas menu.
    if (hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Right) &&
        glm::length(gv(io.MouseClickedPos[ImGuiMouseButton_Right]) - mouse) < 4.0f) {
        m_linkFrom = kNone;
        g_menu.target = hitNode(mouseGraph);
        g_menu.position = mouseGraph - kStateSize * 0.5f;
        ImGui::OpenPopup(g_menu.target != kNone ? "##nodeMenu" : "##canvasMenu");
    }
    if (hovered && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        m_linkFrom = kNone;
    }
    if ((hovered || m_focused) && ImGui::IsKeyPressed(ImGuiKey_Delete, false) && !ImGui::GetIO().WantTextInput &&
        !ImGui::IsAnyItemActive()) {
        deleteGraphSelection(animator);
    }

    // Draw.
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 c0 = iv(p0);
    const ImVec2 c1(p0.x + size.x, p0.y + size.y);
    draw->PushClipRect(c0, c1, true);
    draw->AddRectFilled(c0, c1, kCanvasBg);
    const f32 grid = 32.0f * m_zoom;
    for (f32 x = std::fmod(m_pan.x * m_zoom, grid); x < size.x; x += grid) {
        const bool major = static_cast<int>(std::round((x - m_pan.x * m_zoom) / grid)) % 4 == 0;
        draw->AddLine(ImVec2(p0.x + x, p0.y), ImVec2(p0.x + x, c1.y), major ? kGridMajor : kGridMinor);
    }
    for (f32 y = std::fmod(m_pan.y * m_zoom, grid); y < size.y; y += grid) {
        const bool major = static_cast<int>(std::round((y - m_pan.y * m_zoom) / grid)) % 4 == 0;
        draw->AddLine(ImVec2(p0.x, p0.y + y), ImVec2(c1.x, p0.y + y), major ? kGridMajor : kGridMinor);
    }

    // Links: entry -> default, transitions, the one being made.
    if (sm.validState(sm.defaultState)) {
        arrowLine(draw, center(kEntry), center(sm.defaultState), IM_COL32(220, 140, 60, 255), 2.0f);
    }
    for (usize t = 0; t < sm.transitions.size(); ++t) {
        const anim::AnimTransition& tr = sm.transitions[t];
        if (!sm.validState(tr.to) || (tr.from >= 0 && !sm.validState(tr.from))) continue;
        const auto [a, b] = segment(tr);
        const bool selected = m_item == GraphItem::Transition && m_itemIndex == static_cast<i32>(t);
        const bool running = inst.inTransition() && inst.state == tr.to && (tr.from < 0 || inst.previousState == tr.from);
        arrowLine(draw, a, b, selected ? kLinkSelected : (running ? kLinkActive : kLink), selected || running ? 3.0f : 2.0f);
    }
    if (m_linkFrom != kNone) {
        arrowLine(draw, center(m_linkFrom), mouse, kLinkSelected, 2.0f);
    }

    // Nodes.
    const auto node = [&](i32 s, const char* title, const std::string& subtitle, ImU32 fill, bool current, f32 progress) {
        const auto [pos, sz] = stateRect(s);
        const glm::vec2 a = toScreen(pos);
        const glm::vec2 b = a + sz * m_zoom;
        const f32 rounding = 6.0f * m_zoom;
        draw->AddRectFilled(iv(a + glm::vec2(3, 4)), iv(b + glm::vec2(3, 4)), IM_COL32(0, 0, 0, 70), rounding);
        draw->AddRectFilled(iv(a), iv(b), fill, rounding);
        if (current && progress >= 0.0f) {
            const f32 h = 4.0f * m_zoom;
            draw->AddRectFilled(ImVec2(a.x + rounding, b.y - h - 3.0f), ImVec2(a.x + rounding + (b.x - a.x - 2 * rounding) * progress, b.y - 3.0f),
                                kActiveBorder, 2.0f);
        }
        const bool selected = (m_item == GraphItem::State && m_itemIndex == s) || (m_item == GraphItem::Entry && s == kEntry) ||
                              (m_item == GraphItem::AnyState && s == kAny);
        if (current) draw->AddRect(iv(a), iv(b), kActiveBorder, rounding, 0, 2.5f);
        if (selected) draw->AddRect(iv(a - glm::vec2(2)), iv(b + glm::vec2(2)), kSelectBorder, rounding, 0, 1.5f);
        draw->PushClipRect(iv(a), iv(b), true);
        const ImVec2 titleSize = ImGui::CalcTextSize(title);
        const f32 titleY = subtitle.empty() ? (a.y + b.y - titleSize.y) * 0.5f : a.y + 6.0f * m_zoom;
        draw->AddText(ImVec2((a.x + b.x - titleSize.x) * 0.5f, titleY), IM_COL32(245, 246, 250, 255), title);
        if (!subtitle.empty()) {
            const ImVec2 subSize = ImGui::CalcTextSize(subtitle.c_str());
            draw->AddText(ImVec2((a.x + b.x - subSize.x) * 0.5f, titleY + titleSize.y + 1.0f), IM_COL32(215, 218, 225, 170),
                          subtitle.c_str());
        }
        draw->PopClipRect();
    };
    node(kEntry, "Entry", "", kEntryFill, false, -1.0f);
    node(kAny, "Any State", "", kAnyFill, false, -1.0f);
    for (usize s = 0; s < sm.states.size(); ++s) {
        const anim::AnimState& st = sm.states[s];
        const bool current = inst.state == static_cast<i32>(s) && animator.blendTreeActive();
        f32 progress = -1.0f;
        if (current) {
            const f32 duration = anim::blendNodeDuration(tree, animator.clips(), inst, st.motion);
            progress = std::fmod(inst.stateTime / duration, 1.0f);
        }
        node(static_cast<i32>(s), st.name.c_str(), motionLabel(tree, animator, st.motion),
             static_cast<i32>(s) == sm.defaultState ? kDefaultFill : kStateFill, current, progress);
    }
    const char* help = m_linkFrom != kNone ? "Click a state to connect   Esc: cancel"
                                           : "RMB: create / link   drag: move   MMB / RMB drag: pan   wheel: zoom   Del: delete";
    draw->AddText(ImVec2(p0.x + 8.0f, c1.y - ImGui::GetTextLineHeight() - 6.0f), IM_COL32(150, 155, 165, 220), help);
    draw->PopClipRect();

    drawCanvasMenus(animator, mouseGraph);
}

void AnimationEditor::drawCanvasMenus(scene::AnimatorComponent& animator, const glm::vec2& /*mouseGraph*/)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::StateMachine& sm = tree.stateMachine;
    if (ImGui::BeginPopup("##canvasMenu")) {
        ImGui::TextDisabled("Create State");
        ImGui::Separator();
        const auto addState = [&](const std::string& name, i32 motion) {
            m_itemIndex = tree.addState(name, motion, g_menu.position);
            m_item = GraphItem::State;
        };
        if (ImGui::BeginMenu("From Clip")) {
            const auto clips = animator.clips();
            for (usize i = 0; i < clips.size(); ++i) {
                if (ImGui::MenuItem(clips[i].name().c_str())) {
                    addState(clips[i].name(), tree.addClip(clips[i].name(), static_cast<i32>(i)));
                }
            }
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem("Blend Tree 1D")) {
            addState("Blend 1D", tree.addBlend1D("Blend 1D", floatParameter(tree, "speed")));
        }
        if (ImGui::MenuItem("Blend Tree 2D")) {
            const i32 x = floatParameter(tree, "x");
            addState("Blend 2D", tree.addBlend2D("Blend 2D", x, floatParameter(tree, "y", x)));
        }
        if (ImGui::MenuItem("Empty")) {
            addState("New State", tree.addClip("New State", -1));
        }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("##nodeMenu")) {
        const i32 target = g_menu.target;
        if (target != kEntry && ImGui::MenuItem("Make Transition")) {
            m_linkFrom = target;
        }
        if (target >= 0) {
            if (ImGui::MenuItem("Set as Default State", nullptr, sm.defaultState == target)) {
                sm.defaultState = target;
            }
            if (ImGui::MenuItem("Preview This State")) {
                animator.treeState.forceState(tree, target);
                animator.useBlendTree = true;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Delete State")) {
                m_item = GraphItem::State;
                m_itemIndex = target;
                deleteGraphSelection(animator);
            }
        }
        if (target == kEntry) {
            ImGui::TextDisabled("Entry goes to the default (orange) state.");
        }
        ImGui::EndPopup();
    }
}

void AnimationEditor::deleteGraphSelection(scene::AnimatorComponent& animator)
{
    anim::BlendTree& tree = *animator.blendTree;
    if (m_item == GraphItem::State && tree.stateMachine.validState(m_itemIndex)) {
        tree.removeState(m_itemIndex);
        animator.treeState.state = -1;
        animator.treeState.previousState = -1;
    } else if (m_item == GraphItem::Transition && m_itemIndex >= 0 &&
               static_cast<usize>(m_itemIndex) < tree.stateMachine.transitions.size()) {
        tree.stateMachine.transitions.erase(tree.stateMachine.transitions.begin() + m_itemIndex);
    } else {
        return;
    }
    m_item = GraphItem::None;
    m_itemIndex = -1;
}

// ---------------------------------------------------------------------------------------------- //
// Details

void AnimationEditor::drawGraphDetails(const scene::Scene& scene, scene::AnimatorComponent& animator)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::StateMachine& sm = tree.stateMachine;
    switch (m_item) {
    case GraphItem::State:
        if (sm.validState(m_itemIndex)) {
            drawStateDetails(scene, animator, m_itemIndex);
            return;
        }
        break;
    case GraphItem::Transition:
        if (m_itemIndex >= 0 && static_cast<usize>(m_itemIndex) < sm.transitions.size()) {
            drawTransitionDetails(animator, m_itemIndex);
            return;
        }
        break;
    case GraphItem::Entry: {
        ImGui::TextUnformatted("Entry");
        ImGui::TextDisabled("The graph starts in the default state (orange).");
        ImGui::SetNextItemWidth(220.0f);
        const char* current = sm.validState(sm.defaultState) ? sm.states[static_cast<usize>(sm.defaultState)].name.c_str() : "<none>";
        if (ImGui::BeginCombo("Default state", current)) {
            for (usize s = 0; s < sm.states.size(); ++s) {
                if (ImGui::Selectable(sm.states[s].name.c_str(), sm.defaultState == static_cast<i32>(s))) {
                    sm.defaultState = static_cast<i32>(s);
                }
            }
            ImGui::EndCombo();
        }
        return;
    }
    case GraphItem::AnyState:
        ImGui::TextUnformatted("Any State");
        ImGui::TextDisabled("Transitions from here are checked in every state (e.g. \"hit\" -> Stagger).");
        ImGui::TextDisabled("Right-click it > Make Transition.");
        return;
    case GraphItem::None: break;
    }
    ImGui::TextDisabled("Select a state or a transition. Right-click the canvas to create states, right-click a state");
    ImGui::TextDisabled("> Make Transition to connect it. The orange state is where the graph starts.");
}

void AnimationEditor::drawStateDetails(const scene::Scene& scene, scene::AnimatorComponent& animator, i32 state)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::StateMachine& sm = tree.stateMachine;
    anim::AnimState& st = sm.states[static_cast<usize>(state)];

    ImGui::PushID(state);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("State");
    ImGui::SameLine();
    inputText("##stateName", st.name, 180.0f);
    ImGui::SameLine();
    if (sm.defaultState == state) {
        ImGui::TextColored(ImVec4(0.95f, 0.6f, 0.25f, 1.0f), "default");
    } else if (ImGui::SmallButton("Set as default")) {
        sm.defaultState = state;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Preview")) {
        animator.treeState.forceState(tree, state);
        animator.useBlendTree = true;
        animator.bindPose = false;
    }
    tooltip("Jump to this state now");
    ImGui::SameLine();
    ImGui::Checkbox("Lock movement", &st.lockMovement);
    tooltip("A committed action (attack): the character controller stops walking and only applies the clip's root motion");
    if (st.lockMovement) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::SliderFloat("Turn window", &st.turnWindow, 0.0f, 1.0f, "%.2f s");
        tooltip("Seconds at the start of the state during which the character may still turn towards the input");
    }
    // Outgoing transitions.
    ImGui::SameLine();
    ImGui::TextDisabled("  ->");
    for (usize t = 0; t < sm.transitions.size(); ++t) {
        const anim::AnimTransition& tr = sm.transitions[t];
        if (tr.from != state || !sm.validState(tr.to)) continue;
        ImGui::SameLine();
        ImGui::PushID(static_cast<int>(t));
        if (ImGui::SmallButton(sm.states[static_cast<usize>(tr.to)].name.c_str())) {
            m_item = GraphItem::Transition;
            m_itemIndex = static_cast<i32>(t);
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    drawMotionEditor(scene, animator, st.motion);
    ImGui::PopID();
}

void AnimationEditor::drawTransitionDetails(scene::AnimatorComponent& animator, i32 transition)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::StateMachine& sm = tree.stateMachine;
    anim::AnimTransition& tr = sm.transitions[static_cast<usize>(transition)];
    const char* from = tr.from < 0 ? "Any State" : sm.states[static_cast<usize>(tr.from)].name.c_str();
    ImGui::PushID(transition);
    ImGui::Text("Transition  %s  ->  %s", from, sm.states[static_cast<usize>(tr.to)].name.c_str());
    ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 60.0f);
    if (ImGui::SmallButton("Delete")) {
        sm.transitions.erase(sm.transitions.begin() + transition);
        m_item = GraphItem::None;
        ImGui::PopID();
        return;
    }
    ImGui::SetNextItemWidth(200.0f);
    ImGui::SliderFloat("Blend time", &tr.duration, 0.0f, 1.0f, "%.2f s");
    tooltip("Crossfade from the old state to the new one");
    ImGui::Checkbox("Exit time", &tr.hasExitTime);
    tooltip("Only leave once the source state has played this far (0..1 of its cycle; > 1 = more loops)");
    if (tr.hasExitTime) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(160.0f);
        ImGui::SliderFloat("##exit", &tr.exitTime, 0.0f, 2.0f, "%.2f");
    }

    ImGui::SeparatorText("Conditions (all must hold)");
    i32 remove = -1;
    for (usize c = 0; c < tr.conditions.size(); ++c) {
        anim::TransitionCondition& cond = tr.conditions[c];
        ImGui::PushID(static_cast<int>(c));
        ImGui::SetNextItemWidth(140.0f);
        if (parameterCombo("##param", tree, cond.parameter, false)) {
            const auto type = tree.parameters[static_cast<usize>(cond.parameter)].type;
            cond.op = type == anim::ParameterType::Float ? anim::ConditionOp::Greater : anim::ConditionOp::IsTrue;
        }
        const bool validParam = cond.parameter >= 0 && static_cast<usize>(cond.parameter) < tree.parameters.size();
        const anim::ParameterType type = validParam ? tree.parameters[static_cast<usize>(cond.parameter)].type : anim::ParameterType::Float;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(100.0f);
        if (type == anim::ParameterType::Float) {
            int op = cond.op == anim::ConditionOp::Less ? 1 : 0;
            if (ImGui::Combo("##op", &op, "Greater\0Less\0")) {
                cond.op = op == 1 ? anim::ConditionOp::Less : anim::ConditionOp::Greater;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(90.0f);
            ImGui::DragFloat("##value", &cond.value, 0.01f);
        } else if (type == anim::ParameterType::Bool) {
            int op = cond.op == anim::ConditionOp::IsFalse ? 1 : 0;
            if (ImGui::Combo("##op", &op, "true\0false\0")) {
                cond.op = op == 1 ? anim::ConditionOp::IsFalse : anim::ConditionOp::IsTrue;
            }
        } else {
            cond.op = anim::ConditionOp::IsTrue;
            ImGui::TextDisabled("is fired");
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("x")) {
            remove = static_cast<i32>(c);
        }
        ImGui::PopID();
    }
    if (remove >= 0) {
        tr.conditions.erase(tr.conditions.begin() + remove);
    }
    if (ImGui::SmallButton("+ Condition") && !tree.parameters.empty()) {
        const auto type = tree.parameters[0].type;
        tr.conditions.push_back({0, type == anim::ParameterType::Float ? anim::ConditionOp::Greater : anim::ConditionOp::IsTrue, 0.0f});
    }
    if (tr.conditions.empty() && !tr.hasExitTime) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "  needs a condition or an exit time to fire");
    }
    ImGui::PopID();
}

// ---------------------------------------------------------------------------------------------- //
// Motion (the node a state plays)

void AnimationEditor::drawMotionEditor(const scene::Scene& scene, scene::AnimatorComponent& animator, i32 nodeIndex)
{
    anim::BlendTree& tree = *animator.blendTree;
    if (!tree.validNode(nodeIndex)) {
        ImGui::TextDisabled("This state has no motion.");
        return;
    }
    anim::BlendNode& n = tree.nodes[static_cast<usize>(nodeIndex)];

    if (!ImGui::BeginTable("##motion", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        return;
    }
    ImGui::TableSetupColumn("props", ImGuiTableColumnFlags_WidthStretch, 0.55f);
    ImGui::TableSetupColumn("plot", ImGuiTableColumnFlags_WidthStretch, 0.45f);
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);

    // Motion type.
    int type = static_cast<int>(n.type);
    ImGui::SetNextItemWidth(130.0f);
    if (ImGui::Combo("Motion", &type, "Clip\0Blend Tree 1D\0Blend Tree 2D\0")) {
        n.type = static_cast<anim::BlendNodeType>(type);
        if (n.isBlend() && n.parameter < 0) n.parameter = floatParameter(tree, "speed");
        if (n.type == anim::BlendNodeType::Blend2D && n.parameterY < 0) n.parameterY = floatParameter(tree, "turn", n.parameter);
        tree.sortChildren(nodeIndex);
        m_selectedChild = -1;
    }

    if (n.type == anim::BlendNodeType::Clip) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(200.0f);
        clipCombo("##clip", animator, n.clip);
        ImGui::SetNextItemWidth(160.0f);
        ImGui::SliderFloat("Speed", &n.playbackSpeed, -2.0f, 3.0f, "x%.2f");
        ImGui::SameLine();
        ImGui::Checkbox("Loop", &n.loop);
        tooltip("Off for one-shots (jump, attack): the last frame is held; leave the state with an exit-time transition");
        ImGui::TableSetColumnIndex(1);
        drawPhysicsBones(scene, animator, n);
        ImGui::EndTable();
        return;
    }

    const bool twoD = n.type == anim::BlendNodeType::Blend2D;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    parameterCombo(twoD ? "X##px" : "##px", tree, n.parameter, true);
    if (twoD) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        parameterCombo("Y##py", tree, n.parameterY, true);
    }
    ImGui::Checkbox("Sync cycles", &n.syncPhase);
    tooltip("Children share one normalised clock: feet stay in step while walk and run blend");
    if (!twoD) {
        ImGui::SameLine();
        ImGui::Checkbox("Scale playback", &n.scalePlayback);
        tooltip("Past the last threshold, play faster in proportion so the feet keep up");
    }

    // Children.
    const std::vector<f32> weights = anim::blendChildWeights(tree, animator.treeState, nodeIndex);
    i32 removeChild = -1;
    bool resort = false;
    const int columns = twoD ? 6 : 5;
    if (ImGui::BeginTable("##children", columns, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Motion", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(twoD ? "X" : "Threshold", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        if (twoD) ImGui::TableSetupColumn("Y", ImGuiTableColumnFlags_WidthFixed, 64.0f);
        ImGui::TableSetupColumn("Sync", ImGuiTableColumnFlags_WidthFixed, 34.0f);
        ImGui::TableSetupColumn("Weight", ImGuiTableColumnFlags_WidthFixed, 44.0f);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, 18.0f);
        ImGui::TableHeadersRow();
        for (usize i = 0; i < n.children.size(); ++i) {
            anim::BlendChild& c = n.children[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            if (m_selectedChild == static_cast<i32>(i)) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(70, 90, 130, 120));
            }
            ImGui::TableSetColumnIndex(0);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (tree.validNode(c.node) && tree.nodes[static_cast<usize>(c.node)].type == anim::BlendNodeType::Clip) {
                anim::BlendNode& child = tree.nodes[static_cast<usize>(c.node)];
                if (clipCombo("##clip", animator, child.clip)) {
                    child.name = clipName(animator, child.clip);
                }
            } else {
                ImGui::TextUnformatted(tree.validNode(c.node) ? tree.nodes[static_cast<usize>(c.node)].name.c_str() : "?");
            }
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::DragFloat("##x", &c.threshold, 0.01f, -100.0f, 100.0f, "%.2f");
            if (ImGui::IsItemDeactivatedAfterEdit()) resort = true;
            if (twoD) {
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::DragFloat("##y", &c.thresholdY, 0.01f, -100.0f, 100.0f, "%.2f");
            }
            ImGui::TableNextColumn();
            ImGui::Checkbox("##sync", &c.sync);
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%3.0f%%", static_cast<double>((i < weights.size() ? weights[i] : 0.0f) * 100.0f));
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("x")) removeChild = static_cast<i32>(i);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (removeChild >= 0) {
        n.children.erase(n.children.begin() + removeChild);
        m_selectedChild = -1;
    }
    if (resort) {
        tree.sortChildren(nodeIndex);
    }
    if (ImGui::SmallButton("+ Clip")) {
        const f32 next = n.children.empty() ? 0.0f : (twoD ? 0.0f : n.children.back().threshold + 1.0f);
        const i32 child = tree.addClip(animator.clips().empty() ? "Clip" : animator.clips()[0].name(), animator.clips().empty() ? -1 : 0);
        tree.addChild(nodeIndex, child, next, twoD ? 1.0f : 0.0f);
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Speeds from feet")) {
        // Each clip's ground speed (its feet), at this actor's scale -> threshold (1D) or Y (2D).
        if (animator.legs.empty()) animator.legs = anim::findLegChains(animator.model->skeleton);
        const auto* transform = scene.registry().try_get<scene::TransformComponent>(m_actor);
        const f32 scale = transform ? std::abs(transform->scale.y) : 1.0f;
        for (anim::BlendChild& c : n.children) {
            const anim::BlendNode& child = tree.nodes[static_cast<usize>(c.node)];
            if (child.type != anim::BlendNodeType::Clip || child.clip < 0 || static_cast<usize>(child.clip) >= animator.clips().size()) {
                continue;
            }
            const anim::AnimationClip& clip = animator.clips()[static_cast<usize>(child.clip)];
            const f32 speed = std::abs(clip.rootYawRate()) > 0.0f
                                  ? 0.0f
                                  : anim::estimateGroundSpeed(animator.model->skeleton, clip, animator.legs) * scale;
            (twoD ? c.thresholdY : c.threshold) = speed;
            if (twoD && std::abs(clip.rootYawRate()) > 0.0f) c.threshold = clip.rootYawRate();
        }
        tree.sortChildren(nodeIndex);
    }
    tooltip("Measure each clip's ground speed from its feet (m/s at this actor's scale), and turn clips' turn rate - "
            "parameters then match the movement, no foot sliding");

    ImGui::TableSetColumnIndex(1);
    if (twoD) {
        drawBlend2DPlot(animator, nodeIndex);
    } else {
        drawBlend1DPlot(animator, nodeIndex);
    }
    ImGui::EndTable();
}

void AnimationEditor::drawBlend1DPlot(scene::AnimatorComponent& animator, i32 node)
{
    anim::BlendTree& tree = *animator.blendTree;
    const anim::BlendNode& n = tree.nodes[static_cast<usize>(node)];
    if (n.parameter < 0 || static_cast<usize>(n.parameter) >= tree.parameters.size()) {
        ImGui::TextDisabled("Pick a parameter.");
        return;
    }
    animator.treeState.prepare(tree);
    anim::BlendParameter& param = tree.parameters[static_cast<usize>(n.parameter)];
    f32& value = animator.treeState.parameters[static_cast<usize>(n.parameter)];
    f32 lo = param.min;
    f32 hi = param.max;
    for (const anim::BlendChild& c : n.children) {
        lo = std::min(lo, c.threshold);
        hi = std::max(hi, c.threshold);
    }
    if (hi - lo < 1e-3f) hi = lo + 1.0f;

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(ImGui::GetContentRegionAvail().x, 100.0f), 130.0f);
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(p0, p1, kPlotBg, 4.0f);
    const f32 pad = 14.0f;
    const auto xOf = [&](f32 v) { return p0.x + pad + (v - lo) / (hi - lo) * (size.x - 2.0f * pad); };
    const f32 baseY = p1.y - 26.0f;
    const f32 topY = p0.y + 24.0f;
    draw->AddLine(ImVec2(p0.x + pad, baseY), ImVec2(p1.x - pad, baseY), kPlotAxis, 1.0f);
    for (usize i = 0; i < n.children.size(); ++i) {
        const f32 t = n.children[i].threshold;
        const bool first = i == 0;
        const bool last = i + 1 == n.children.size();
        const ImVec2 peak(xOf(t), topY);
        draw->AddLine(ImVec2(first ? p0.x + pad : xOf(n.children[i - 1].threshold), first ? topY : baseY), peak, kWeight, 1.5f);
        draw->AddLine(peak, ImVec2(last ? p1.x - pad : xOf(n.children[i + 1].threshold), last ? topY : baseY), kWeight, 1.5f);
        draw->AddCircleFilled(ImVec2(xOf(t), baseY), 4.0f, kMarker);
        const std::string& name = tree.validNode(n.children[i].node) ? tree.nodes[static_cast<usize>(n.children[i].node)].name : "?";
        char text[96];
        std::snprintf(text, sizeof(text), "%s %.2f", name.c_str(), static_cast<double>(t));
        draw->AddText(ImVec2(xOf(t) - ImGui::CalcTextSize(text).x * 0.5f, baseY + 6.0f), kMarker, text);
    }
    ImGui::InvisibleButton("##plot1d", size);
    if (ImGui::IsItemActive()) {
        value = std::clamp(lo + (ImGui::GetIO().MousePos.x - p0.x - pad) / (size.x - 2.0f * pad) * (hi - lo), param.min,
                           std::max(param.max, hi));
        animator.useBlendTree = true;
        animator.bindPose = false;
    }
    tooltip("Drag to set the parameter and preview the blend");
    const f32 vx = std::clamp(xOf(value), p0.x + pad, p1.x - pad);
    draw->AddLine(ImVec2(vx, topY - 6.0f), ImVec2(vx, baseY), kValue, 2.0f);
    char text[96];
    std::snprintf(text, sizeof(text), "%s = %.2f", param.name.c_str(), static_cast<double>(value));
    draw->AddText(ImVec2(p0.x + 8.0f, p0.y + 4.0f), kValue, text);
}

void AnimationEditor::drawBlend2DPlot(scene::AnimatorComponent& animator, i32 node)
{
    anim::BlendTree& tree = *animator.blendTree;
    anim::BlendNode& n = tree.nodes[static_cast<usize>(node)];
    const auto valid = [&](i32 p) { return p >= 0 && static_cast<usize>(p) < tree.parameters.size(); };
    if (!valid(n.parameter) || !valid(n.parameterY)) {
        ImGui::TextDisabled("Pick the X and Y parameters.");
        return;
    }
    animator.treeState.prepare(tree);
    anim::BlendParameter& px = tree.parameters[static_cast<usize>(n.parameter)];
    anim::BlendParameter& py = tree.parameters[static_cast<usize>(n.parameterY)];
    f32& vx = animator.treeState.parameters[static_cast<usize>(n.parameter)];
    f32& vy = animator.treeState.parameters[static_cast<usize>(n.parameterY)];

    glm::vec2 lo(px.min, py.min);
    glm::vec2 hi(px.max, py.max);
    for (const anim::BlendChild& c : n.children) {
        lo = glm::min(lo, glm::vec2(c.threshold, c.thresholdY));
        hi = glm::max(hi, glm::vec2(c.threshold, c.thresholdY));
    }
    hi = glm::max(hi, lo + glm::vec2(1e-3f));
    const glm::vec2 margin = (hi - lo) * 0.08f;
    lo -= margin;
    hi += margin;

    const f32 side = std::clamp(ImGui::GetContentRegionAvail().x, 140.0f, 300.0f);
    const glm::vec2 p0 = gv(ImGui::GetCursorScreenPos());
    const glm::vec2 p1 = p0 + glm::vec2(side);
    const auto toScreen = [&](const glm::vec2& v) {
        const glm::vec2 t = (v - lo) / (hi - lo);
        return glm::vec2(p0.x + t.x * side, p1.y - t.y * side);
    };
    const auto toValue = [&](const glm::vec2& s) {
        return lo + glm::vec2((s.x - p0.x) / side, (p1.y - s.y) / side) * (hi - lo);
    };
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(iv(p0), iv(p1), kPlotBg, 4.0f);
    // Axes through zero.
    if (lo.x < 0.0f && hi.x > 0.0f) draw->AddLine(iv(toScreen({0.0f, lo.y})), iv(toScreen({0.0f, hi.y})), kPlotAxis);
    if (lo.y < 0.0f && hi.y > 0.0f) draw->AddLine(iv(toScreen({lo.x, 0.0f})), iv(toScreen({hi.x, 0.0f})), kPlotAxis);
    draw->AddText(ImVec2(p1.x - ImGui::CalcTextSize(px.name.c_str()).x - 6.0f, p1.y - ImGui::GetTextLineHeight() - 4.0f),
                  kPlotAxis, px.name.c_str());
    draw->AddText(ImVec2(p0.x + 6.0f, p0.y + 4.0f), kPlotAxis, py.name.c_str());

    ImGui::InvisibleButton("##plot2d", ImVec2(side, side));
    const ImGuiIO& io = ImGui::GetIO();
    const glm::vec2 mouse = gv(io.MousePos);
    if (ImGui::IsItemActivated()) {
        // Grab a sample point (edit it) or the parameter point (preview).
        m_selectedChild = -1;
        for (usize i = 0; i < n.children.size(); ++i) {
            if (glm::length(toScreen({n.children[i].threshold, n.children[i].thresholdY}) - mouse) < 9.0f) {
                m_selectedChild = static_cast<i32>(i);
            }
        }
    }
    if (ImGui::IsItemActive()) {
        const glm::vec2 v = toValue(mouse);
        if (m_selectedChild >= 0 && static_cast<usize>(m_selectedChild) < n.children.size()) {
            n.children[static_cast<usize>(m_selectedChild)].threshold = v.x;
            n.children[static_cast<usize>(m_selectedChild)].thresholdY = v.y;
        } else {
            vx = v.x;
            vy = v.y;
            animator.useBlendTree = true;
            animator.bindPose = false;
        }
    }
    tooltip("Drag empty space: preview at that point.  Drag a dot: move that motion.");

    const std::vector<f32> weights = anim::blendChildWeights(tree, animator.treeState, node);
    for (usize i = 0; i < n.children.size(); ++i) {
        const glm::vec2 s = toScreen({n.children[i].threshold, n.children[i].thresholdY});
        const f32 w = i < weights.size() ? weights[i] : 0.0f;
        if (w > 0.001f) {
            draw->AddCircleFilled(iv(s), 6.0f + 18.0f * w, IM_COL32(90, 170, 255, 60));
        }
        const bool selected = m_selectedChild == static_cast<i32>(i);
        draw->AddCircleFilled(iv(s), selected ? 6.0f : 4.5f, selected ? IM_COL32(255, 255, 255, 255) : kMarker);
        const std::string& name = tree.validNode(n.children[i].node) ? tree.nodes[static_cast<usize>(n.children[i].node)].name : "?";
        draw->AddText(ImVec2(s.x + 7.0f, s.y - 7.0f), kMarker, name.c_str());
    }
    const glm::vec2 v = toScreen(glm::clamp(glm::vec2(vx, vy), lo, hi));
    draw->AddLine(iv(v - glm::vec2(7, 0)), iv(v + glm::vec2(7, 0)), kValue, 2.0f);
    draw->AddLine(iv(v - glm::vec2(0, 7)), iv(v + glm::vec2(0, 7)), kValue, 2.0f);
    char text[96];
    std::snprintf(text, sizeof(text), "(%.2f, %.2f)", static_cast<double>(vx), static_cast<double>(vy));
    draw->AddText(ImVec2(v.x + 8.0f, v.y + 4.0f), kValue, text);
}

void AnimationEditor::drawPhysicsBones(const scene::Scene& scene, scene::AnimatorComponent& animator, anim::BlendNode& n)
{
    if (!animator.model) {
        return;
    }
    const anim::SkinnedModel& model = *animator.model;
    auto it = m_bodyLayouts.find(&model);
    if (it == m_bodyLayouts.end()) {
        it = m_bodyLayouts.emplace(&model, physanim::layoutBody(model)).first;
    }
    const physanim::BodyLayout& layout = it->second;
    const auto& names = model.skeleton.names();

    ImGui::TextUnformatted("Physics bones");
    tooltip("While this clip leads the pose in Play, these bones run on muscles: they follow the clip until they\n"
            "hit something, then the physics decides (a blade bounces off stone, cuts flesh...). The other bones\n"
            "play normally. With the whole body (pelvis included) the character keeps its balance with limited\n"
            "force: something that stops the body - a wall, a stone - knocks it over.");
    // Presets.
    i32 hand = -1;
    for (const entt::entity e : scene.registry().view<scene::AttachmentComponent, scene::WeaponComponent>()) {
        const auto& a = scene.registry().get<scene::AttachmentComponent>(e);
        if (a.parent == m_actor) hand = a.joint;
    }
    if (hand < 0) hand = anim::findWeaponHand(model.skeleton);
    if (ImGui::SmallButton("None")) n.physicsJoints.clear();
    tooltip("Plain animation");
    ImGui::SameLine();
    ImGui::BeginDisabled(hand < 0);
    if (ImGui::SmallButton("Weapon arm")) n.physicsJoints = physanim::weaponArmJoints(layout, hand);
    tooltip("Hand, forearm and upper arm of the hand holding the weapon (attacks)");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::SmallButton("Full body")) n.physicsJoints = physanim::fullBodyJoints(layout);
    tooltip("Every body: the whole character is simulated (leap attacks, falls); it can be knocked over");
    if (!n.physicsJoints.empty()) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::SliderFloat("Strength", &n.physicsStrength, 0.1f, 3.0f, "x%.2f");
        tooltip("Muscle strength: weaker muscles give way more on hits");
    }

    // The bodies as a tree (bones riding on a body - fingers, toes - are not listed).
    std::vector<u8> on(layout.segments.size(), 0);
    for (const i32 j : n.physicsJoints) {
        if (const i32 s = layout.segmentOf(j); s >= 0) on[static_cast<usize>(s)] = 1;
    }
    std::vector<int> depth(layout.segments.size(), 0);
    for (usize i = 0; i < layout.segments.size(); ++i) {
        const i32 p = layout.segments[i].parent;
        depth[i] = p >= 0 ? depth[static_cast<usize>(p)] + 1 : 0;
    }
    bool changed = false;
    if (ImGui::BeginChild("##physicsBones", ImVec2(0.0f, std::max(ImGui::GetContentRegionAvail().y - 4.0f, 120.0f)), ImGuiChildFlags_Borders)) {
        for (usize i = 0; i < layout.segments.size(); ++i) {
            const physanim::BodySegment& s = layout.segments[i];
            ImGui::PushID(static_cast<int>(i));
            ImGui::Indent(static_cast<f32>(depth[i]) * 10.0f);
            bool v = on[i] != 0;
            if (ImGui::Checkbox(names[static_cast<usize>(s.joint)].c_str(), &v)) {
                on[i] = v ? 1 : 0;
                changed = true;
            }
            if (!s.riders.empty() && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                std::string riders;
                for (const i32 r : s.riders) riders += "\n  " + names[static_cast<usize>(r)];
                ImGui::SetTooltip("Carries:%s", riders.c_str());
            }
            ImGui::Unindent(static_cast<f32>(depth[i]) * 10.0f);
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    if (changed) {
        n.physicsJoints.clear();
        for (usize i = 0; i < layout.segments.size(); ++i) {
            if (on[i]) n.physicsJoints.push_back(layout.segments[i].joint);
        }
    }
}

} // namespace kinetic::editor
