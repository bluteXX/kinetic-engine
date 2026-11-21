#pragma once

#include "kinetic/core/Types.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

namespace kinetic {

// Small JSON value for engine files (actor graphs, VFX): null, bool, number, string, array,
// object (keys sorted, so files diff cleanly). parse() / dump() round-trip.
class Json {
public:
    enum class Type : u8 { Null, Bool, Number, String, Array, Object };
    using Array = std::vector<Json>;
    using Object = std::map<std::string, Json, std::less<>>;

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool v) : m_type(Type::Bool), m_bool(v) {}
    Json(int v) : m_type(Type::Number), m_number(v) {}
    Json(i64 v) : m_type(Type::Number), m_number(static_cast<f64>(v)) {}
    Json(u32 v) : m_type(Type::Number), m_number(v) {}
    Json(f32 v) : m_type(Type::Number), m_number(v) {}
    Json(f64 v) : m_type(Type::Number), m_number(v) {}
    Json(const char* v) : m_type(Type::String), m_string(v) {}
    Json(std::string v) : m_type(Type::String), m_string(std::move(v)) {}
    Json(std::string_view v) : m_type(Type::String), m_string(v) {}
    Json(Array v) : m_type(Type::Array), m_array(std::make_shared<Array>(std::move(v))) {}
    Json(Object v) : m_type(Type::Object), m_object(std::make_shared<Object>(std::move(v))) {}
    Json(const glm::vec2& v) : Json(Array{v.x, v.y}) {}
    Json(const glm::vec3& v) : Json(Array{v.x, v.y, v.z}) {}
    Json(const glm::vec4& v) : Json(Array{v.x, v.y, v.z, v.w}) {}
    Json(const glm::mat4& m);

    [[nodiscard]] static Json array() { return Json(Array{}); }
    [[nodiscard]] static Json object() { return Json(Object{}); }

    [[nodiscard]] Type type() const { return m_type; }
    [[nodiscard]] bool isNull() const { return m_type == Type::Null; }
    [[nodiscard]] bool isNumber() const { return m_type == Type::Number; }
    [[nodiscard]] bool isString() const { return m_type == Type::String; }
    [[nodiscard]] bool isArray() const { return m_type == Type::Array; }
    [[nodiscard]] bool isObject() const { return m_type == Type::Object; }

    // Typed reads with a fallback when the value has another type.
    [[nodiscard]] bool asBool(bool fallback = false) const { return m_type == Type::Bool ? m_bool : fallback; }
    [[nodiscard]] f64 asNumber(f64 fallback = 0.0) const { return m_type == Type::Number ? m_number : fallback; }
    [[nodiscard]] f32 asFloat(f32 fallback = 0.0f) const { return m_type == Type::Number ? static_cast<f32>(m_number) : fallback; }
    [[nodiscard]] i32 asInt(i32 fallback = 0) const { return m_type == Type::Number ? static_cast<i32>(m_number) : fallback; }
    [[nodiscard]] const std::string& asString() const;
    [[nodiscard]] glm::vec2 asVec2(const glm::vec2& fallback = glm::vec2(0.0f)) const;
    [[nodiscard]] glm::vec3 asVec3(const glm::vec3& fallback = glm::vec3(0.0f)) const;
    [[nodiscard]] glm::vec4 asVec4(const glm::vec4& fallback = glm::vec4(0.0f)) const;
    [[nodiscard]] glm::mat4 asMat4(const glm::mat4& fallback = glm::mat4(1.0f)) const;

    // Arrays.
    [[nodiscard]] usize size() const;
    [[nodiscard]] const Json& operator[](usize i) const;
    void push(Json value);
    [[nodiscard]] const Array& items() const;

    // Objects: missing keys read as null.
    [[nodiscard]] const Json& operator[](std::string_view key) const;
    [[nodiscard]] bool has(std::string_view key) const;
    void set(std::string key, Json value);
    [[nodiscard]] const Object& members() const;

    // Text. `indent` > 0: pretty-printed.
    [[nodiscard]] std::string dump(int indent = 2) const;
    [[nodiscard]] static std::optional<Json> parse(std::string_view text, std::string* error = nullptr);

private:
    void write(std::string& out, int indent, int depth) const;

    Type m_type = Type::Null;
    bool m_bool = false;
    f64 m_number = 0.0;
    std::string m_string;
    std::shared_ptr<Array> m_array; // copy-on-write is not needed: values are built once
    std::shared_ptr<Object> m_object;
};

// File helpers (UTF-8 text).
[[nodiscard]] std::optional<Json> loadJsonFile(const std::string& path, std::string* error = nullptr);
bool saveJsonFile(const std::string& path, const Json& value, std::string* error = nullptr);

} // namespace kinetic
