#include "kinetic/core/Json.h"

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace kinetic {

namespace {

const Json kNull;
const std::string kEmptyString;
const Json::Array kEmptyArray;
const Json::Object kEmptyObject;

void writeString(std::string& out, const std::string& s)
{
    out += '"';
    for (const char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += c;
            }
        }
    }
    out += '"';
}

struct Parser {
    std::string_view text;
    usize pos = 0;
    std::string error;

    void skip()
    {
        while (pos < text.size()) {
            const char c = text[pos];
            if (c == ' ' || c == '\n' || c == '\r' || c == '\t') {
                ++pos;
            } else if (c == '/' && pos + 1 < text.size() && text[pos + 1] == '/') { // comments, for hand-written files
                while (pos < text.size() && text[pos] != '\n') ++pos;
            } else {
                break;
            }
        }
    }
    bool fail(const char* what)
    {
        if (error.empty()) {
            usize line = 1;
            for (usize i = 0; i < pos && i < text.size(); ++i) line += text[i] == '\n' ? 1 : 0;
            error = std::string(what) + " (line " + std::to_string(line) + ")";
        }
        return false;
    }
    bool literal(std::string_view word)
    {
        if (text.substr(pos, word.size()) == word) {
            pos += word.size();
            return true;
        }
        return false;
    }
    bool parseString(std::string& out)
    {
        if (pos >= text.size() || text[pos] != '"') return fail("expected a string");
        ++pos;
        while (pos < text.size() && text[pos] != '"') {
            char c = text[pos++];
            if (c == '\\' && pos < text.size()) {
                const char e = text[pos++];
                switch (e) {
                case 'n': c = '\n'; break;
                case 'r': c = '\r'; break;
                case 't': c = '\t'; break;
                case 'b': c = '\b'; break;
                case 'f': c = '\f'; break;
                case 'u': {
                    if (pos + 4 > text.size()) return fail("bad \\u escape");
                    unsigned code = 0;
                    std::from_chars(text.data() + pos, text.data() + pos + 4, code, 16);
                    pos += 4;
                    // UTF-8 encode (BMP only).
                    if (code < 0x80) {
                        out += static_cast<char>(code);
                    } else if (code < 0x800) {
                        out += static_cast<char>(0xC0 | (code >> 6));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (code >> 12));
                        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    continue;
                }
                default: c = e; break;
                }
            }
            out += c;
        }
        if (pos >= text.size()) return fail("unterminated string");
        ++pos;
        return true;
    }
    bool parseValue(Json& out, int depth)
    {
        if (depth > 200) return fail("nesting too deep");
        skip();
        if (pos >= text.size()) return fail("unexpected end");
        const char c = text[pos];
        if (c == '{') {
            ++pos;
            Json::Object object;
            skip();
            if (pos < text.size() && text[pos] == '}') {
                ++pos;
                out = Json(std::move(object));
                return true;
            }
            while (true) {
                skip();
                std::string key;
                if (!parseString(key)) return false;
                skip();
                if (pos >= text.size() || text[pos] != ':') return fail("expected ':'");
                ++pos;
                Json value;
                if (!parseValue(value, depth + 1)) return false;
                object[key] = std::move(value);
                skip();
                if (pos < text.size() && text[pos] == ',') {
                    ++pos;
                    skip();
                    if (pos < text.size() && text[pos] == '}') { ++pos; break; } // trailing comma
                    continue;
                }
                if (pos < text.size() && text[pos] == '}') { ++pos; break; }
                return fail("expected ',' or '}'");
            }
            out = Json(std::move(object));
            return true;
        }
        if (c == '[') {
            ++pos;
            Json::Array array;
            skip();
            if (pos < text.size() && text[pos] == ']') {
                ++pos;
                out = Json(std::move(array));
                return true;
            }
            while (true) {
                Json value;
                if (!parseValue(value, depth + 1)) return false;
                array.push_back(std::move(value));
                skip();
                if (pos < text.size() && text[pos] == ',') {
                    ++pos;
                    skip();
                    if (pos < text.size() && text[pos] == ']') { ++pos; break; }
                    continue;
                }
                if (pos < text.size() && text[pos] == ']') { ++pos; break; }
                return fail("expected ',' or ']'");
            }
            out = Json(std::move(array));
            return true;
        }
        if (c == '"') {
            std::string s;
            if (!parseString(s)) return false;
            out = Json(std::move(s));
            return true;
        }
        if (literal("true")) { out = Json(true); return true; }
        if (literal("false")) { out = Json(false); return true; }
        if (literal("null")) { out = Json(); return true; }
        // Number.
        const usize start = pos;
        while (pos < text.size() && (std::isdigit(static_cast<unsigned char>(text[pos])) || text[pos] == '-' || text[pos] == '+' ||
                                     text[pos] == '.' || text[pos] == 'e' || text[pos] == 'E')) {
            ++pos;
        }
        if (start == pos) return fail("unexpected character");
        const std::string number(text.substr(start, pos - start));
        char* end = nullptr;
        const f64 v = std::strtod(number.c_str(), &end);
        if (end != number.c_str() + number.size()) return fail("bad number");
        out = Json(v);
        return true;
    }
};

} // namespace

Json::Json(const glm::mat4& m)
    : Json(Array{})
{
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) m_array->push_back(Json(m[c][r]));
    }
}

const std::string& Json::asString() const { return m_type == Type::String ? m_string : kEmptyString; }

glm::vec2 Json::asVec2(const glm::vec2& fallback) const
{
    if (!isArray() || size() < 2) return fallback;
    return {(*this)[0].asFloat(fallback.x), (*this)[1].asFloat(fallback.y)};
}

glm::vec3 Json::asVec3(const glm::vec3& fallback) const
{
    if (isNumber()) return glm::vec3(asFloat()); // a single number = all three
    if (!isArray() || size() < 3) return fallback;
    return {(*this)[0].asFloat(fallback.x), (*this)[1].asFloat(fallback.y), (*this)[2].asFloat(fallback.z)};
}

glm::vec4 Json::asVec4(const glm::vec4& fallback) const
{
    if (!isArray() || size() < 3) return fallback;
    return {(*this)[0].asFloat(fallback.x), (*this)[1].asFloat(fallback.y), (*this)[2].asFloat(fallback.z),
            size() > 3 ? (*this)[3].asFloat(fallback.w) : 1.0f};
}

glm::mat4 Json::asMat4(const glm::mat4& fallback) const
{
    if (!isArray() || size() != 16) return fallback;
    glm::mat4 m(1.0f);
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) m[c][r] = (*this)[static_cast<usize>(c * 4 + r)].asFloat();
    }
    return m;
}

usize Json::size() const
{
    if (m_type == Type::Array) return m_array->size();
    if (m_type == Type::Object) return m_object->size();
    return 0;
}

const Json& Json::operator[](usize i) const { return m_type == Type::Array && i < m_array->size() ? (*m_array)[i] : kNull; }

void Json::push(Json value)
{
    if (m_type != Type::Array) {
        *this = Json(Array{});
    }
    m_array->push_back(std::move(value));
}

const Json::Array& Json::items() const { return m_type == Type::Array ? *m_array : kEmptyArray; }

const Json& Json::operator[](std::string_view key) const
{
    if (m_type != Type::Object) return kNull;
    const auto it = m_object->find(key);
    return it == m_object->end() ? kNull : it->second;
}

bool Json::has(std::string_view key) const { return m_type == Type::Object && m_object->find(key) != m_object->end(); }

void Json::set(std::string key, Json value)
{
    if (m_type != Type::Object) {
        *this = Json(Object{});
    }
    (*m_object)[std::move(key)] = std::move(value);
}

const Json::Object& Json::members() const { return m_type == Type::Object ? *m_object : kEmptyObject; }

void Json::write(std::string& out, int indent, int depth) const
{
    const auto newline = [&](int d) {
        if (indent <= 0) return;
        out += '\n';
        out.append(static_cast<usize>(indent * d), ' ');
    };
    switch (m_type) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += m_bool ? "true" : "false"; break;
    case Type::Number: {
        if (!std::isfinite(m_number)) {
            out += "0";
        } else if (m_number == std::floor(m_number) && std::abs(m_number) < 1e15) {
            out += std::to_string(static_cast<i64>(m_number));
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.7g", m_number);
            out += buf;
        }
        break;
    }
    case Type::String: writeString(out, m_string); break;
    case Type::Array: {
        // Short arrays of numbers (vectors, matrices) on one line.
        bool flat = m_array->size() <= 16;
        for (const Json& v : *m_array) flat = flat && (v.isNumber() || v.isNull() || v.m_type == Type::Bool);
        out += '[';
        for (usize i = 0; i < m_array->size(); ++i) {
            if (i > 0) out += flat ? ", " : ",";
            if (!flat) newline(depth + 1);
            (*m_array)[i].write(out, indent, depth + 1);
        }
        if (!flat && !m_array->empty()) newline(depth);
        out += ']';
        break;
    }
    case Type::Object: {
        out += '{';
        usize i = 0;
        for (const auto& [key, value] : *m_object) {
            if (i++ > 0) out += ',';
            newline(depth + 1);
            writeString(out, key);
            out += indent > 0 ? ": " : ":";
            value.write(out, indent, depth + 1);
        }
        if (!m_object->empty()) newline(depth);
        out += '}';
        break;
    }
    }
}

std::string Json::dump(int indent) const
{
    std::string out;
    write(out, indent, 0);
    if (indent > 0) out += '\n';
    return out;
}

std::optional<Json> Json::parse(std::string_view text, std::string* error)
{
    Parser p{text};
    Json value;
    if (!p.parseValue(value, 0)) {
        if (error) *error = p.error;
        return std::nullopt;
    }
    p.skip();
    if (p.pos != text.size()) {
        p.fail("trailing characters");
        if (error) *error = p.error;
        return std::nullopt;
    }
    return value;
}

std::optional<Json> loadJsonFile(const std::string& path, std::string* error)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        if (error) *error = "cannot open " + path;
        return std::nullopt;
    }
    std::stringstream ss;
    ss << file.rdbuf();
    return Json::parse(ss.str(), error);
}

bool saveJsonFile(const std::string& path, const Json& value, std::string* error)
{
    // Write next to the target, then replace it: a crash mid-write never leaves half a file.
    const std::string temp = path + ".tmp";
    {
        std::ofstream file(temp, std::ios::binary | std::ios::trunc);
        if (!file) {
            if (error) *error = "cannot write " + temp;
            return false;
        }
        file << value.dump(2);
        if (!file) {
            if (error) *error = "write failed: " + temp;
            return false;
        }
    }
    std::remove(path.c_str());
    if (std::rename(temp.c_str(), path.c_str()) != 0) {
        if (error) *error = "cannot replace " + path;
        return false;
    }
    return true;
}

} // namespace kinetic
