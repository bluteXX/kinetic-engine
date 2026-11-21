#include <kinetic/core/Json.h>

#include <doctest/doctest.h>

#include <filesystem>

using namespace kinetic;

TEST_CASE("Json: parse and dump round-trip")
{
    const char* text = R"({
        // comments are allowed in hand-written files
        "name": "Warrior \"A\"\n",
        "numbers": [1, -2.5, 1e3],
        "flag": true,
        "nothing": null,
        "nested": {"x": [0.25, 0.5, 0.75], "empty": {}, "list": []},
    })";
    std::string error;
    const auto json = Json::parse(text, &error);
    REQUIRE_MESSAGE(json.has_value(), error);
    CHECK((*json)["name"].asString() == "Warrior \"A\"\n");
    CHECK((*json)["numbers"].size() == 3);
    CHECK((*json)["numbers"][1].asFloat() == doctest::Approx(-2.5f));
    CHECK((*json)["numbers"][2].asNumber() == doctest::Approx(1000.0));
    CHECK((*json)["flag"].asBool());
    CHECK((*json)["nothing"].isNull());
    CHECK((*json)["missing"]["deeper"].isNull()); // missing keys read as null
    CHECK((*json)["nested"]["x"].asVec3() == glm::vec3(0.25f, 0.5f, 0.75f));

    const auto again = Json::parse(json->dump());
    REQUIRE(again.has_value());
    CHECK(again->dump() == json->dump());
    CHECK(Json::parse(json->dump(0)).has_value()); // compact form too
}

TEST_CASE("Json: errors and matrices")
{
    std::string error;
    CHECK_FALSE(Json::parse("{\"a\": [1, 2", &error).has_value());
    CHECK_FALSE(error.empty());
    CHECK_FALSE(Json::parse("{} trailing").has_value());

    glm::mat4 m(1.0f);
    m[3] = glm::vec4(0.1f, -0.2f, 0.3f, 1.0f);
    m[0][1] = 0.5f;
    const auto back = Json::parse(Json(m).dump());
    REQUIRE(back.has_value());
    const glm::mat4 r = back->asMat4();
    for (int c = 0; c < 4; ++c) {
        for (int k = 0; k < 4; ++k) CHECK(r[c][k] == doctest::Approx(m[c][k]));
    }
}

TEST_CASE("Json: files are written whole")
{
    const auto path = (std::filesystem::temp_directory_path() / "kinetic_json_test.json").string();
    Json j = Json::object();
    j.set("value", 42);
    REQUIRE(saveJsonFile(path, j));
    const auto loaded = loadJsonFile(path);
    REQUIRE(loaded.has_value());
    CHECK((*loaded)["value"].asInt() == 42);
    std::filesystem::remove(path);
}
