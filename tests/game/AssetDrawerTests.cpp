// File operations of the editor's Assets drawer (content browser) on a temporary folder. Only
// folders and non-asset files are used, so no renderer work happens.

#include "AssetDrawer.h"

#include <doctest/doctest.h>

#include <fstream>

using namespace kinetic;
namespace fs = std::filesystem;

namespace {

struct TempRoot {
    fs::path path;
    TempRoot()
    {
        path = fs::temp_directory_path() / "kinetic_drawer_test";
        fs::remove_all(path);
        fs::create_directories(path / "models" / "Hero" / "textures");
        std::ofstream(path / "models" / "Hero" / "textures" / "hero_basecolor.png") << "x";
    }
    ~TempRoot()
    {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

} // namespace

TEST_CASE("Asset drawer: create, move and rename folders and files")
{
    TempRoot temp;
    renderer::Renderer renderer; // not initialised: no asset files are loaded in this test
    editor::AssetDrawer drawer;
    drawer.init(renderer, temp.path);
    const fs::path root = drawer.root();

    CHECK(drawer.createFolder(root, "Characters"));
    CHECK(fs::is_directory(root / "Characters"));
    CHECK_FALSE(drawer.createFolder(root, "Characters")); // exists
    CHECK_FALSE(drawer.createFolder(root, "bad/name"));

    // Move a whole folder (with its sub-folders and files) into another.
    CHECK(drawer.movePath(root / "models" / "Hero", root / "Characters"));
    CHECK(fs::exists(root / "Characters" / "Hero" / "textures" / "hero_basecolor.png"));
    CHECK_FALSE(fs::exists(root / "models" / "Hero"));

    // A folder cannot go into itself or its own sub-folder.
    CHECK_FALSE(drawer.movePath(root / "Characters", root / "Characters" / "Hero"));

    // Move a single file, then rename it (the extension is kept).
    CHECK(drawer.movePath(root / "Characters" / "Hero" / "textures" / "hero_basecolor.png", root / "Characters" / "Hero"));
    CHECK(fs::exists(root / "Characters" / "Hero" / "hero_basecolor.png"));
    CHECK(drawer.renamePath(root / "Characters" / "Hero" / "hero_basecolor.png", "warrior_basecolor"));
    CHECK(fs::exists(root / "Characters" / "Hero" / "warrior_basecolor.png"));

    // Renaming the folder that is open keeps it open under its new name.
    drawer.openFolder(root / "Characters" / "Hero");
    CHECK(drawer.renamePath(root / "Characters" / "Hero", "Warrior"));
    CHECK(fs::is_directory(root / "Characters" / "Warrior"));
    CHECK(fs::weakly_canonical(drawer.currentFolder()) == fs::weakly_canonical(root / "Characters" / "Warrior"));

    // Nothing outside the root is touched.
    CHECK_FALSE(drawer.movePath(root / "Characters", root.parent_path()));
}
