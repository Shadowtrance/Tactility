#include "doctest.h"
#include <Tactility/file/File.h>

#include <cstdio>

using namespace tt;

TEST_CASE("findOrCreateDirectory can create a directory tree without prefix") {
    CHECK_EQ(file::findOrCreateDirectory("test1/test1", 0777), true);
    // TODO: delete dirs
}

TEST_CASE("findOrCreateDirectory can create a directory tree with prefix") {
    CHECK_EQ(file::findOrCreateDirectory("/tmp/test2", 0777), true);
    // TODO: delete dirs
}

// deleteRecursively ---------------------------------------------------------------------------

TEST_CASE("deleteRecursively removes a directory tree") {
    REQUIRE(file::findOrCreateDirectory("/tmp/tt_del/nested/deeper", 0777));

    for (const auto* path : { "/tmp/tt_del/a.txt", "/tmp/tt_del/nested/b.txt",
                              "/tmp/tt_del/nested/deeper/c.txt" }) {
        auto* handle = std::fopen(path, "w");
        REQUIRE(handle != nullptr);
        std::fputs("x", handle);
        std::fclose(handle);
    }

    CHECK(file::deleteRecursively("/tmp/tt_del"));
    CHECK_FALSE(file::isDirectory("/tmp/tt_del"));
}

/**
 * Guards a real infinite recursion.
 *
 * scandir() reports "." and "..", and "<path>/." is itself a directory - so without an
 * explicit skip, deleteRecursively() descended into it forever, building
 * "dir/./././././..." until the path blew past the filesystem limit and the delete failed.
 * Seen in the wild when reinstalling an app over an existing installation.
 */
TEST_CASE("deleteRecursively does not recurse into . or ..") {
    REQUIRE(file::findOrCreateDirectory("/tmp/tt_dots/child", 0777));

    auto* handle = std::fopen("/tmp/tt_dots/child/file.txt", "w");
    REQUIRE(handle != nullptr);
    std::fclose(handle);

    // Would previously fail with "unknown type" on an absurdly long path
    CHECK(file::deleteRecursively("/tmp/tt_dots"));
    CHECK_FALSE(file::isDirectory("/tmp/tt_dots"));
}

TEST_CASE("deleteRecursively on a missing path does not report success") {
    CHECK_FALSE(file::deleteRecursively("/tmp/tt_does_not_exist_12345"));
}

TEST_CASE("deleteRecursively on an empty path is a no-op") {
    CHECK(file::deleteRecursively(""));
}
