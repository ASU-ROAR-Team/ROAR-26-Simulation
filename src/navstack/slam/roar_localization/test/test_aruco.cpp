//
// test_aruco.cpp
//
// Tests for the ArucoMap class:
//   - YAML loading
//   - Marker lookup (existing and missing IDs)
//   - Programmatic addition
//   - Schema validation (malformed YAML, duplicates, missing fields)
//   - ids() accessor
//

#include <iostream>
#include <fstream>
#include <cstdio>
#include <stdexcept>
#include "roar_localization/aruco_map.hpp"

using namespace roar;

static int passed = 0;
static int failed = 0;

#define CHECK(name, cond) \
    do { \
        if (cond) { std::cout << "  PASS  " << (name) << "\n"; ++passed; } \
        else       { std::cout << "  FAIL  " << (name) << "\n"; ++failed; } \
    } while(0)

// ─────────────────────────────────────────────────────────────────────────────
// Helper: write a temp YAML file with the given content, return its path
// ─────────────────────────────────────────────────────────────────────────────
static std::string writeTempYaml(const std::string& content,
                                 const std::string& name)
{
    const std::string path = "/tmp/" + name;
    std::ofstream f(path);
    f << content;
    f.close();
    return path;
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Load a valid YAML file with multiple markers
// ─────────────────────────────────────────────────────────────────────────────
void test_load_valid_yaml()
{
    std::cout << "\n[1. Load valid YAML]\n";

    const std::string yaml = R"(markers:
  - id: 51
    position: [5.0, 0.0, 0.5]
  - id: 52
    position: [10.0, 5.0, 0.5]
  - id: 53
    position: [-3.0, -2.0, 0.5]
)";
    const std::string path = writeTempYaml(yaml, "aruco_valid.yaml");

    ArucoMap m;
    bool loaded = false;
    try {
        m.loadFromYaml(path);
        loaded = true;
    } catch (const std::exception& e) {
        std::cout << "  exception: " << e.what() << "\n";
    }

    CHECK("loads without exception", loaded);
    CHECK("size == 3",               m.size() == 3);
    CHECK("has id 51",               m.has(51));
    CHECK("has id 52",               m.has(52));
    CHECK("has id 53",               m.has(53));
    CHECK("does NOT have id 99",     !m.has(99));

    auto p51 = m.getMarkerPosition(51);
    CHECK("id 51 lookup succeeds", p51.has_value());
    if (p51) {
        CHECK("id 51 x correct", std::abs(p51->x() -  5.0) < 1e-9);
        CHECK("id 51 y correct", std::abs(p51->y() -  0.0) < 1e-9);
        CHECK("id 51 z correct", std::abs(p51->z() -  0.5) < 1e-9);
    }

    auto p53 = m.getMarkerPosition(53);
    CHECK("id 53 lookup succeeds", p53.has_value());
    if (p53) {
        CHECK("id 53 x is -3.0", std::abs(p53->x() - (-3.0)) < 1e-9);
        CHECK("id 53 y is -2.0", std::abs(p53->y() - (-2.0)) < 1e-9);
    }

    auto p99 = m.getMarkerPosition(99);
    CHECK("missing id returns nullopt", !p99.has_value());

    std::remove(path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. ids() returns all registered IDs
// ─────────────────────────────────────────────────────────────────────────────
void test_ids_listing()
{
    std::cout << "\n[2. ids() lists all registered IDs]\n";
    ArucoMap m;
    m.addMarker(10, Vec3(0.0, 0.0, 0.0));
    m.addMarker(20, Vec3(1.0, 0.0, 0.0));
    m.addMarker(30, Vec3(2.0, 0.0, 0.0));

    auto ids = m.ids();
    CHECK("size == 3", ids.size() == 3);

    // Order is not guaranteed (unordered_map), so just check set membership
    bool has10 = false, has20 = false, has30 = false;
    for (int id : ids) {
        if (id == 10) has10 = true;
        if (id == 20) has20 = true;
        if (id == 30) has30 = true;
    }
    CHECK("contains 10", has10);
    CHECK("contains 20", has20);
    CHECK("contains 30", has30);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. addMarker overwrites an existing entry
// ─────────────────────────────────────────────────────────────────────────────
void test_add_overwrite()
{
    std::cout << "\n[3. addMarker overwrites existing entries]\n";
    ArucoMap m;
    m.addMarker(51, Vec3(1.0, 2.0, 3.0));

    auto p_first = m.getMarkerPosition(51);
    CHECK("first add stored", p_first && std::abs(p_first->x() - 1.0) < 1e-9);

    // Overwrite
    m.addMarker(51, Vec3(99.0, 99.0, 99.0));
    auto p_overwritten = m.getMarkerPosition(51);
    CHECK("overwrite x updated",
          p_overwritten && std::abs(p_overwritten->x() - 99.0) < 1e-9);
    CHECK("size still 1", m.size() == 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Malformed YAML — missing top-level 'markers' key
// ─────────────────────────────────────────────────────────────────────────────
void test_missing_markers_key()
{
    std::cout << "\n[4. Missing 'markers' key throws]\n";
    const std::string yaml = R"(foo: bar
some_other_key: 42
)";
    const std::string path = writeTempYaml(yaml, "aruco_no_markers.yaml");

    ArucoMap m;
    bool threw = false;
    try {
        m.loadFromYaml(path);
    } catch (const std::runtime_error&) {
        threw = true;
    }

    CHECK("throws on missing 'markers'", threw);
    std::remove(path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Malformed YAML — marker without 'id' or 'position'
// ─────────────────────────────────────────────────────────────────────────────
void test_missing_fields()
{
    std::cout << "\n[5. Marker entry missing fields throws]\n";

    // Missing 'position'
    {
        const std::string yaml = R"(markers:
  - id: 51
)";
        const std::string path = writeTempYaml(yaml, "aruco_no_pos.yaml");
        ArucoMap m;
        bool threw = false;
        try { m.loadFromYaml(path); }
        catch (const std::runtime_error&) { threw = true; }
        CHECK("throws when 'position' missing", threw);
        std::remove(path.c_str());
    }

    // Missing 'id'
    {
        const std::string yaml = R"(markers:
  - position: [1.0, 2.0, 3.0]
)";
        const std::string path = writeTempYaml(yaml, "aruco_no_id.yaml");
        ArucoMap m;
        bool threw = false;
        try { m.loadFromYaml(path); }
        catch (const std::runtime_error&) { threw = true; }
        CHECK("throws when 'id' missing", threw);
        std::remove(path.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Malformed YAML — wrong number of position elements
// ─────────────────────────────────────────────────────────────────────────────
void test_wrong_position_size()
{
    std::cout << "\n[6. Wrong number of position elements throws]\n";
    const std::string yaml = R"(markers:
  - id: 51
    position: [1.0, 2.0]
)";
    const std::string path = writeTempYaml(yaml, "aruco_bad_pos.yaml");

    ArucoMap m;
    bool threw = false;
    try { m.loadFromYaml(path); }
    catch (const std::runtime_error&) { threw = true; }

    CHECK("throws on 2-element position", threw);
    std::remove(path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. Malformed YAML — duplicate marker IDs
// ─────────────────────────────────────────────────────────────────────────────
void test_duplicate_ids()
{
    std::cout << "\n[7. Duplicate marker IDs throws]\n";
    const std::string yaml = R"(markers:
  - id: 51
    position: [1.0, 0.0, 0.0]
  - id: 51
    position: [2.0, 0.0, 0.0]
)";
    const std::string path = writeTempYaml(yaml, "aruco_dup.yaml");

    ArucoMap m;
    bool threw = false;
    try { m.loadFromYaml(path); }
    catch (const std::runtime_error&) { threw = true; }

    CHECK("throws on duplicate id", threw);
    std::remove(path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Nonexistent file throws
// ─────────────────────────────────────────────────────────────────────────────
void test_missing_file()
{
    std::cout << "\n[8. Nonexistent file throws]\n";

    ArucoMap m;
    bool threw = false;
    try {
        m.loadFromYaml("/tmp/this_file_should_not_exist_12345.yaml");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK("throws on missing file", threw);
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. Empty markers list is valid (loads but with zero entries)
// ─────────────────────────────────────────────────────────────────────────────
void test_empty_markers()
{
    std::cout << "\n[9. Empty markers list is valid]\n";
    const std::string yaml = R"(markers: []
)";
    const std::string path = writeTempYaml(yaml, "aruco_empty.yaml");

    ArucoMap m;
    bool loaded = false;
    try {
        m.loadFromYaml(path);
        loaded = true;
    } catch (const std::exception&) {}

    CHECK("loads empty list", loaded);
    CHECK("size == 0", m.size() == 0);
    CHECK("ids() is empty", m.ids().empty());
    std::remove(path.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Reloading replaces previous contents
// ─────────────────────────────────────────────────────────────────────────────
void test_reload_replaces()
{
    std::cout << "\n[10. loadFromYaml replaces previous contents]\n";

    const std::string yaml1 = R"(markers:
  - id: 1
    position: [0.0, 0.0, 0.0]
  - id: 2
    position: [1.0, 1.0, 1.0]
)";
    const std::string yaml2 = R"(markers:
  - id: 99
    position: [9.0, 9.0, 9.0]
)";
    const std::string p1 = writeTempYaml(yaml1, "aruco_first.yaml");
    const std::string p2 = writeTempYaml(yaml2, "aruco_second.yaml");

    ArucoMap m;
    m.loadFromYaml(p1);
    CHECK("first load has 2 markers", m.size() == 2);

    m.loadFromYaml(p2);
    CHECK("after reload size == 1", m.size() == 1);
    CHECK("after reload old IDs gone", !m.has(1) && !m.has(2));
    CHECK("after reload new ID present", m.has(99));

    std::remove(p1.c_str());
    std::remove(p2.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    std::cout << "=== ArucoMap unit tests ===\n";

    test_load_valid_yaml();
    test_ids_listing();
    test_add_overwrite();
    test_missing_markers_key();
    test_missing_fields();
    test_wrong_position_size();
    test_duplicate_ids();
    test_missing_file();
    test_empty_markers();
    test_reload_replaces();

    std::cout << "\n=== Results: "
              << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}
