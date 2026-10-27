#include "roar_localization/aruco_map.hpp"

#include <yaml-cpp/yaml.h>
#include <sstream>

namespace roar {

// ─────────────────────────────────────────────────────────────────────────────
// loadFromYaml
//
// Parses a YAML file with this schema:
//
//   markers:
//     - id: 51
//       position: [5.0, 0.0, 0.5]
//     - id: 52
//       position: [10.0, 5.0, 0.5]
//
// Throws std::runtime_error with a descriptive message on any parse error,
// missing field, or duplicate ID.
// ─────────────────────────────────────────────────────────────────────────────
void ArucoMap::loadFromYaml(const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            "ArucoMap: failed to load YAML file '" + yaml_path +
            "': " + e.what());
    }

    if (!root["markers"] || !root["markers"].IsSequence()) {
        throw std::runtime_error(
            "ArucoMap: YAML file '" + yaml_path +
            "' must contain a top-level 'markers' sequence");
    }

    markers_.clear();

    for (const auto& node : root["markers"])
    {
        if (!node["id"] || !node["position"]) {
            throw std::runtime_error(
                "ArucoMap: every marker entry must have 'id' and 'position' fields");
        }

        const int id = node["id"].as<int>();

        if (!node["position"].IsSequence() || node["position"].size() != 3) {
            std::ostringstream oss;
            oss << "ArucoMap: marker id=" << id
                << " must have 'position' as a sequence of exactly 3 floats";
            throw std::runtime_error(oss.str());
        }

        const Vec3 p(
            node["position"][0].as<double>(),
            node["position"][1].as<double>(),
            node["position"][2].as<double>()
        );

        if (markers_.count(id) > 0) {
            std::ostringstream oss;
            oss << "ArucoMap: duplicate marker id=" << id << " in '"
                << yaml_path << "'";
            throw std::runtime_error(oss.str());
        }

        markers_[id] = p;
    }
}

} // namespace roar
