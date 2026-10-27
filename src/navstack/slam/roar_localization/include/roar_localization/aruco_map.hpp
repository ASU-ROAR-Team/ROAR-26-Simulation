#pragma once

// ═════════════════════════════════════════════════════════════════════════════
// aruco_map.hpp
// ─────────────────────────────────────────────────────────────────────────────
// Static lookup table for ArUco marker world positions.
//
// CONCEPT:
//   The ERC competition publishes the world positions of all ArUco markers
//   on the Mars Yard ahead of time (Update Report #3). When perception
//   detects a marker and reports its ID + pose in the camera frame, we use
//   this map to look up where that marker is in world coordinates. Combined
//   with the camera-frame measurement, this gives us an absolute position
//   fix for the rover.
//
// WHY YAML AT STARTUP:
//   - Updated easily without rebuilding (just edit aruco_map.yaml).
//   - Loaded once at node initialisation, then read-only at runtime.
//   - No risk of mid-mission corruption.
//
// LOOKUP:
//   getMarkerPosition(id) returns:
//     - std::optional<Vec3> with world position if marker is in the map
//     - std::nullopt if the marker ID is unknown (e.g., perception
//       hallucinated a non-existent ID — defensive)
// ═════════════════════════════════════════════════════════════════════════════

#include <unordered_map>
#include <optional>
#include <string>
#include <stdexcept>
#include "roar_localization/math_utils.hpp"

namespace roar {

class ArucoMap
{
public:
    ArucoMap() = default;

    // Load the marker map from a YAML file.
    // Throws std::runtime_error if the file is missing or malformed.
    //
    // Expected YAML schema:
    //   markers:
    //     - id: 51
    //       position: [5.0, 0.0, 0.5]
    //     - id: 52
    //       position: [10.0, 5.0, 0.5]
    //     ...
    void loadFromYaml(const std::string& yaml_path);

    // Add or overwrite a marker entry programmatically.
    // Used by tests and the simulation node.
    void addMarker(int id, const Vec3& world_position)
    {
        markers_[id] = world_position;
    }

    // Look up a marker's world position by ID.
    // Returns std::nullopt if the ID is not in the map.
    std::optional<Vec3> getMarkerPosition(int id) const
    {
        auto it = markers_.find(id);
        if (it == markers_.end()) return std::nullopt;
        return it->second;
    }

    // Returns true if the marker ID is registered.
    bool has(int id) const
    {
        return markers_.find(id) != markers_.end();
    }

    // Number of markers loaded.
    size_t size() const { return markers_.size(); }

    // All registered IDs (for logging / debugging).
    std::vector<int> ids() const
    {
        std::vector<int> out;
        out.reserve(markers_.size());
        for (const auto& [id, _] : markers_) out.push_back(id);
        return out;
    }

private:
    std::unordered_map<int, Vec3> markers_;
};

} // namespace roar
