#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace gemini336_orbslam3
{
struct MapContext
{
    uint64_t map_id = 0;
    // Session-local identity of successive sampled active-map references;
    // independent of ordinary BA revisions, never a persistent global frame.
    uint64_t generation = 0;
};

struct KeyFrameData
{
    uint64_t id = 0;
    double timestamp = 0.0; // Original Core sensor seconds, not ROS collection time.
    std::array<double, 3> translation_wc{};
    std::array<double, 4> quaternion_wc_xyzw{};
};

struct MapOutputData
{
    MapContext context;
    std::vector<std::array<float, 3>> map_points;
    std::vector<KeyFrameData> keyframes;
};

enum class MapOutputStatus { Ready, NoActiveMap, Unavailable };

struct MapOutputResult
{
    MapOutputStatus status = MapOutputStatus::Unavailable;
    // Engaged exactly for Ready, including a valid map with empty collections.
    std::optional<MapOutputData> data;
};
}
