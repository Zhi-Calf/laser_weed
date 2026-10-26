#pragma once
#include <filesystem>
#include "rk_nav/core/row_tracking.hpp"
#include "rk_nav/core/status.hpp"

namespace rk_nav {
struct PerceptionOutput;
struct TemporalTrackerConfig {
    bool enabled{false};
    std::uint32_t confirmation_frames{3}, max_missed_frames{3};
    double max_gap_s{0.5}, max_coast_s{0.35};
    double association_gate_m{0.12}, heading_gate_rad{0.08};
    double target_gate_m{0.15}, target_heading_gate_rad{0.08};
    double smoothing_tau_s{0.1};
};
Result<TemporalTrackerConfig> loadTemporalTrackerConfig(const std::filesystem::path& path);

class RowTemporalTracker {
public:
    explicit RowTemporalTracker(TemporalTrackerConfig config = {}) : config_(config) {}
    Status validateConfig() const;
    Status validateInput(std::int64_t timestamp_ns, std::uint64_t sequence) const;
    Status update(PerceptionOutput& output);
    void reset();
    bool enabled() const { return config_.enabled; }
private:
    struct Track {
        std::uint64_t id{0};
        double lateral{0.0}, heading{0.0}, lateral_velocity{0.0}, heading_velocity{0.0};
        std::int64_t last_observed_ns{0};
        std::uint32_t hits{0}, misses{0};
        bool confirmed{false}, observed{false};
    };
    TemporalTrackerConfig config_;
    std::vector<Track> tracks_;
    std::int64_t last_timestamp_ns_{0}, last_accepted_ns_{0};
    std::uint64_t last_sequence_{0}, next_id_{1}, left_id_{0}, right_id_{0};
    std::uint64_t pending_left_id_{0}, pending_right_id_{0};
    std::uint32_t pending_hits_{0};
};
} // namespace rk_nav
