#include "rk_nav/perception/row_temporal_tracker.hpp"
#include "rk_nav/pipeline/perception_pipeline.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>
#include <stdexcept>

namespace rk_nav {
namespace {
void invalidate(NavigationObservation& nav, TrackingStatus status) {
    nav.status=status; nav.confidence=0;
    nav.lateral_error_m=nav.heading_error_rad=std::numeric_limits<double>::quiet_NaN();
}
}
Status RowTemporalTracker::validateConfig() const {
    const auto& c=config_;
    if (c.confirmation_frames<1 || c.confirmation_frames>100 || c.max_missed_frames>100 ||
        !std::isfinite(c.max_gap_s) || c.max_gap_s<=0 ||
        !std::isfinite(c.max_coast_s) || c.max_coast_s<=0 || c.max_coast_s>c.max_gap_s ||
        !std::isfinite(c.association_gate_m) || c.association_gate_m<=0 ||
        !std::isfinite(c.heading_gate_rad) || c.heading_gate_rad<=0 ||
        !std::isfinite(c.target_gate_m) || c.target_gate_m<=0 ||
        !std::isfinite(c.target_heading_gate_rad) || c.target_heading_gate_rad<=0 ||
        !std::isfinite(c.smoothing_tau_s) || c.smoothing_tau_s<=0)
        return Status::invalidArgument("invalid temporal tracker configuration");
    return Status::ok();
}
Status RowTemporalTracker::validateInput(std::int64_t timestamp_ns,std::uint64_t sequence) const {
    const auto valid=validateConfig(); if (!valid.isOk()) return valid;
    if (!config_.enabled) return Status::ok();
    if (timestamp_ns<=0 || (last_timestamp_ns_ &&
        (timestamp_ns<=last_timestamp_ns_ || sequence<=last_sequence_)))
        return Status::invalidArgument("tracking requires positive increasing capture timestamps and frame IDs");
    return Status::ok();
}
void RowTemporalTracker::reset() {
    tracks_.clear(); left_id_=right_id_=0; last_timestamp_ns_=last_accepted_ns_=0; last_sequence_=0;
    pending_left_id_=pending_right_id_=0; pending_hits_=0;
    // Never recycle identities during the lifetime of this tracker.
}

Status RowTemporalTracker::update(PerceptionOutput& output) {
    auto& nav=output.navigation;
    const auto valid=validateInput(nav.capture_time_ns,nav.sequence);
    if (!valid.isOk()) return valid;
    if (!config_.enabled) return Status::ok();
    if (nav.status==TrackingStatus::kValid &&
        (!std::isfinite(nav.lateral_error_m) || !std::isfinite(nav.heading_error_rad)))
        return Status::invalidArgument("non-finite tracking observation");
    RowTrackingDiagnostics diag;
    diag.raw_valid=nav.status==TrackingStatus::kValid;
    if (diag.raw_valid) { diag.raw_lateral_m=nav.lateral_error_m; diag.raw_heading_rad=nav.heading_error_rad; }
    const auto now=nav.capture_time_ns;
    double dt=last_timestamp_ns_ ? double(now-last_timestamp_ns_)/1e9 : 0.0;
    if (dt>config_.max_gap_s) { reset(); diag.reset_after_gap=true; dt=0; }
    last_timestamp_ns_=now; last_sequence_=nav.sequence;
    for (auto& t : tracks_) {
        t.lateral+=t.lateral_velocity*dt; t.heading+=t.heading_velocity*dt;
        t.observed=false; ++t.misses;
    }
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track& t) {
        return t.misses>config_.max_missed_frames+1 || double(now-t.last_observed_ns)/1e9>config_.max_coast_s;
    }),tracks_.end());
    const auto find = [&](std::uint64_t id)->Track* {
        const auto it=std::find_if(tracks_.begin(),tracks_.end(),[&](const Track& t) { return t.id==id; });
        return it==tracks_.end()?nullptr:&*it;
    };
    if (left_id_ && (!find(left_id_) || !find(right_id_) || double(now-last_accepted_ns_)/1e9>config_.max_coast_s))
        left_id_=right_id_=0;
    if (left_id_) {
        const auto* l=find(left_id_); const auto* r=find(right_id_);
        diag.prediction_available=true;
        diag.predicted_lateral_m=.5*(l->lateral+r->lateral);
        diag.predicted_heading_rad=std::atan(.5*(std::tan(l->heading)+std::tan(r->heading)));
    }

    auto* rows=output.rows ? &*output.rows : nullptr;
    const bool geometry_valid=diag.raw_valid && rows && rows->navigation_pair_valid &&
        rows->method==RowExtractionMethod::kAssociatedRows && rows->selected_left>=0 && rows->selected_right>=0 &&
        static_cast<std::size_t>(rows->selected_left)<rows->fitted_rows.size() &&
        static_cast<std::size_t>(rows->selected_right)<rows->fitted_rows.size();
    const std::size_t count=geometry_valid ? rows->fitted_rows.size() : 0;
    std::vector<int> assigned(count,-1);
    std::vector<std::tuple<double,std::size_t,std::size_t>> edges;
    const auto measurement = [&](std::size_t i) {
        double slope=rows->fitted_rows[i].slope, intercept=rows->fitted_rows[i].intercept_m;
        if (static_cast<int>(i)==rows->selected_left) { slope=rows->joint_fit.left_slope; intercept=rows->joint_fit.left_intercept_m; }
        if (static_cast<int>(i)==rows->selected_right) { slope=rows->joint_fit.right_slope; intercept=rows->joint_fit.right_intercept_m; }
        return cv::Vec2d(intercept,std::atan(slope));
    };
    for (std::size_t r=0;r<count;++r) {
        const auto m=measurement(r);
        for (std::size_t t=0;t<tracks_.size();++t) {
            const double dy=std::abs(m[0]-tracks_[t].lateral), da=std::abs(m[1]-tracks_[t].heading);
            if (dy<=config_.association_gate_m && da<=config_.heading_gate_rad)
                edges.emplace_back(dy/config_.association_gate_m+da/config_.heading_gate_rad,r,t);
        }
    }
    std::sort(edges.begin(),edges.end());
    std::vector<bool> used(tracks_.size(),false);
    for (const auto& edge : edges) {
        const auto r=std::get<1>(edge), t=std::get<2>(edge);
        if (assigned[r]>=0 || used[t]) continue;
        assigned[r]=static_cast<int>(t); used[t]=true;
    }
    bool rejected=false;
    if (geometry_valid && left_id_) {
        const int l=assigned[static_cast<std::size_t>(rows->selected_left)];
        const int r=assigned[static_cast<std::size_t>(rows->selected_right)];
        rejected=l<0 || r<0 || tracks_[static_cast<std::size_t>(l)].id!=left_id_ || tracks_[static_cast<std::size_t>(r)].id!=right_id_ ||
            std::abs(nav.lateral_error_m-diag.predicted_lateral_m)>config_.target_gate_m ||
            std::abs(nav.heading_error_rad-diag.predicted_heading_rad)>config_.target_heading_gate_rad;
    }
    if (!rejected) for (std::size_t r=0;r<count;++r) {
        const auto m=measurement(r);
        if (assigned[r]<0) {
            Track track; track.id=next_id_++; track.lateral=m[0]; track.heading=m[1];
            track.last_observed_ns=now; track.hits=1; track.confirmed=config_.confirmation_frames==1; track.observed=true;
            assigned[r]=static_cast<int>(tracks_.size()); tracks_.push_back(track);
        } else {
            auto& t=tracks_[static_cast<std::size_t>(assigned[r])];
            if (t.misses>1) t.hits=0;
            const double alpha=dt>0 ? 1.0-std::exp(-dt/config_.smoothing_tau_s) : 1.0;
            const double error_y=m[0]-t.lateral, error_a=m[1]-t.heading;
            t.lateral+=alpha*error_y; t.heading+=alpha*error_a;
            if (dt>0) {
                const double beta=.25*alpha*alpha;
                t.lateral_velocity=std::clamp(t.lateral_velocity+beta*error_y/dt,-.5,.5);
                t.heading_velocity=std::clamp(t.heading_velocity+beta*error_a/dt,-.5,.5);
            }
            t.misses=0; t.last_observed_ns=now; t.observed=true; ++t.hits;
            t.confirmed=t.confirmed || t.hits>=config_.confirmation_frames;
        }
        rows->fitted_rows[r].track_id=tracks_[static_cast<std::size_t>(assigned[r])].id;
    }
    for (auto& t : tracks_) if (!t.observed) { t.hits=0; t.lateral_velocity*=.5; t.heading_velocity*=.5; }
    bool accepted=false;
    if (geometry_valid && !rejected) {
        const auto& l=tracks_[static_cast<std::size_t>(assigned[static_cast<std::size_t>(rows->selected_left)])];
        const auto& r=tracks_[static_cast<std::size_t>(assigned[static_cast<std::size_t>(rows->selected_right)])];
        if (!left_id_) {
            if (pending_left_id_==l.id && pending_right_id_==r.id) ++pending_hits_;
            else { pending_left_id_=l.id; pending_right_id_=r.id; pending_hits_=1; }
        }
        if (l.confirmed && r.confirmed && (left_id_ || pending_hits_>=config_.confirmation_frames)) {
            left_id_=l.id; right_id_=r.id; last_accepted_ns_=now; accepted=true;
            pending_left_id_=pending_right_id_=0; pending_hits_=0;
            nav.lateral_error_m=.5*(l.lateral+r.lateral);
            nav.heading_error_rad=std::atan(.5*(std::tan(l.heading)+std::tan(r.heading)));
        }
    }
    if (!geometry_valid || rejected) { pending_left_id_=pending_right_id_=0; pending_hits_=0; }
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track& t) {
        return !t.observed && t.misses>config_.max_missed_frames;
    }),tracks_.end());
    if (left_id_ && (!find(left_id_) || !find(right_id_))) left_id_=right_id_=0;
    if (!left_id_) diag.prediction_available=false;
    // Predictions are diagnostic only. Only fresh, confirmed measured pairs can be VALID.
    diag.state=accepted ? RowTrackingState::kTracking : rejected ? RowTrackingState::kRejected :
        left_id_ ? RowTrackingState::kCoasting : geometry_valid ? RowTrackingState::kAcquiring : RowTrackingState::kLost;
    if (!accepted) invalidate(nav,geometry_valid ? TrackingStatus::kLowConfidence :
        nav.status==TrackingStatus::kLowConfidence ? TrackingStatus::kLowConfidence : TrackingStatus::kNoRow);
    diag.left_id=left_id_; diag.right_id=right_id_;
    for (const auto& t : tracks_) diag.tracks.push_back({t.id,t.lateral,t.heading,t.misses,t.confirmed,t.observed});
    nav.tracking=std::move(diag);
    return Status::ok();
}

Result<TemporalTrackerConfig> loadTemporalTrackerConfig(const std::filesystem::path& path) {
    try {
        cv::FileStorage fs(path.string(),cv::FileStorage::READ);
        if (!fs.isOpened()) return Status::notFound("cannot open tracker configuration");
        TemporalTrackerConfig c;
        const auto n=fs["row_tracker"];
        if (!n.empty()) {
            if (!n.isMap()) return Status::invalidArgument("row_tracker must be a mapping");
            if (!n["enabled"].empty()) {
                if (!n["enabled"].isInt()) return Status::invalidArgument("tracker enabled must be 0 or 1");
                const int enabled=static_cast<int>(n["enabled"]);
                if (enabled!=0 && enabled!=1) return Status::invalidArgument("tracker enabled must be 0 or 1");
                c.enabled=enabled==1;
            }
            const auto read = [&](const char* key,double& value) {
                if (!n[key].empty()) {
                    if (!n[key].isReal() && !n[key].isInt()) throw std::invalid_argument("tracker parameter must be numeric");
                    n[key]>>value;
                }
            };
            for (const auto& key : {"confirmation_frames","max_missed_frames"}) if (!n[key].empty()) {
                if (!n[key].isInt()) return Status::invalidArgument("tracker frame budget must be integer");
                const int value=static_cast<int>(n[key]);
                if (value<0 || value>100) return Status::invalidArgument("tracker frame budget out of range");
                if (std::string(key)=="confirmation_frames") c.confirmation_frames=static_cast<std::uint32_t>(value);
                else c.max_missed_frames=static_cast<std::uint32_t>(value);
            }
            read("max_gap_s",c.max_gap_s); read("max_coast_s",c.max_coast_s);
            read("association_gate_m",c.association_gate_m); read("heading_gate_rad",c.heading_gate_rad);
            read("target_gate_m",c.target_gate_m); read("target_heading_gate_rad",c.target_heading_gate_rad);
            read("smoothing_tau_s",c.smoothing_tau_s);
        }
        const auto valid=RowTemporalTracker(c).validateConfig(); if (!valid.isOk()) return valid;
        return c;
    } catch (const std::exception& e) { return Status::invalidArgument(e.what()); }
}
} // namespace rk_nav
