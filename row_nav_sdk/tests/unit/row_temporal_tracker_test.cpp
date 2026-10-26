#include "test_framework.hpp"
#include "rk_nav/pipeline/perception_pipeline.hpp"
#include "rk_nav/io/navigation_output.hpp"
#include "support/row_sequences.hpp"
#include <chrono>
#include <fstream>

namespace {
rk_nav::TemporalTrackerConfig enabledConfig() { rk_nav::TemporalTrackerConfig c; c.enabled=true; return c; }
auto trackedPipeline() { return rk_nav::TrackedPerceptionPipeline(rk_nav::test::sequencePipeline(),enabledConfig()); }
}

TEST_CASE("temporal tracker confirms measured rows and keeps persistent identities") {
    auto pipeline=trackedPipeline();
    std::uint64_t left=0,right=0;
    for (std::uint64_t i=0;i<20;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("drift",i).segmentation);
        REQUIRE(out.isOk());
        const auto& nav=out.value().navigation;
        if (i<2) {
            REQUIRE(nav.tracking.state==rk_nav::RowTrackingState::kAcquiring);
            REQUIRE(nav.status==rk_nav::TrackingStatus::kLowConfidence);
            REQUIRE(std::isnan(nav.lateral_error_m));
        } else {
            REQUIRE(nav.status==rk_nav::TrackingStatus::kValid);
            if (i==2) { left=nav.tracking.left_id; right=nav.tracking.right_id; }
            REQUIRE(left!=0 && right!=0 && left!=right);
            REQUIRE(nav.tracking.left_id==left && nav.tracking.right_id==right);
            REQUIRE(std::abs(nav.lateral_error_m-(.04+.002*double(i)))<.01);
            REQUIRE(nav.tracking.tracks.size()==4);
        }
    }
}

TEST_CASE("temporal tracker coasts diagnostically without turning predictions into observations") {
    auto pipeline=trackedPipeline();
    std::uint64_t id=0;
    for (std::uint64_t i=0;i<30;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("short_occlusion",i).segmentation);
        REQUIRE(out.isOk());
        const auto& nav=out.value().navigation;
        if (i==24) id=nav.tracking.left_id;
        if (i==25 || i==26) {
            REQUIRE(nav.status==rk_nav::TrackingStatus::kNoRow);
            REQUIRE(nav.tracking.state==rk_nav::RowTrackingState::kCoasting);
            REQUIRE(nav.tracking.prediction_available && !nav.tracking.raw_valid);
            REQUIRE(std::isnan(nav.lateral_error_m) && nav.confidence==0);
            for (const auto& track : nav.tracking.tracks) REQUIRE(!track.observed);
        }
        if (i==27) REQUIRE(nav.status==rk_nav::TrackingStatus::kValid && nav.tracking.left_id==id);
    }
}

TEST_CASE("temporal tracker rejects an impulse without poisoning the next observation") {
    auto pipeline=trackedPipeline();
    std::uint64_t id=0;
    for (std::uint64_t i=0;i<28;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("impulse",i).segmentation);
        REQUIRE(out.isOk());
        const auto& nav=out.value().navigation;
        if (i==24) id=nav.tracking.left_id;
        if (i==25) {
            REQUIRE(nav.tracking.raw_valid);
            REQUIRE(nav.tracking.state==rk_nav::RowTrackingState::kRejected);
            REQUIRE(nav.status!=rk_nav::TrackingStatus::kValid && std::isnan(nav.lateral_error_m));
        }
        if (i==26) {
            REQUIRE(nav.status==rk_nav::TrackingStatus::kValid && nav.tracking.left_id==id);
            REQUIRE(std::abs(nav.lateral_error_m-.04)<.01);
        }
    }
}

TEST_CASE("temporal tracker expires stale identities and reacquires with confirmation") {
    auto pipeline=trackedPipeline();
    std::uint64_t old_id=0;
    for (std::uint64_t i=0;i<34;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("long_occlusion",i).segmentation);
        REQUIRE(out.isOk());
        const auto& nav=out.value().navigation;
        if (i==19) old_id=nav.tracking.left_id;
        if (i==24) {
            REQUIRE(nav.tracking.state==rk_nav::RowTrackingState::kLost);
            REQUIRE(!nav.tracking.prediction_available && nav.tracking.tracks.empty());
        }
        if (i==29 || i==30) REQUIRE(nav.tracking.state==rk_nav::RowTrackingState::kAcquiring);
        if (i==31) REQUIRE(nav.status==rk_nav::TrackingStatus::kValid && nav.tracking.left_id!=old_id);
    }
}

TEST_CASE("temporal tracker rejects duplicates and out of order timestamps transactionally") {
    auto pipeline=trackedPipeline();
    for (std::uint64_t i=0;i<4;++i) REQUIRE(pipeline.process(rk_nav::test::sequenceFrame("drift",i).segmentation).isOk());
    auto frame=rk_nav::test::sequenceFrame("drift",4).segmentation;
    frame.capture_time_ns=1200000000;
    REQUIRE(!pipeline.process(frame).isOk());
    frame=rk_nav::test::sequenceFrame("drift",3).segmentation;
    REQUIRE(!pipeline.process(frame).isOk());
    frame=rk_nav::test::sequenceFrame("drift",4).segmentation;
    const auto result=pipeline.process(frame);
    REQUIRE(result.isOk() && result.value().navigation.status==rk_nav::TrackingStatus::kValid);
    frame=rk_nav::test::sequenceFrame("drift",5).segmentation; frame.capture_time_ns=0;
    REQUIRE(!pipeline.process(frame).isOk());
}

TEST_CASE("expired processing does not advance temporal state and time gaps reset it") {
    auto pipeline=trackedPipeline();
    const auto frame=rk_nav::test::sequenceFrame("drift",0).segmentation;
    auto expired=pipeline.process(frame,0.0,1);
    REQUIRE(expired.isOk() && expired.value().navigation.status==rk_nav::TrackingStatus::kStaleFrame);
    REQUIRE(pipeline.process(frame).isOk()); // Same timestamp/ID still accepted after expired work.
    REQUIRE(pipeline.process(rk_nav::test::sequenceFrame("drift",1).segmentation).isOk());
    auto gap=rk_nav::test::sequenceFrame("drift",10).segmentation;
    const auto out=pipeline.process(gap);
    REQUIRE(out.isOk() && out.value().navigation.tracking.reset_after_gap);
    REQUIRE(out.value().navigation.tracking.state==rk_nav::RowTrackingState::kAcquiring);
}

TEST_CASE("temporal smoothing reduces jitter and keeps step response bounded") {
    auto pipeline=trackedPipeline();
    double raw_error=0,filtered_error=0;
    for (std::uint64_t i=0;i<60;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("jitter",i).segmentation);
        REQUIRE(out.isOk());
        if (i<10) continue;
        const auto& nav=out.value().navigation;
        REQUIRE(nav.status==rk_nav::TrackingStatus::kValid);
        raw_error+=std::pow(nav.tracking.raw_lateral_m-.04,2);
        filtered_error+=std::pow(nav.lateral_error_m-.04,2);
    }
    REQUIRE(filtered_error<raw_error*.5);
    auto step_pipeline=trackedPipeline();
    for (std::uint64_t i=0;i<30;++i) {
        const auto out=step_pipeline.process(rk_nav::test::sequenceFrame("step",i).segmentation);
        REQUIRE(out.isOk());
        if (i>=23) REQUIRE(std::abs(out.value().navigation.lateral_error_m-.08)<.005);
    }
}

TEST_CASE("tracking JSON encodes unavailable numbers as null and reports measured state") {
    auto pipeline=trackedPipeline();
    const auto out=pipeline.process(rk_nav::test::sequenceFrame("jitter",0).segmentation);
    REQUIRE(out.isOk());
    const auto json=rk_nav::serializeNavigationJson(out.value().navigation,out.value().rows,out.value().centerline,false);
    REQUIRE(json.isOk());
    REQUIRE(json.value().find("\"lateral_error_m\":null")!=std::string::npos);
    REQUIRE(json.value().find("\"state\":\"acquiring\"")!=std::string::npos);
    REQUIRE(json.value().find("\"track_id\":")!=std::string::npos);
}

TEST_CASE("tracker config is opt in and rejects invalid activation budgets") {
    const auto id=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path=std::filesystem::temp_directory_path()/("rk-track-config-"+std::to_string(id)+".yaml");
    std::ofstream(path)<<"%YAML:1.0\nschema_version: 1\n";
    auto cfg=rk_nav::loadTemporalTrackerConfig(path); REQUIRE(cfg.isOk() && !cfg.value().enabled);
    std::ofstream(path)<<"%YAML:1.0\nrow_tracker:\n   enabled: 1\n   confirmation_frames: 2\n";
    cfg=rk_nav::loadTemporalTrackerConfig(path); REQUIRE(cfg.isOk() && cfg.value().enabled && cfg.value().confirmation_frames==2);
    std::ofstream(path)<<"%YAML:1.0\nrow_tracker:\n   enabled: 2\n";
    REQUIRE(!rk_nav::loadTemporalTrackerConfig(path).isOk());
    std::ofstream(path)<<"%YAML:1.0\nrow_tracker:\n   confirmation_frames: 0\n";
    REQUIRE(!rk_nav::loadTemporalTrackerConfig(path).isOk());
    std::filesystem::remove(path);
}

TEST_CASE("rejected target does not remain locked indefinitely after sustained movement") {
    auto pipeline=trackedPipeline();
    std::uint64_t first=0;
    bool reacquired=false;
    for (std::uint64_t i=0;i<40;++i) {
        const auto out=pipeline.process(rk_nav::test::sequenceFrame("sustained_jump",i).segmentation);
        REQUIRE(out.isOk()); const auto& nav=out.value().navigation;
        if (i==24) first=nav.tracking.left_id;
        if (i>=25 && i<=27) REQUIRE(nav.status!=rk_nav::TrackingStatus::kValid);
        if (i>=25 && nav.status==rk_nav::TrackingStatus::kValid) {
            REQUIRE(nav.tracking.left_id!=first);
            REQUIRE(std::abs(nav.lateral_error_m-.26)<.01);
            reacquired=true;
        }
    }
    REQUIRE(reacquired);
}

TEST_CASE("missing-frame budget is enforced at high frame rate and low confidence cannot confirm") {
    rk_nav::TemporalTrackerConfig config=enabledConfig(); config.max_missed_frames=1;
    rk_nav::TrackedPerceptionPipeline pipeline(rk_nav::test::sequencePipeline(),config);
    for (std::uint64_t i=0;i<3;++i) REQUIRE(pipeline.process(rk_nav::test::sequenceFrame("drift",i).segmentation).isOk());
    for (std::uint64_t i=3;i<=4;++i) {
        auto frame=rk_nav::test::sequenceFrame("drift",i).segmentation;
        frame.capture_time_ns=1200000000+static_cast<std::int64_t>(i-2)*1000000;
        frame.combined_mask.setTo(0);
        const auto out=pipeline.process(frame); REQUIRE(out.isOk());
        REQUIRE(out.value().navigation.tracking.state==(i==3 ? rk_nav::RowTrackingState::kCoasting : rk_nav::RowTrackingState::kLost));
    }
    auto low_pipeline=trackedPipeline();
    for (std::uint64_t i=0;i<3;++i) {
        auto frame=rk_nav::test::sequenceFrame("drift",i).segmentation;
        frame.instances[0].confidence=.01F;
        const auto out=low_pipeline.process(frame); REQUIRE(out.isOk());
        REQUIRE(out.value().navigation.status==rk_nav::TrackingStatus::kLowConfidence);
        REQUIRE(out.value().navigation.tracking.tracks.empty());
    }
    const auto first=low_pipeline.process(rk_nav::test::sequenceFrame("drift",3).segmentation);
    REQUIRE(first.isOk() && first.value().navigation.tracking.state==rk_nav::RowTrackingState::kAcquiring);
}

TEST_CASE("new target confirmation requires the same adjacent pair across consecutive frames") {
    auto pipeline=trackedPipeline();
    for (std::uint64_t i=0;i<5;++i) {
        rk_nav::test::RowScene scene; scene.spacing=.3; scene.center=i==1 ? .18 : .12;
        auto frame=rk_nav::test::sequenceFrame("drift",i).segmentation;
        frame.combined_mask=rk_nav::test::sceneMask(scene);
        const auto out=pipeline.process(frame); REQUIRE(out.isOk());
        REQUIRE(out.value().navigation.tracking.raw_valid);
        if (i<4) REQUIRE(out.value().navigation.tracking.state==rk_nav::RowTrackingState::kAcquiring);
        else REQUIRE(out.value().navigation.status==rk_nav::TrackingStatus::kValid);
    }
}
