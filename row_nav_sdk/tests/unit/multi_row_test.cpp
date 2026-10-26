#include "test_framework.hpp"
#include "support/row_scenes.hpp"
#include "rk_nav/geometry/centerline_fitter.hpp"
#include "rk_nav/io/navigation_output.hpp"
#include <limits>
#include <fstream>
#include <chrono>

TEST_CASE("associated rows recover all configured spacings gaps clutter and offset") {
    const auto projector = rk_nav::test::sceneProjector();
    for (const auto& scene : rk_nav::test::rowScenes()) {
        const auto mask=rk_nav::test::sceneMask(scene);
        const auto result=rk_nav::RowExtractor(rk_nav::test::sceneConfig(scene,
            rk_nav::RowExtractionMethod::kAssociatedRows)).extractBetweenRows(mask,projector);
        REQUIRE(result.isOk());
        const auto& rows=result.value();
        REQUIRE(rows.navigation_pair_valid==scene.expected_valid);
        REQUIRE(rows.fitted_rows.size()>=3);
        if (!scene.expected_valid) { REQUIRE(rows.centerline_ground_points.empty()); continue; }
        REQUIRE(rows.selected_left>=0 && rows.selected_right>=0);
        REQUIRE(rows.fitted_rows[static_cast<std::size_t>(rows.selected_left)].id !=
                rows.fitted_rows[static_cast<std::size_t>(rows.selected_right)].id);
        const auto fitted=rk_nav::CenterlineFitter{}.fit(rows.centerline_ground_points);
        REQUIRE(fitted.isOk());
        REQUIRE(std::abs(fitted.value().intercept_m-scene.center)<.015);
        REQUIRE(std::abs(fitted.value().slope-scene.slope)<.005);
        REQUIRE(std::abs(rows.median_row_spacing_m-scene.spacing)<.02);
        REQUIRE(rows.confidence>.65F);
        if (scene.short_gap || scene.bridge) REQUIRE(rows.accepted_bands<rows.sampled_bands);
        if (scene.weeds) REQUIRE(!rows.rejected_points.empty());
    }
}

TEST_CASE("association has one observation per band and distinct row identities") {
    std::vector<rk_nav::RowBand> bands;
    for (std::uint64_t i=0;i<15;++i) {
        const double x=.12*double(i);
        bands.push_back({i,x,{{x,.4+.02*x},{x,-.4+.02*x}}});
    }
    const auto rows=rk_nav::MultiRowAssociator{}.associate(bands);
    REQUIRE(rows.size()==2);
    REQUIRE(rows[0].points.size()==15 && rows[1].points.size()==15);
    REQUIRE(rows[0].id!=rows[1].id);
    for (const auto& row : rows) for (std::size_t i=1;i<row.bands.size();++i)
        REQUIRE(row.bands[i]>row.bands[i-1]);
}

TEST_CASE("multi-row pipeline preserves rejected selection diagnostics without navigation") {
    auto scene=rk_nav::test::rowScenes().back();
    const auto projector=rk_nav::test::sceneProjector();
    rk_nav::SegmentationResult seg;
    seg.combined_mask=rk_nav::test::sceneMask(scene); seg.instances.push_back({{},0,1.F,{}});
    rk_nav::PerceptionPipeline pipeline(projector,rk_nav::RowExtractor(rk_nav::test::sceneConfig(scene,
        rk_nav::RowExtractionMethod::kAssociatedRows)),rk_nav::CenterlineFitter{});
    const auto result=pipeline.process(seg);
    REQUIRE(result.isOk());
    REQUIRE(result.value().navigation.status==rk_nav::TrackingStatus::kNoRow);
    REQUIRE(result.value().rows.has_value());
    REQUIRE(!result.value().rows->rejection_reason.empty());
    REQUIRE(!result.value().centerline.has_value());
    REQUIRE(rk_nav::renderBevOverlay(result.value(),projector).isOk());
    const auto json=rk_nav::serializeNavigationJson(result.value().navigation,result.value().rows,result.value().centerline,false);
    REQUIRE(json.isOk());
    REQUIRE(json.value().find("\"pair_valid\":false")!=std::string::npos);
    REQUIRE(json.value().find("\"rows\":[{")!=std::string::npos);
}

TEST_CASE("multi-row extraction rejects invalid settings and empty observations") {
    const auto projector=rk_nav::test::sceneProjector();
    auto config=rk_nav::test::sceneConfig({},rk_nav::RowExtractionMethod::kAssociatedRows);
    const cv::Mat mask=cv::Mat::zeros(projector.config().output_size,CV_8UC1);
    const auto empty=rk_nav::RowExtractor(config).extractBetweenRows(mask,projector);
    REQUIRE(empty.isOk() && !empty.value().navigation_pair_valid);
    config.multi_row.association_gate_m=std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!rk_nav::RowExtractor(config).extractBetweenRows(mask,projector).isOk());
}

TEST_CASE("joint fit keeps distinct boundary slopes under a soft constraint") {
    rk_nav::FittedRow left,right;
    for (std::uint64_t i=0;i<40;++i) {
        const double x=.4+.12*double(i);
        left.inliers.emplace_back(x,.035*x+.4);
        right.inliers.emplace_back(x,.015*x-.4);
    }
    const auto fit=rk_nav::RowBoundaryFitter{}.fitPair(left,right);
    REQUIRE(fit.isOk());
    REQUIRE(fit.value().left_slope>fit.value().right_slope+.005);
    REQUIRE(std::abs((fit.value().left_slope+fit.value().right_slope)*.5-.025)<.001);
    REQUIRE(fit.value().rms_m<.01);
}

TEST_CASE("distant fragments and two same-side rows cannot invent a vehicle corridor") {
    const auto projector=rk_nav::test::sceneProjector();
    auto config=rk_nav::test::sceneConfig({},rk_nav::RowExtractionMethod::kAssociatedRows);
    cv::Mat mask=cv::Mat::zeros(projector.config().output_size,CV_8UC1);
    for (const double y : {.4,.8}) cv::line(mask,projector.groundToBev({.4,y}),
        projector.groundToBev({5.8,y}),cv::Scalar(255),3);
    auto result=rk_nav::RowExtractor(config).extractBetweenRows(mask,projector);
    REQUIRE(result.isOk() && !result.value().navigation_pair_valid);
    mask.setTo(0);
    for (const double y : {-.4,.4}) cv::line(mask,projector.groundToBev({2.,y}),
        projector.groundToBev({5.8,y}),cv::Scalar(255),3);
    result=rk_nav::RowExtractor(config).extractBetweenRows(mask,projector);
    REQUIRE(result.isOk() && !result.value().navigation_pair_valid);
    REQUIRE(result.value().fitted_rows.size()==2);
}

TEST_CASE("diverging rows are rejected instead of forced parallel") {
    const auto projector=rk_nav::test::sceneProjector();
    auto config=rk_nav::test::sceneConfig({},rk_nav::RowExtractionMethod::kAssociatedRows);
    cv::Mat mask=cv::Mat::zeros(projector.config().output_size,CV_8UC1);
    for (int v=59;v<600;++v) {
        const double x=(639-v)*.01;
        for (const double sign : {-1.,1.}) cv::circle(mask,projector.groundToBev({x,sign*(.4+.15*x)}),
            2,cv::Scalar(255),cv::FILLED);
    }
    const auto result=rk_nav::RowExtractor(config).extractBetweenRows(mask,projector);
    REQUIRE(result.isOk() && !result.value().navigation_pair_valid);
    REQUIRE(result.value().rejection_reason=="adjacent row directions disagree");
}

TEST_CASE("BEV geometry settings support baseline selection and validate spacing prior") {
    const auto id=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path=std::filesystem::temp_directory_path()/("rk-row-config-"+std::to_string(id)+".yaml");
    std::ofstream(path)<<"%YAML:1.0\nschema_version: 1\n";
    auto result=rk_nav::loadRowExtractorConfig(path);
    REQUIRE(result.isOk() && result.value().method==rk_nav::RowExtractionMethod::kAssociatedRows);
    std::ofstream(path)<<"%YAML:1.0\nrow_extractor:\n   method: nearest_band\n";
    result=rk_nav::loadRowExtractorConfig(path);
    REQUIRE(result.isOk() && result.value().method==rk_nav::RowExtractionMethod::kNearestBandBaseline);
    std::ofstream(path)<<"%YAML:1.0\nrow_extractor:\n   method: associated\n   expected_spacing_m: 0.4\n   spacing_tolerance_m: 0.06\n";
    result=rk_nav::loadRowExtractorConfig(path);
    REQUIRE(result.isOk() && std::abs(result.value().multi_row.expected_spacing_m-.4)<1e-8);
    std::ofstream(path)<<"%YAML:1.0\nrow_extractor:\n   method: unknown\n";
    REQUIRE(!rk_nav::loadRowExtractorConfig(path).isOk());
    std::ofstream(path)<<"%YAML:1.0\nrow_extractor:\n   expected_spacing_m: -1.\n";
    REQUIRE(!rk_nav::loadRowExtractorConfig(path).isOk());
    std::ofstream(path)<<"%YAML:1.0\nrow_extractor:\n   expected_spacing_m: text\n";
    REQUIRE(!rk_nav::loadRowExtractorConfig(path).isOk());
    std::filesystem::remove(path);
}
