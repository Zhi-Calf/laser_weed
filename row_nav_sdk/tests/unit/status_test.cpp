#include "test_framework.hpp"
#include "rk_nav/core/status.hpp"

TEST_CASE("default status is ok") {
    const rk_nav::Status status;
    REQUIRE(status.isOk());
    REQUIRE(status.code() == rk_nav::StatusCode::kOk);
}

TEST_CASE("result retains error") {
    const rk_nav::Result<int> result(rk_nav::Status::notFound("missing"));
    REQUIRE(!result.isOk());
    REQUIRE(result.status().code() == rk_nav::StatusCode::kNotFound);
    REQUIRE(result.status().message() == "missing");
}

TEST_CASE("result retains value") {
    const rk_nav::Result<int> result(42);
    REQUIRE(result.isOk());
    REQUIRE(result.value() == 42);
}
