#pragma once

#include <optional>
#include <string>
#include <utility>

namespace rk_nav {

enum class StatusCode {
    kOk = 0,
    kInvalidArgument,
    kNotFound,
    kUnavailable,
    kTimeout,
    kEndOfStream,
    kInternal
};

class Status {
public:
    Status() = default;
    Status(StatusCode code, std::string message) : code_(code), message_(std::move(message)) {}

    static Status ok() { return {}; }
    static Status invalidArgument(std::string message) { return {StatusCode::kInvalidArgument, std::move(message)}; }
    static Status notFound(std::string message) { return {StatusCode::kNotFound, std::move(message)}; }
    static Status unavailable(std::string message) { return {StatusCode::kUnavailable, std::move(message)}; }
    static Status timeout(std::string message) { return {StatusCode::kTimeout, std::move(message)}; }
    static Status endOfStream(std::string message = "end of stream") { return {StatusCode::kEndOfStream, std::move(message)}; }
    static Status internal(std::string message) { return {StatusCode::kInternal, std::move(message)}; }

    [[nodiscard]] bool isOk() const { return code_ == StatusCode::kOk; }
    [[nodiscard]] StatusCode code() const { return code_; }
    [[nodiscard]] const std::string& message() const { return message_; }

private:
    StatusCode code_{StatusCode::kOk};
    std::string message_;
};

template <typename T>
class Result {
public:
    Result(T value) : value_(std::move(value)), status_(Status::ok()) {}
    Result(Status status) : status_(std::move(status)) {}

    [[nodiscard]] bool isOk() const { return status_.isOk() && value_.has_value(); }
    [[nodiscard]] const Status& status() const { return status_; }
    T& value() { return value_.value(); }
    const T& value() const { return value_.value(); }

private:
    std::optional<T> value_;
    Status status_;
};

}  // namespace rk_nav
