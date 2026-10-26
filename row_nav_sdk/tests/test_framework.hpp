#pragma once

#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rk_nav::test {
using TestFunction = std::function<void()>;

struct TestCase { std::string name; TestFunction function; };

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(std::string name, TestFunction function) {
        registry().push_back({std::move(name), std::move(function)});
    }
};

inline void require(bool condition, const char* expression, const char* file, int line) {
    if (!condition) throw std::runtime_error(std::string(file) + ":" + std::to_string(line) + " requirement failed: " + expression);
}
}  // namespace rk_nav::test

#define RK_NAV_CONCAT_INNER(a, b) a##b
#define RK_NAV_CONCAT(a, b) RK_NAV_CONCAT_INNER(a, b)
#define TEST_CASE(name) \
    static void RK_NAV_CONCAT(test_, __LINE__)(); \
    static ::rk_nav::test::Registrar RK_NAV_CONCAT(registrar_, __LINE__)(name, RK_NAV_CONCAT(test_, __LINE__)); \
    static void RK_NAV_CONCAT(test_, __LINE__)()
#define REQUIRE(expression) ::rk_nav::test::require(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
