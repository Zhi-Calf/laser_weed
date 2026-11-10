#pragma once
#include <type_traits>
#include <utility>
namespace rk_nav::detail {
template<class F> class ScopeExit {
    static_assert(std::is_nothrow_invocable_v<F>, "cleanup must not throw");
public:
    explicit ScopeExit(F cleanup) : cleanup_(std::move(cleanup)) {}
    ScopeExit(const ScopeExit&)=delete;
    ScopeExit& operator=(const ScopeExit&)=delete;
    ~ScopeExit() noexcept { cleanup_(); }
private:
    F cleanup_;
};
}
