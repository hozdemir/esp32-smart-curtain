#pragma once

namespace curtain {

enum class ErrorCode {
    None,
    InvalidArgument,
    NotInitialized,
    HardwareFailure,
};

/** Carries command success or a small, actionable error category without exceptions. */
class Result {
public:
    [[nodiscard]] static constexpr Result success() { return Result{ErrorCode::None}; }
    [[nodiscard]] static constexpr Result failure(ErrorCode error) { return Result{error}; }

    [[nodiscard]] constexpr bool ok() const { return error_ == ErrorCode::None; }
    [[nodiscard]] constexpr explicit operator bool() const { return ok(); }
    [[nodiscard]] constexpr ErrorCode error() const { return error_; }

private:
    explicit constexpr Result(ErrorCode error) : error_{error} {}

    ErrorCode error_;
};

}  // namespace curtain
