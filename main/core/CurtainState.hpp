#pragma once

namespace curtain {

enum class CurtainState {
    Unknown,
    Open,
    Closed,
    Opening,
    Closing,
    Stopped,
    Stalled,
};

}  // namespace curtain
