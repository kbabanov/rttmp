#ifndef RTTMP_HPP
#define RTTMP_HPP

#include <array>
#include <tuple>
#include <cmath>

namespace rttmp {
    template<class T = float>
    struct MotionRange {
        T tBegin       = {};
        T tEnd         = {};
        T acceleration = {};
    };

    template<class T = float>
    std::array<MotionRange<T>, 3> generate(T positionError,
                                           T stateSpeed,
                                           T goalSpeed,
                                           T acceleration,
                                           T deceleration,
                                           T speedLimit) noexcept {

        if(std::abs(positionError) <= std::numeric_limits<T>::epsilon() ||  // already there
           std::abs(acceleration)  <= std::numeric_limits<T>::epsilon() ||  // no solution
           std::abs(deceleration)  <= std::numeric_limits<T>::epsilon() ||  // no solution
           std::abs(speedLimit)    <= std::numeric_limits<T>::epsilon() ) { // no solution
            return {};
        }

        acceleration = std::abs(acceleration);
        deceleration = std::abs(deceleration);
        speedLimit   = std::abs(speedLimit);

        if(std::abs(goalSpeed) > speedLimit) {
            goalSpeed = (goalSpeed < 0 ? -1 : 1) * speedLimit;
        }

        auto getSAndTime = [](auto currentSpeed, auto maxSpeed, auto acceleration) -> std::tuple<T, T> {
            auto s  = (maxSpeed * maxSpeed - currentSpeed * currentSpeed)     / (2 * acceleration);
            auto su = ((maxSpeed - currentSpeed) * (maxSpeed - currentSpeed)) / (2 * acceleration);
            auto t  = (s + su) / maxSpeed;
            return {s, t};
        };

        if(std::abs(stateSpeed) > speedLimit) {
            speedLimit = std::abs(stateSpeed);
        }

        auto direction = positionError >= 0? 1 : -1;
        stateSpeed *= direction;
        goalSpeed  *= direction;
        auto [s2max,  ta] = getSAndTime(stateSpeed, speedLimit, acceleration);
        auto [s2goal, td] = getSAndTime(goalSpeed , speedLimit, deceleration);

        auto s         = std::abs(positionError);
        auto sConst    = s - s2max - s2goal;

        // trapezoidal form
        if(sConst >= 0) {
            auto tc = sConst / speedLimit;
            return {
                    MotionRange<T>{.tBegin = 0,       .tEnd = ta,           .acceleration = +direction * acceleration},
                    MotionRange<T>{.tBegin = ta,      .tEnd = ta + tc,      .acceleration = 0                        },
                    MotionRange<T>{.tBegin = ta + tc, .tEnd = ta + tc + td, .acceleration = -direction * deceleration}
            };
        }

        // already running too fast, stopping as much as possible
        if(std::abs(ta) <= std::numeric_limits<decltype(ta)>::epsilon()) {
            auto a = (deceleration / 2);
            auto b = stateSpeed;
            auto c = s;
            auto d = b * b - 4 * a * c;
            auto t = std::abs((-b + std::sqrt(d)) / (2 * a));

            return {
                    MotionRange<T>{.tBegin = 0, .tEnd = 0, .acceleration = 0},
                    MotionRange<T>{.tBegin = 0, .tEnd = 0, .acceleration = 0                        },
                    MotionRange<T>{.tBegin = 0, .tEnd = t, .acceleration = -direction * deceleration}
            };
        }

        // triangular form
        auto a  = acceleration;
        auto d  = deceleration;
        auto Vs = stateSpeed;
        auto Vg = goalSpeed;

        auto v1 = Vs * Vs + 2 * a * s;
        auto v2 = Vg * Vg;

        if(v1 < v2) {
            // no solution, compute nearest possible Vg
            Vg = sqrt(Vs * Vs + 2 * a * s);
        }

        auto vCross = std::sqrt((d * Vs * Vs + a * Vg * Vg + 2 * a * d * s) / (a + d));
        if(vCross <= std::numeric_limits<decltype(s)>::epsilon()) {
            return {};
        }

        std::tie(s2max,  ta) = getSAndTime(Vs, vCross, a);
        std::tie(s2goal, td) = getSAndTime(Vg, vCross, d);

        if(ta < 0) {
            // there is no precise solution, returning closest
            return {
                    MotionRange<T>{.tBegin = 0,   .tEnd = -ta, .acceleration = -direction * deceleration},
                    MotionRange<T>{.tBegin = -ta, .tEnd = -ta, .acceleration = 0                        },
                    MotionRange<T>{.tBegin = -ta, .tEnd = -ta, .acceleration = 0                        }
            };
        }

        return {
                MotionRange<T>{.tBegin = 0,  .tEnd = ta,      .acceleration = +direction * acceleration},
                MotionRange<T>{.tBegin = ta, .tEnd = ta,      .acceleration = 0                        },
                MotionRange<T>{.tBegin = ta, .tEnd = ta + td, .acceleration = -direction * deceleration}
        };
    }
}

#endif //RTTMP_HPP
