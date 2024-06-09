# rttmp
real-time trapezoidal motion profile

# why?
regular trapezoidal motion profile uses **only** state & goal **positions**<br />
whereas this implementation in addition uses state & goal **speeds**

position + speed design allowing to regeneration motion profile at **every step** which gives more accurate traveling and noise resistant behaviour

# how to use
```cpp
auto profile = rttmp::generate(goal.position - state.position,
                               state.speed,
                               goal.speed,
                               params.acceleration,
                               params.deceleration,
                               params.maxSpeed);

// profile is std::array<MotionRange, 3>
// where MotionRange is
//
// template<class T = float>
// struct MotionRange {
//     T tBegin       = {};
//     T tEnd         = {};
//     T acceleration = {};
// };

```

# getting distance, velocity and time
```cpp
std::vector<double> t = {0};              // time
std::vector<double> v = {state.speed};    // velocity
std::vector<double> s = {state.position}; // position

auto getRange = [](auto& ranges, double t) {
    for(size_t i = 0 ; i < 3; ++i) {
        if(i == 0 && t >= ranges[i].tBegin && t < ranges[i].tEnd) {
            return ranges[i];
        } else if(i == ranges.size() - 1 && t > ranges[i].tBegin && t <= ranges[i].tEnd) {
            return ranges[i];
        } else if(t >= ranges[i].tBegin && t <= ranges[i].tEnd) {
            return ranges[i];
        }
    }

    return ranges[2];
};

auto iters = (size_t)(profile[2].tEnd / dt);
if(iters >= 1e6) {
    iters = 0;
}

for(size_t i = 1; i < iters; ++i) {
    double tt = (double)i * dt;
    auto r = getRange(profile, tt);
    t.push_back(tt);
    v.push_back(v[v.size() - 1] + r.acceleration  * dt);
    s.push_back(s[s.size() - 1] + v[v.size() - 1] * dt);
}

// here you can draw/dump v and s to view built profile
```

# generation examples
![Alt text](doc/case1.jpg?raw=true "case 1")
![Alt text](doc/case2.jpg?raw=true "case 2")
![Alt text](doc/case3.jpg?raw=true "case 3")
![Alt text](doc/case4.jpg?raw=true "case 4")
![Alt text](doc/case5.jpg?raw=true "case 5")
![Alt text](doc/case6.jpg?raw=true "case 6")

# used math
![Alt text](doc/math.jpg?raw=true "math")
