#pragma once
#include <array>
#include <string_view>
namespace reverse_assist {
// Baked raster dimensions; assets/chinese_cues.json is the source of these labels.
inline constexpr std::array<std::string_view, 13> chinese_cue_text{
    "\xe5\x85\xb3\xe9\x97\xad", // off
    "\xe7\xbc\x96\xe8\xbe\x91\xe7\x9b\xae\xe6\xa0\x87", // edit
    "\xe8\xa7\x84\xe5\x88\x92\xe4\xb8\xad", // wait
    "\xe5\xb7\xa6\xe6\x89\x93", // left
    "\xe5\x8f\xb3\xe6\x89\x93", // right
    "\xe4\xbf\x9d\xe6\x8c\x81", // hold
    "\xe5\x9b\x9e\xe6\xad\xa3", // straight
    "\xe5\x81\x9c\xe8\xbd\xa6", // stop
    "\xe5\xb7\xb2\xe5\x88\xb0\xe4\xbd\x8d", // arrived
    "\xe6\x90\x9c\xe7\xb4\xa2\xe8\xb6\x85\xe9\x99\x90", // limit
    "\xe5\x90\x91\xe5\x89\x8d\xe8\xb0\x83\xe6\x95\xb4", // move
    "\xe6\xa3\x80\xe6\x9f\xa5\xe6\x95\xb0\xe6\x8d\xae", // data
    "\xe5\x87\x8f\xe9\x80\x9f", // slow
};
inline constexpr std::array<double, 13> chinese_cue_width{
1.828571428571, 3.716129032258, 2.742857142857, 1.858064516129, 1.888524590164, 1.828571428571, 1.952542372881, 1.828571428571, 2.742857142857, 3.657142857143, 3.657142857143, 3.657142857143, 1.828571428571
};
}
