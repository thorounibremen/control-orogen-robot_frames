#ifndef robot_frames_TYPES_HPP
#define robot_frames_TYPES_HPP

#include <string>

namespace robot_frames {
    struct Chain{
        std::string name;
        std::string root_link;
        std::string tip_link;
    };
}

#endif

