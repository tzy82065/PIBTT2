// orientation.cpp
#include "../include/orientation.hpp"

std::map<Orientation, int> dir_to_angle = {
    {Orientation::X_PLUS, 0},
    {Orientation::Y_PLUS, 90},
    {Orientation::X_MINUS, 180},
    {Orientation::Y_MINUS, 270}
};