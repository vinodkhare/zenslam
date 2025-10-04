#include "keypoint.h"

size_t zenslam::keypoint::index_next { };

zenslam::keypoint::keypoint(const KeyPoint &keypoint) :
    KeyPoint { keypoint } {}
