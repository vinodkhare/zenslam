#include "stereo_folder_reader.h"

zenslam::stereo_folder_reader::stereo_folder_reader
(
    const path_type &left_dir,
    const path_type &right_dir,
    const double     timescale
) :
    _left(left_dir, false, timescale),
    _right(right_dir, false, timescale) {}

zenslam::stereo_folder_reader::stereo_folder_reader(const class options::folder &options):
    _left { options.root / options.left, false, options.timescale },
    _right { options.root / options.right, false, options.timescale }
{

}


