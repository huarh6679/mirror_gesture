
#ifndef STEREO_TOOLKIT_PRETREATMENT_STEREO_SPLIT_HPP_
#define STEREO_TOOLKIT_PRETREATMENT_STEREO_SPLIT_HPP_

#include <cstdlib>
#include <vector>
#include <iostream>
#include <string>
#include <time.h>

#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;

#define StereoDevice "stereo"
#define MAX_LEN 1024

namespace stereo_toolkit
{

class StereoSplit
{

public:
    StereoSplit(/*args*/) = default;
    StereoSplit(const YAML::Node& config_node);
    ~StereoSplit();

    bool Run();

private:
    bool SplitLeftRight();

private:
    string root_dir_, device_type_;
    string images_data_;
    string left_calib_data_, right_calib_data_;

}; //class StereoSplit

} //namespace stereo_toolkit

#endif //STEREO_TOOLKIT_PRETREATMENT_STEREO_SPLIT_HPP_