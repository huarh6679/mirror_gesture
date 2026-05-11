
#include "split_stereo.hpp"
#include <algorithm>

namespace stereo_toolkit
{

StereoSplit::StereoSplit(const YAML::Node& config_node)
{
	root_dir_ = config_node["data_root"].as<string>();
	device_type_ = config_node["device_type"].as<string>();
    images_data_ = root_dir_ + "/" + config_node["images_data"].as<string>();
    left_calib_data_ = root_dir_ + "/" + config_node["left_calib_data"].as<string>(); 
    right_calib_data_ = root_dir_ + "/" + config_node["right_calib_data"].as<string>(); 

    cout << ">>>>>>>>>>>>>>>>>> Config Info: <<<<<<<<<<<<<<<<" << endl
     << "data_root: " << root_dir_ << endl
	 << "device_type: " << device_type_ << endl
     << "images_data: " << images_data_ << endl
     << "left_calib_data: " << left_calib_data_ << endl
     << "right_calib_data: " << right_calib_data_ << endl
     << "--------------------------------------------------" << std::endl;
}
    
StereoSplit::~StereoSplit()
{

}

bool StereoSplit::Run()
{
	if(!SplitLeftRight()){
		return false;
	}
	return true;
}

bool StereoSplit::SplitLeftRight()
{
    std::vector<String> file_list;
    glob(images_data_, file_list, false);
    std::cout << "files size: " << file_list.size() << std::endl;
	vector<string> image_list;
    for(std::string image_name : file_list){
        image_list.push_back(image_name);
    }

    // for(auto image_name : image_list){
    //     std::cout << image_name << std::endl;
    // }

	int n_img = (int)image_list.size();
	Mat frame, output;
	Mat temp = imread(image_list[0]);
	uint32_t width = temp.cols, height = temp.rows;
	uint32_t x_start = 0, y_start = 0;
	static int index = 0;
    for(int i = 0; i < n_img; ++i)
    {
		//frame = imread(image_list[i]);
        frame = imread(images_data_ + "/" + std::to_string(i) + ".png");

		if(device_type_ == StereoDevice){
			x_start = 0;
			y_start = 0;
		}
		else{
			fprintf( stderr, "Unknow device type .\n" );
			return false;
		}

        char imgindex[MAX_LEN];

        {
            Rect rect_select = Rect(x_start + width / 2, y_start, width / 2, height);
            Mat output = frame(rect_select).clone();
            snprintf(imgindex, MAX_LEN, "%06d", index);
            //std::string file_name = left_calib_data_ + "/" + std::to_string(index) + ".png";
            std::string file_name = right_calib_data_ + "/" + imgindex + ".png";
            imwrite(file_name, output);
        }
        {
            Rect rect_select = Rect(x_start, y_start, width / 2, height);
            Mat output = frame(rect_select).clone();
            snprintf(imgindex, MAX_LEN, "%06d", index);
            //std::string file_name = right_calib_data_ + "/" + std::to_string(index) + ".png";
            std::string file_name = left_calib_data_ + "/" + imgindex + ".png";
            imwrite(file_name, output);
        }

		index++;
	}
	return true;
}


} //namespace stereo_toolkit