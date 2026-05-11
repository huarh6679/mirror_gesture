#ifndef CAMERA_CALIB_HPP
#define CAMERA_CALIB_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>

enum CameraModel {
    CAMERA_PINHOLE,
    CAMERA_FISHEYE
};

struct StereoCalibration {
    cv::Mat K1, D1;
    cv::Mat K2, D2;
    cv::Mat R, T;
    cv::Mat E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize;
    float rms;
    CameraModel model;
    bool isValid;
};

struct CalibConfig {
    CameraModel cameraModel;
    std::string leftImagePath;
    std::string rightImagePath;
    std::string calibrationFile;
    std::string outputPath;
    cv::Size boardSize;
    float squareSize;
    bool enableDebug;
};

class StereoCameraCalibration {
public:
    StereoCameraCalibration(CameraModel model = CAMERA_PINHOLE);
    ~StereoCameraCalibration();

    void setCameraModel(CameraModel model);

    bool calibrateStereoCamera(const std::vector<std::string>& leftImagePaths,
                                const std::vector<std::string>& rightImagePaths,
                                cv::Size boardSize,
                                float squareSize,
                                StereoCalibration& calibration);

    bool stereoRectify(StereoCalibration& calibration,
                       cv::Size newImageSize = cv::Size());

    void undistortImage(const cv::Mat& input, cv::Mat& output,
                        const cv::Mat& K, const cv::Mat& D);

    void stereoUndistortRectify(const cv::Mat& leftInput, cv::Mat& leftOutput,
                                 const cv::Mat& rightInput, cv::Mat& rightOutput,
                                 const StereoCalibration& calibration);

    void saveCalibration(const std::string& filename,
                         const StereoCalibration& calibration);

    bool loadCalibration(const std::string& filename,
                         StereoCalibration& calibration);

    static bool loadConfig(const std::string& configFile, CalibConfig& config);

    static bool loadImages(const std::string& path,
                           std::vector<std::string>& imagePaths);

private:
    CameraModel m_model;
    cv::Size m_newImageSize;
    cv::Mat m_map1l, m_map2l, m_map1r, m_map2r;
    bool m_rectifyInitialized;

    bool calibratePinhole(const std::vector<std::string>& leftImagePaths,
                          const std::vector<std::string>& rightImagePaths,
                          cv::Size boardSize,
                          float squareSize,
                          StereoCalibration& calibration);

    bool calibrateFisheye(const std::vector<std::string>& leftImagePaths,
                          const std::vector<std::string>& rightImagePaths,
                          cv::Size boardSize,
                          float squareSize,
                          StereoCalibration& calibration);

    void computeRectificationMaps(const StereoCalibration& calibration);
};

#endif
