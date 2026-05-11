#ifndef FISH_EYE_CALIB_HPP
#define FISH_EYE_CALIB_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>

struct StereoFishEyeCalibration {
    cv::Mat K1, D1;
    cv::Mat K2, D2;
    cv::Mat R, T;
    cv::Mat E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize;
    float rms;
    bool isValid;
};

class FishEyeStereoCalibration {
public:
    FishEyeStereoCalibration();
    ~FishEyeStereoCalibration();

    void setCameraModel(int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                              cv::fisheye::CALIB_CHECK_COND);

    bool calibrateLeftCamera(const std::vector<std::string>& imagePaths,
                              cv::Size boardSize,
                              cv::Mat& K1, cv::Mat& D1);

    bool calibrateRightCamera(const std::vector<std::string>& imagePaths,
                               cv::Size boardSize,
                               cv::Mat& K2, cv::Mat& D2);

    bool calibrateStereoCamera(const std::vector<std::string>& leftImagePaths,
                                const std::vector<std::string>& rightImagePaths,
                                cv::Size boardSize,
                                float squareSize,
                                StereoFishEyeCalibration& calibration);

    bool stereoRectify(StereoFishEyeCalibration& calibration,
                       cv::Size newImageSize = cv::Size());

    void undistortImage(const cv::Mat& input, cv::Mat& output,
                        const cv::Mat& K, const cv::Mat& D,
                        bool useOptimalProjection = true);

    void stereoUndistortRectify(const cv::Mat& leftInput, cv::Mat& leftOutput,
                                 const cv::Mat& rightInput, cv::Mat& rightOutput,
                                 const StereoFishEyeCalibration& calibration);

    void saveCalibration(const std::string& filename,
                         const StereoFishEyeCalibration& calibration);

    bool loadCalibration(const std::string& filename,
                         StereoFishEyeCalibration& calibration);

    static std::vector<cv::Point2f> findFishEyeChessboardCorners(
        const cv::Mat& image, cv::Size boardSize, bool showResult = false);

    static bool loadImages(const std::string& path,
                           std::vector<std::string>& imagePaths);

private:
    int m_flags;
    cv::Size m_newImageSize;
    cv::Mat m_map1l, m_map2l, m_map1r, m_map2r;
    bool m_rectifyInitialized;

    std::vector<std::vector<cv::Point3f>> m_objectPoints;
    std::vector<std::vector<cv::Point2f>> m_imagePoints1, m_imagePoints2;

    void computeRectificationMaps(StereoFishEyeCalibration& calibration);
};

#endif
