#include "camera_calib.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

StereoCameraCalibration::StereoCameraCalibration(CameraModel model)
    : m_model(model), m_rectifyInitialized(false)
{
}

StereoCameraCalibration::~StereoCameraCalibration() {
}

void StereoCameraCalibration::setCameraModel(CameraModel model) {
    m_model = model;
    m_rectifyInitialized = false;
}

bool StereoCameraCalibration::calibrateStereoCamera(
    const std::vector<std::string>& leftImagePaths,
    const std::vector<std::string>& rightImagePaths,
    cv::Size boardSize,
    float squareSize,
    StereoCalibration& calibration)
{
    calibration.model = m_model;

    if (m_model == CAMERA_PINHOLE) {
        return calibratePinhole(leftImagePaths, rightImagePaths, boardSize, squareSize, calibration);
    } else {
        return calibrateFisheye(leftImagePaths, rightImagePaths, boardSize, squareSize, calibration);
    }
}

bool StereoCameraCalibration::calibratePinhole(
    const std::vector<std::string>& leftImagePaths,
    const std::vector<std::string>& rightImagePaths,
    cv::Size boardSize,
    float squareSize,
    StereoCalibration& calibration)
{
    if (leftImagePaths.size() != rightImagePaths.size()) {
        std::cerr << "Left and right image counts must match!" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
    cv::Size imageSize;

    for (size_t i = 0; i < leftImagePaths.size(); ++i) {
        cv::Mat img1 = cv::imread(leftImagePaths[i], cv::IMREAD_GRAYSCALE);
        cv::Mat img2 = cv::imread(rightImagePaths[i], cv::IMREAD_GRAYSCALE);

        if (img1.empty() || img2.empty()) {
            std::cerr << "Cannot load images at index: " << i << std::endl;
            continue;
        }

        if (i == 0) imageSize = img1.size();

        std::vector<cv::Point2f> corners1, corners2;
        bool found1 = cv::findChessboardCorners(img1, boardSize, corners1,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found2 = cv::findChessboardCorners(img2, boardSize, corners2,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found1 && found2) {
            cv::cornerSubPix(img1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
            cv::cornerSubPix(img2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);

            std::vector<cv::Point3f> obj;
            for (int r = 0; r < boardSize.height; ++r) {
                for (int c = 0; c < boardSize.width; ++c) {
                    obj.push_back(cv::Point3f(c * squareSize, r * squareSize, 0.0f));
                }
            }
            objectPoints.push_back(obj);
            std::cout << "  Frame " << i << ": Chessboard detected\n";
        } else {
            std::cout << "  Frame " << i << ": Chessboard NOT found\n";
        }
    }

    if (objectPoints.empty()) {
        std::cerr << "No valid stereo image pairs found!" << std::endl;
        return false;
    }

    calibration.K1 = cv::Mat::eye(3, 3, CV_64F);
    calibration.D1 = cv::Mat::zeros(5, 1, CV_64F);
    calibration.K2 = cv::Mat::eye(3, 3, CV_64F);
    calibration.D2 = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs1, tvecs1, rvecs2, tvecs2;
    double rms1 = cv::calibrateCamera(objectPoints, imagePoints1, imageSize,
                                       calibration.K1, calibration.D1, rvecs1, tvecs1);
    std::cout << "Left Camera RMS: " << rms1 << std::endl;

    double rms2 = cv::calibrateCamera(objectPoints, imagePoints2, imageSize,
                                       calibration.K2, calibration.D2, rvecs2, tvecs2);
    std::cout << "Right Camera RMS: " << rms2 << std::endl;

    calibration.R = cv::Mat::eye(3, 3, CV_64F);
    calibration.T = cv::Mat::zeros(3, 1, CV_64F);

    calibration.rms = cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                           calibration.K1, calibration.D1,
                                           calibration.K2, calibration.D2,
                                           imageSize, calibration.R, calibration.T,
                                           calibration.E, calibration.F,
                                           cv::CALIB_FIX_INTRINSIC,
                                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

    std::cout << "Stereo Calibration RMS: " << calibration.rms << std::endl;
    calibration.imageSize = imageSize;
    calibration.isValid = true;

    return true;
}

bool StereoCameraCalibration::calibrateFisheye(
    const std::vector<std::string>& leftImagePaths,
    const std::vector<std::string>& rightImagePaths,
    cv::Size boardSize,
    float squareSize,
    StereoCalibration& calibration)
{
    if (leftImagePaths.size() != rightImagePaths.size()) {
        std::cerr << "Left and right image counts must match!" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints1, imagePoints2;
    cv::Size imageSize;

    for (size_t i = 0; i < leftImagePaths.size(); ++i) {
        cv::Mat img1 = cv::imread(leftImagePaths[i], cv::IMREAD_GRAYSCALE);
        cv::Mat img2 = cv::imread(rightImagePaths[i], cv::IMREAD_GRAYSCALE);

        if (img1.empty() || img2.empty()) {
            std::cerr << "Cannot load images at index: " << i << std::endl;
            continue;
        }

        if (i == 0) imageSize = img1.size();

        std::vector<cv::Point2f> corners1, corners2;
        bool found1 = cv::findChessboardCorners(img1, boardSize, corners1,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found2 = cv::findChessboardCorners(img2, boardSize, corners2,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found1 && found2) {
            cv::cornerSubPix(img1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
            cv::cornerSubPix(img2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);

            std::vector<cv::Point3f> obj;
            for (int r = 0; r < boardSize.height; ++r) {
                for (int c = 0; c < boardSize.width; ++c) {
                    obj.push_back(cv::Point3f(c * squareSize, r * squareSize, 0.0f));
                }
            }
            objectPoints.push_back(obj);
            std::cout << "  Frame " << i << ": Chessboard detected\n";
        } else {
            std::cout << "  Frame " << i << ": Chessboard NOT found\n";
        }
    }

    if (objectPoints.empty()) {
        std::cerr << "No valid stereo image pairs found!" << std::endl;
        return false;
    }

    calibration.K1 = cv::Mat::eye(3, 3, CV_64F);
    calibration.D1 = cv::Mat::zeros(4, 1, CV_64F);
    calibration.K2 = cv::Mat::eye(3, 3, CV_64F);
    calibration.D2 = cv::Mat::zeros(4, 1, CV_64F);

    std::vector<cv::Mat> rvecs1, tvecs1, rvecs2, tvecs2;
    int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_CHECK_COND;

    double rms1 = cv::fisheye::calibrate(objectPoints, imagePoints1, imageSize,
                                          calibration.K1, calibration.D1,
                                          rvecs1, tvecs1, flags);
    std::cout << "Left Camera RMS: " << rms1 << std::endl;

    double rms2 = cv::fisheye::calibrate(objectPoints, imagePoints2, imageSize,
                                          calibration.K2, calibration.D2,
                                          rvecs2, tvecs2, flags);
    std::cout << "Right Camera RMS: " << rms2 << std::endl;

    calibration.R = cv::Mat::eye(3, 3, CV_64F);
    calibration.T = cv::Mat::zeros(3, 1, CV_64F);

    calibration.rms = cv::fisheye::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                                    calibration.K1, calibration.D1,
                                                    calibration.K2, calibration.D2,
                                                    imageSize, calibration.R, calibration.T,
                                                    cv::fisheye::CALIB_FIX_INTRINSIC,
                                                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

    std::cout << "Stereo Calibration RMS: " << calibration.rms << std::endl;
    calibration.imageSize = imageSize;
    calibration.isValid = true;

    return true;
}

bool StereoCameraCalibration::stereoRectify(StereoCalibration& calibration, cv::Size newImageSize) {
    if (!calibration.isValid) {
        std::cerr << "Invalid calibration data!" << std::endl;
        return false;
    }

    if (newImageSize.width == 0 || newImageSize.height == 0) {
        newImageSize = calibration.imageSize;
    }
    m_newImageSize = newImageSize;

    if (calibration.model == CAMERA_PINHOLE) {
        std::cout << "Pinhole calibration" << std::endl;
        cv::stereoRectify(calibration.K1, calibration.D1,
                          calibration.K2, calibration.D2,
                          calibration.imageSize, calibration.R, calibration.T,
                          calibration.R1, calibration.R2,
                          calibration.P1, calibration.P2, calibration.Q,
                          cv::CALIB_ZERO_DISPARITY, 0, m_newImageSize);
    } else {
        std::cout << "Fisheye calibration" << std::endl;
        cv::fisheye::stereoRectify(calibration.K1, calibration.D1,
                                   calibration.K2, calibration.D2,
                                   calibration.imageSize, calibration.R, calibration.T,
                                   calibration.R1, calibration.R2,
                                   calibration.P1, calibration.P2, calibration.Q,
                                   cv::CALIB_ZERO_DISPARITY, m_newImageSize);
    }

    computeRectificationMaps(calibration);
    m_rectifyInitialized = true;

    return true;
}

void StereoCameraCalibration::computeRectificationMaps(const StereoCalibration& calibration) {
    cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    if (calibration.model == CAMERA_PINHOLE) {
        cv::initUndistortRectifyMap(calibration.K1, calibration.D1, calibration.R1,
                                     calibration.P1, m_newImageSize, CV_16SC2,
                                     m_map1l, m_map2l);
        cv::initUndistortRectifyMap(calibration.K2, calibration.D2, calibration.R2,
                                     calibration.P2, m_newImageSize, CV_16SC2,
                                     m_map1r, m_map2r);
    } else {
        cv::Mat n_instrinsic = calibration.K1.clone();
        n_instrinsic.at<double>(0,0) /=  2.0;
        n_instrinsic.at<double>(1,1) /=  2.0;

        cv::fisheye::initUndistortRectifyMap(calibration.K1, calibration.D1, E,
                                              calibration.K1, m_newImageSize, CV_16SC2,
                                              m_map1l, m_map2l);
        cv::fisheye::initUndistortRectifyMap(calibration.K2, calibration.D2, E,
                                              calibration.K1, m_newImageSize, CV_16SC2,
                                              m_map1r, m_map2r);
    }
}

void StereoCameraCalibration::undistortImage(const cv::Mat& input, cv::Mat& output,
                                             const cv::Mat& K, const cv::Mat& D) {
    cv::Size outputSize = input.size();

    if (m_model == CAMERA_PINHOLE) {
        cv::undistort(input, output, K, D);
    } else {
        cv::fisheye::undistortImage(input, output, K, D, K, outputSize);
    }
}

void StereoCameraCalibration::stereoUndistortRectify(const cv::Mat& leftInput, cv::Mat& leftOutput,
                                                      const cv::Mat& rightInput, cv::Mat& rightOutput,
                                                      const StereoCalibration& calibration) {
    if (!m_rectifyInitialized) {
        StereoCalibration calibCopy = calibration;
        if (!stereoRectify(calibCopy)) {
            std::cerr << "Failed to initialize stereo rectification!" << std::endl;
            return;
        }
    }

    cv::remap(leftInput, leftOutput, m_map1l, m_map2l, cv::INTER_LINEAR);
    cv::remap(rightInput, rightOutput, m_map1r, m_map2r, cv::INTER_LINEAR);
}

void StereoCameraCalibration::saveCalibration(const std::string& filename,
                                              const StereoCalibration& calibration) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    fs << "cameraModel" << (calibration.model == CAMERA_PINHOLE ? "pinhole" : "fisheye");
    fs << "imageWidth" << calibration.imageSize.width;
    fs << "imageHeight" << calibration.imageSize.height;
    fs << "rms" << calibration.rms;

    fs << "K1" << calibration.K1;
    fs << "D1" << calibration.D1;
    fs << "K2" << calibration.K2;
    fs << "D2" << calibration.D2;
    fs << "R" << calibration.R;
    fs << "T" << calibration.T / 1000.0;
    fs << "E" << calibration.E;
    fs << "F" << calibration.F;

    if (calibration.isValid) {
        fs << "R1" << calibration.R1;
        fs << "R2" << calibration.R2;
        fs << "P1" << calibration.P1;
        fs << "P2" << calibration.P2;
        fs << "Q" << calibration.Q;
        fs << "newImageWidth" << m_newImageSize.width;
        fs << "newImageHeight" << m_newImageSize.height;
    }

    fs.release();
    std::cout << "Calibration saved to: " << filename << std::endl;
}

bool StereoCameraCalibration::loadCalibration(const std::string& filename,
                                              StereoCalibration& calibration) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Cannot open calibration file: " << filename << std::endl;
        return false;
    }

    std::string modelStr;
    fs["cameraModel"] >> modelStr;
    calibration.model = (modelStr == "pinhole") ? CAMERA_PINHOLE : CAMERA_FISHEYE;

    fs["imageWidth"] >> calibration.imageSize.width;
    fs["imageHeight"] >> calibration.imageSize.height;
    fs["rms"] >> calibration.rms;

    fs["K1"] >> calibration.K1;
    fs["D1"] >> calibration.D1;
    fs["K2"] >> calibration.K2;
    fs["D2"] >> calibration.D2;
    fs["R"] >> calibration.R;
    fs["T"] >> calibration.T;
    calibration.T = calibration.T * 1000.0;
    fs["E"] >> calibration.E;
    fs["F"] >> calibration.F;

    fs["R1"] >> calibration.R1;
    fs["R2"] >> calibration.R2;
    fs["P1"] >> calibration.P1;
    fs["P2"] >> calibration.P2;
    fs["Q"] >> calibration.Q;

    int newWidth, newHeight;
    fs["newImageWidth"] >> newWidth;
    fs["newImageHeight"] >> newHeight;
    m_newImageSize = cv::Size(newWidth, newHeight);

    calibration.isValid = true;
    fs.release();

    computeRectificationMaps(calibration);
    m_rectifyInitialized = true;

    return true;
}

bool StereoCameraCalibration::loadConfig(const std::string& configFile, CalibConfig& config) {
    std::ifstream file(configFile);
    if (!file.is_open()) {
        std::cerr << "Cannot open config file: " << configFile << std::endl;
        return false;
    }

    std::cout << "Loading config from: " << configFile << "\n";

    std::string line;
    int lineNum = 0;
    while (std::getline(file, line)) {
        lineNum++;
        
        size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) {
            line = line.substr(0, commentPos);
        }
        
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;

        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        
        while (key.back() == ' ' || key.back() == '\t') key.pop_back();
        while (key.front() == ' ' || key.front() == '\t') key.erase(key.begin());
        while (value.back() == ' ' || value.back() == '\t') value.pop_back();
        while (value.front() == ' ' || value.front() == '\t') value.erase(value.begin());

        std::cout << "  Line " << lineNum << ": '" << key << "' = '" << value << "'\n";

        if (key == "camera_model") {
            config.cameraModel = (value == "pinhole") ? CAMERA_PINHOLE : CAMERA_FISHEYE;
        } else if (key == "left_images_path") {
            config.leftImagePath = value;
        } else if (key == "right_images_path") {
            config.rightImagePath = value;
        } else if (key == "calibration_file") {
            config.calibrationFile = value;
        } else if (key == "output_path") {
            config.outputPath = value;
        } else if (key == "board_width") {
            config.boardSize.width = std::stoi(value);
        } else if (key == "board_height") {
            config.boardSize.height = std::stoi(value);
        } else if (key == "square_size") {
            config.squareSize = std::stof(value);
        } else if (key == "enable_debug") {
            config.enableDebug = (value == "true" || value == "1");
        }
    }

    file.close();
    return true;
}

bool StereoCameraCalibration::loadImages(const std::string& path, std::vector<std::string>& imagePaths) {
    std::vector<cv::String> files;
    cv::glob(path, files);

    for (const auto& file : files) {
        std::string ext = file.substr(file.find_last_of('.') + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == "jpg" || ext == "png" || ext == "jpeg" || ext == "bmp" || ext == "tif") {
            imagePaths.push_back(file);
        }
    }

    std::sort(imagePaths.begin(), imagePaths.end());
    return !imagePaths.empty();
}
