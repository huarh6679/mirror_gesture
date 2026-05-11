#include "pinhole_calib.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

PinholeStereoCalibration::PinholeStereoCalibration()
    : m_flags(cv::CALIB_FIX_INTRINSIC)
    , m_rectifyInitialized(false)
{
}

PinholeStereoCalibration::~PinholeStereoCalibration() {
}

void PinholeStereoCalibration::setCameraModel(int flags) {
    m_flags = flags;
}

bool PinholeStereoCalibration::calibrateLeftCamera(
    const std::vector<std::string>& imagePaths,
    cv::Size boardSize,
    cv::Mat& K1, cv::Mat& D1)
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::Size imageSize;

    for (size_t i = 0; i < imagePaths.size(); ++i) {
        cv::Mat img = cv::imread(imagePaths[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "Cannot load image: " << imagePaths[i] << std::endl;
            continue;
        }

        if (i == 0) {
            imageSize = img.size();
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cornerSubPix(img, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
            imagePoints.push_back(corners);
        }
    }

    if (imagePoints.empty()) {
        std::cerr << "No valid chessboard corners found!" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>>().swap(objectPoints);
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        std::vector<cv::Point3f> obj;
        for (int r = 0; r < boardSize.height; ++r) {
            for (int c = 0; c < boardSize.width; ++c) {
                obj.push_back(cv::Point3f(c * 30.0f, r * 30.0f, 0.0f));
            }
        }
        objectPoints.push_back(obj);
    }

    K1 = cv::Mat::eye(3, 3, CV_64F);
    D1 = cv::Mat::zeros(5, 1, CV_64F);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, K1, D1,
                                      rvecs, tvecs);

    std::cout << "Left Camera RMS: " << rms << std::endl;
    return true;
}

bool PinholeStereoCalibration::calibrateRightCamera(
    const std::vector<std::string>& imagePaths,
    cv::Size boardSize,
    cv::Mat& K2, cv::Mat& D2)
{
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::Size imageSize;

    for (size_t i = 0; i < imagePaths.size(); ++i) {
        cv::Mat img = cv::imread(imagePaths[i], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "Cannot load image: " << imagePaths[i] << std::endl;
            continue;
        }

        if (i == 0) {
            imageSize = img.size();
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, boardSize, corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cornerSubPix(img, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
            imagePoints.push_back(corners);
        }
    }

    if (imagePoints.empty()) {
        std::cerr << "No valid chessboard corners found!" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>>().swap(objectPoints);
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        std::vector<cv::Point3f> obj;
        for (int r = 0; r < boardSize.height; ++r) {
            for (int c = 0; c < boardSize.width; ++c) {
                obj.push_back(cv::Point3f(c * 30.0f, r * 30.0f, 0.0f));
            }
        }
        objectPoints.push_back(obj);
    }

    K2 = cv::Mat::eye(3, 3, CV_64F);
    D2 = cv::Mat::zeros(5, 1, CV_64F);

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, K2, D2,
                                      rvecs, tvecs);

    std::cout << "Right Camera RMS: " << rms << std::endl;
    return true;
}

bool PinholeStereoCalibration::calibrateStereoCamera(
    const std::vector<std::string>& leftImagePaths,
    const std::vector<std::string>& rightImagePaths,
    cv::Size boardSize,
    float squareSize,
    StereoPinholeCalibration& calibration)
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

        if (i == 0) {
            imageSize = img1.size();
        }

        std::vector<cv::Point2f> corners1, corners2;
        bool found1 = cv::findChessboardCorners(img1, boardSize, corners1,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        bool found2 = cv::findChessboardCorners(img2, boardSize, corners2,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found1 && found2) {
            cornerSubPix(img1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
            cornerSubPix(img2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
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
            std::cout << "  Frame " << i << ": Chessboard NOT found (" 
                      << (found1 ? "left OK, right FAIL" : found2 ? "left FAIL, right OK" : "both FAIL") << ")\n";
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
                                       calibration.K1, calibration.D1,
                                       rvecs1, tvecs1);
    std::cout << "Left Camera RMS: " << rms1 << std::endl;

    double rms2 = cv::calibrateCamera(objectPoints, imagePoints2, imageSize,
                                       calibration.K2, calibration.D2,
                                       rvecs2, tvecs2);
    std::cout << "Right Camera RMS: " << rms2 << std::endl;

    calibration.R = cv::Mat::eye(3, 3, CV_64F);
    calibration.T = cv::Mat::zeros(3, 1, CV_64F);

    int flags = m_flags;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6);
    
    calibration.rms = cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2,
                                           calibration.K1, calibration.D1,
                                           calibration.K2, calibration.D2,
                                           imageSize, calibration.R, calibration.T,
                                           calibration.E, calibration.F, flags, criteria);

    std::cout << "Stereo Calibration RMS: " << calibration.rms << std::endl;

    calibration.imageSize = imageSize;
    calibration.isValid = true;

    m_objectPoints = objectPoints;
    m_imagePoints1 = imagePoints1;
    m_imagePoints2 = imagePoints2;

    return true;
}

bool PinholeStereoCalibration::stereoRectify(
    StereoPinholeCalibration& calibration,
    cv::Size newImageSize)
{
    if (!calibration.isValid) {
        std::cerr << "Invalid calibration data!" << std::endl;
        return false;
    }

    if (newImageSize.width == 0 || newImageSize.height == 0) {
        newImageSize = calibration.imageSize;
    }
    m_newImageSize = newImageSize;

    cv::stereoRectify(calibration.K1, calibration.D1,
                      calibration.K2, calibration.D2,
                      calibration.imageSize, calibration.R, calibration.T,
                      calibration.R1, calibration.R2,
                      calibration.P1, calibration.P2, calibration.Q,
                      cv::CALIB_ZERO_DISPARITY, 0, m_newImageSize);

    computeRectificationMaps(calibration);
    m_rectifyInitialized = true;

    return true;
}

void PinholeStereoCalibration::computeRectificationMaps(
    const StereoPinholeCalibration& calibration)
{
    cv::initUndistortRectifyMap(calibration.K1, calibration.D1, calibration.R1,
                                 calibration.P1, m_newImageSize, CV_16SC2,
                                 m_map1l, m_map2l);
    cv::initUndistortRectifyMap(calibration.K2, calibration.D2, calibration.R2,
                                 calibration.P2, m_newImageSize, CV_16SC2,
                                 m_map1r, m_map2r);
}

void PinholeStereoCalibration::undistortImage(
    const cv::Mat& input, cv::Mat& output,
    const cv::Mat& K, const cv::Mat& D,
    bool useOptimalProjection)
{
    cv::Size outputSize = input.size();

    if (useOptimalProjection) {
        cv::undistort(input, output, K, D);
    } else {
        cv::Mat map1, map2;
        cv::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F),
                                     K, outputSize, CV_16SC2, map1, map2);
        remap(input, output, map1, map2, cv::INTER_LINEAR);
    }
}

void PinholeStereoCalibration::stereoUndistortRectify(
    const cv::Mat& leftInput, cv::Mat& leftOutput,
    const cv::Mat& rightInput, cv::Mat& rightOutput,
    const StereoPinholeCalibration& calibration)
{
    if (!m_rectifyInitialized) {
        StereoPinholeCalibration calibCopy = calibration;
        if (!stereoRectify(calibCopy)) {
            std::cerr << "Failed to initialize stereo rectification!" << std::endl;
            return;
        }
    }

    remap(leftInput, leftOutput, m_map1l, m_map2l, cv::INTER_LINEAR);
    remap(rightInput, rightOutput, m_map1r, m_map2r, cv::INTER_LINEAR);
}

void PinholeStereoCalibration::saveCalibration(
    const std::string& filename,
    const StereoPinholeCalibration& calibration)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

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
    std::cout << "Calibration saved to: " << filename << " (T in meters)" << std::endl;
}

bool PinholeStereoCalibration::loadCalibration(
    const std::string& filename,
    StereoPinholeCalibration& calibration)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Cannot open calibration file: " << filename << std::endl;
        return false;
    }

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

    calibration.imageSize = cv::Size(calibration.imageSize.width, calibration.imageSize.height);
    calibration.isValid = true;

    fs.release();
    std::cout << "Calibration loaded from: " << filename << " (T converted from meters to mm)" << std::endl;

    computeRectificationMaps(calibration);
    m_rectifyInitialized = true;

    return true;
}

std::vector<cv::Point2f> PinholeStereoCalibration::findChessboardCorners(
    const cv::Mat& image, cv::Size boardSize, bool showResult)
{
    std::vector<cv::Point2f> corners;

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    bool found = cv::findChessboardCorners(gray, boardSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
    }

    if (showResult && found) {
        cv::Mat display;
        cv::cvtColor(gray, display, cv::COLOR_GRAY2BGR);
        drawChessboardCorners(display, boardSize, corners, found);
        cv::imshow("Chessboard Corners", display);
        cv::waitKey(0);
    }

    return corners;
}

bool PinholeStereoCalibration::loadImages(
    const std::string& path,
    std::vector<std::string>& imagePaths)
{
    std::vector<cv::String> files;
    cv::glob(path, files);

    for (const auto& file : files) {
        std::string ext = file.substr(file.find_last_of('.') + 1);
        if (ext == "jpg" || ext == "png" || ext == "jpeg" || ext == "bmp" || ext == "tif") {
            imagePaths.push_back(file);
        }
    }

    std::sort(imagePaths.begin(), imagePaths.end());
    return !imagePaths.empty();
}
