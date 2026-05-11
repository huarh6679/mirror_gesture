#include "camera_calib.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>

void printCalibrationResult(const StereoCalibration& calib) {
    std::cout << "\n========================================\n";
    std::cout << "       Calibration Results\n";
    std::cout << "========================================\n";
    std::cout << "Camera Model: " << (calib.model == CAMERA_PINHOLE ? "Pinhole" : "Fisheye") << "\n";

    std::cout << "\nLeft Camera Intrinsics (K1):\n";
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < calib.K1.rows; ++i) {
        for (int j = 0; j < calib.K1.cols; ++j) {
            std::cout << std::setw(12) << calib.K1.at<double>(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nLeft Camera Distortion (D1):\n";
    for (int i = 0; i < calib.D1.rows; ++i) {
        std::cout << std::setw(12) << calib.D1.at<double>(i, 0);
    }
    std::cout << "\n";

    std::cout << "\nRight Camera Intrinsics (K2):\n";
    for (int i = 0; i < calib.K2.rows; ++i) {
        for (int j = 0; j < calib.K2.cols; ++j) {
            std::cout << std::setw(12) << calib.K2.at<double>(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nRight Camera Distortion (D2):\n";
    for (int i = 0; i < calib.D2.rows; ++i) {
        std::cout << std::setw(12) << calib.D2.at<double>(i, 0);
    }
    std::cout << "\n";

    std::cout << "\nRotation Matrix (R):\n";
    for (int i = 0; i < calib.R.rows; ++i) {
        for (int j = 0; j < calib.R.cols; ++j) {
            std::cout << std::setw(12) << calib.R.at<double>(i, j) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nTranslation Vector (T):\n";
    std::cout << "  (mm): ";
    for (int i = 0; i < calib.T.rows; ++i) {
        std::cout << std::setw(12) << calib.T.at<double>(i, 0);
    }
    std::cout << "\n  (m) : ";
    for (int i = 0; i < calib.T.rows; ++i) {
        std::cout << std::setw(12) << calib.T.at<double>(i, 0) / 1000.0;
    }
    std::cout << "\n";

    std::cout << "\nStereo RMS Error: " << calib.rms << "\n";
    std::cout << "========================================\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage:\n";
        std::cout << "  " << argv[0] << " <config_file.ini>\n";
        std::cout << "  " << argv[0] << " --undistort <calibration.yaml> <left_image> <right_image>\n";
        std::cout << "\nExample:\n";
        std::cout << "  " << argv[0] << " calib_config.ini\n";
        return 0;
    }

    std::string arg1 = argv[1];
    if (arg1 == "--undistort") {
        if (argc < 5) {
            std::cerr << "Usage: " << argv[0] << " --undistort <calibration.yaml> <left_image> <right_image>\n";
            return -1;
        }

        StereoCalibration calib;
        calib.model = CAMERA_FISHEYE;
        StereoCameraCalibration calibration(CAMERA_FISHEYE);

        if (!calibration.loadCalibration(argv[2], calib)) {
            std::cerr << "Failed to load calibration file!\n";
            return -1;
        }

        calibration.setCameraModel(calib.model);

        cv::Mat leftImg = cv::imread(argv[3]);
        cv::Mat rightImg = cv::imread(argv[4]);

        if (leftImg.empty() || rightImg.empty()) {
            std::cerr << "Failed to load images!\n";
            return -1;
        }

        cv::Mat leftUndistorted, rightUndistorted;
        calibration.stereoUndistortRectify(leftImg, leftUndistorted, rightImg, rightUndistorted, calib);

        cv::imshow("Left Original", leftImg);
        cv::imshow("Right Original", rightImg);
        cv::imshow("Left Rectified", leftUndistorted);
        cv::imshow("Right Rectified", rightUndistorted);
        cv::waitKey(0);

        cv::imwrite("left_rectified.png", leftUndistorted);
        cv::imwrite("right_rectified.png", rightUndistorted);
        std::cout << "Rectified images saved!\n";

        return 0;
    }

    CalibConfig config;
    config.boardSize = cv::Size(9, 6);
    config.squareSize = 30.0f;
    config.enableDebug = false;

    if (!StereoCameraCalibration::loadConfig(argv[1], config)) {
        std::cerr << "Failed to load config file!\n";
        return -1;
    }

    std::cout << "========================================\n";
    std::cout << "Camera Calibration Configuration\n";
    std::cout << "========================================\n";
    std::cout << "Camera Model: " << (config.cameraModel == CAMERA_PINHOLE ? "Pinhole" : "Fisheye") << "\n";
    std::cout << "Left Images: " << config.leftImagePath << "\n";
    std::cout << "Right Images: " << config.rightImagePath << "\n";
    std::cout << "Calibration File: " << config.calibrationFile << "\n";
    std::cout << "Board Size: " << config.boardSize.width << "x" << config.boardSize.height << "\n";
    std::cout << "Square Size: " << config.squareSize << " mm\n";
    std::cout << "========================================\n";

    std::vector<std::string> leftImages, rightImages;
    if (!StereoCameraCalibration::loadImages(config.leftImagePath, leftImages)) {
        std::cerr << "Failed to load left images!\n";
        return -1;
    }
    if (!StereoCameraCalibration::loadImages(config.rightImagePath, rightImages)) {
        std::cerr << "Failed to load right images!\n";
        return -1;
    }

    std::cout << "\nLoaded " << leftImages.size() << " left images\n";
    std::cout << "Loaded " << rightImages.size() << " right images\n";

    if (leftImages.size() != rightImages.size()) {
        std::cerr << "Error: Number of left and right images must match!\n";
        return -1;
    }

    if (config.enableDebug) {
        std::cout << "\nImage pair verification:\n";
        std::cout << "------------------------\n";
        for (size_t i = 0; i < leftImages.size(); ++i) {
            std::string leftName = leftImages[i].substr(leftImages[i].find_last_of('/') + 1);
            std::string rightName = rightImages[i].substr(rightImages[i].find_last_of('/') + 1);
            std::cout << "Pair " << std::setw(3) << i << ": " << leftName << " <-> " << rightName << "\n";
        }
        std::cout << "------------------------\n";
    }

    StereoCameraCalibration calibration(config.cameraModel);
    StereoCalibration calib;

    std::cout << "\nStarting stereo calibration...\n";
    if (!calibration.calibrateStereoCamera(leftImages, rightImages, config.boardSize, config.squareSize, calib)) {
        std::cerr << "Calibration failed!\n";
        return -1;
    }

    printCalibrationResult(calib);

    std::cout << "\nPerforming stereo rectification...\n";
    if (!calibration.stereoRectify(calib)) {
        std::cerr << "Stereo rectification failed!\n";
        return -1;
    }

    calibration.saveCalibration(config.calibrationFile, calib);

    if (!config.outputPath.empty()) {
        std::filesystem::create_directories(config.outputPath);

        cv::Mat leftImg = cv::imread(leftImages[0]);
        cv::Mat rightImg = cv::imread(rightImages[0]);

        if (!leftImg.empty() && !rightImg.empty()) {
            cv::Mat leftRect, rightRect;
            calibration.stereoUndistortRectify(leftImg, leftRect, rightImg, rightRect, calib);

            std::string leftOutput = config.outputPath + "/left_rectified.png";
            std::string rightOutput = config.outputPath + "/right_rectified.png";
            cv::imwrite(leftOutput, leftRect);
            cv::imwrite(rightOutput, rightRect);
            std::cout << "Rectified images saved to: " << config.outputPath << "\n";
        }
    }

    std::cout << "Calibration complete!\n";
    return 0;
}
