#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>

#include <fstream>
#include <iostream>

std::vector<std::vector<cv::Vec3f>> objectPoints;
std::vector<std::vector<cv::Vec2f>> imagePoints;

void readWorldCoordinates();
void readImageCoordinates();

int main( int argc, char** argv )
{
    // Variable for error in camera calibration
    double output;

    readImageCoordinates();
    readWorldCoordinates();

    //    Printing vectors
//    for(int i=0; i<imagePoints.size();i++){
//        for(int j=0; j<imagePoints[i].size(); j++){
//            printf("%f ", imagePoints[i][j][0]);
//            printf("%f ", imagePoints[i][j][1]);
//        }
//    }
//    printf("\n");
//
//    for(int i=0; i<objectPoints.size();i++){
//        for(int j=0; j<objectPoints[i].size(); j++){
//            printf("%f ", objectPoints[i][j][0]);
//            printf("%f ", objectPoints[i][j][1]);
//            printf("%f ", objectPoints[i][j][2]);
//        }
//    }
//    printf("\n");


    // Camera Matrix
    cv::Mat cameraMatrix(3 ,3, CV_32FC1, 0.0);
    // Guess Camera Matrix
    cameraMatrix.at<float>(0 ,0) = 40.0; //fx
    cameraMatrix.at<float>(1 ,1) = 40.0; //fy
    cameraMatrix.at<float>(2 ,2) = 1280/960; //aspect ratio
    cameraMatrix.at<float>(0 ,1) = 0.0; //skew
    cameraMatrix.at<float>(0 ,2) = 0.0; //cx
    cameraMatrix.at<float>(1 ,2) = 0.0; //cy
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    // Calibration Camera
    output = cv::calibrateCamera(objectPoints, imagePoints, cv::Size(1280,960), cameraMatrix, distCoeffs, rvecs, tvecs, 1);
    // printing Camera Matrix
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            printf("%f ", cameraMatrix.at<float>(i,j));
        }
        printf("\n");
    }

    printf("Running smoothly\n");

    return 0;
}

void readWorldCoordinates(){
    std::ifstream infile("../Data/world_coordinates.txt");

    if (infile.is_open()) {
        std::string line;
        while (std::getline(infile, line)) {

            // Remove leading and trailing brackets
            line.erase(0, 1);  // Remove '('
            line.erase(line.size() - 1, 1);  // Remove ')'
            
            // Split the line into comma-separated values
            std::stringstream ss(line);
            std::string token;
            std::vector<float> point;  // Temporary vector to store x and y coordinates
            std::vector<cv::Vec3f> vectorOfpoints; // Temporary vector to store Vec2f

            int i = 0;
            while (std::getline(ss, token, ',')) {
                if(i==0){
                    token.erase(0,1);
                }
                if(i%3==0 && i!=0){
                    token.erase(0,1);
                }
                if(i%3==2 && i>0){
                    token.pop_back();
                }
                point.push_back(std::stof(token));
              
                i++;
            }
            
            // Saving points in vector
            for(int i=0; i<point.size()-2; i=i+3){
                cv::Vec3f element(point[i], point[i+1], point[i+2]);
                vectorOfpoints.push_back(element);
            }

            // Saving vector in objectPoints
            objectPoints.push_back(vectorOfpoints);
        }
        infile.close();

    } else {
        std::cerr << "Error opening world_coordinates file!" << std::endl;
    }

}

void readImageCoordinates(){
    std::ifstream infile("../Data/image_coordinates.txt");

    if (infile.is_open()) {
        std::string line;
        while (std::getline(infile, line)) {

            // Remove leading and trailing brackets
            line.erase(0, 1);  // Remove '('
            line.erase(line.size() - 1, 1);  // Remove ')'
            
            // Split the line into comma-separated values
            std::stringstream ss(line);
            std::string token;
            std::vector<float> point;  // Temporary vector to store x and y coordinates
            std::vector<cv::Vec2f> vectorOfpoints; // Temporary vector to store Vec2f

            int i = 0;
            while (std::getline(ss, token, ',')) {
                if(i==0){
                    token.erase(0,1);
                }
                if(i%2==0 && i!=0){
                    token.erase(1,1);
                }
                if(i%2==1){
                    token.pop_back();
                }
                
                point.push_back(std::stof(token));
                
                i++;
            }
            
            // Saving points in vector
            for(int i=0; i<point.size()-1; i=i+2){
                cv::Vec2f element(point[i], point[i+1]);
                vectorOfpoints.push_back(element);
            }

            // Saving vector in imagePoints
            imagePoints.push_back(vectorOfpoints);
        }
        infile.close();

    } else {
        std::cerr << "Error opening image_coordinates file!" << std::endl;
    }
}
