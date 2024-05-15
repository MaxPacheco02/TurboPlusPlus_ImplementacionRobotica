#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
// Does not work with some kernels#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <fstream>

using namespace std;
using namespace cv;

// Camera values
int capture_width = 1280 ;
int capture_height = 720 ;
int display_width = 1280 ;
int display_height = 720 ;
int framerate = 30 ;
int flip_method = 0 ;

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
        std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
        "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
        std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char** argv) {
    std::string pipeline = gstreamer_pipeline(capture_width,
        capture_height,
        display_width,
        display_height,
        framerate,
        flip_method);
    cv::VideoCapture cap;
    cap.open(pipeline, cv::CAP_GSTREAMER); //open raspberrypi camera


    // VideoCapture video_load(0);//capturing video from default camera//
    namedWindow("Adjust");//declaring window to show the image//
    // Test with image 
//    std::string image_path = samples::findFile("../image.jpeg");
//    Mat prevframe = imread(image_path, IMREAD_COLOR);
//    float a = 0.7;
//    Mat frame;
//    resize(prevframe, frame, Size(ceil(prevframe.cols*a), ceil(prevframe.rows*a)), INTER_LINEAR); 
//    Mat showingframe = frame;
    
    //Reading json file
    ifstream colorfile("../Data/max_laptop.json", ifstream::binary);
    Json::Value colors;
    colorfile >> colors;

    string chooseColor = colors["type"].asString();
    
    // HUE values
    int Hue_Lower_Value = colors[chooseColor]["hue_low"].asInt();//initial hue value(lower)
    int Hue_Lower_Upper_Value = colors[chooseColor]["hue_up"].asInt();//initial hue value(upper)
    int Saturation_Lower_Value = colors[chooseColor]["sat_low"].asInt();//initial saturation(lower)
    int Saturation_Upper_Value = colors[chooseColor]["sat_up"].asInt();//initial saturation(upper)
    int Value_Lower = colors[chooseColor]["val_low"].asInt();//initial value (lower)
    int Value_Upper = colors[chooseColor]["val_up"].asInt();//initial saturation(upper)


    // Trackbars
    createTrackbar("Hue_Lower", "Adjust", &Hue_Lower_Value, 179);//track-bar for lower hue//
    createTrackbar("Hue_Upper", "Adjust", &Hue_Lower_Upper_Value, 179);//track-bar for lower-upper hue//
    createTrackbar("Sat_Lower", "Adjust", &Saturation_Lower_Value, 255);//track-bar for lower saturation//
    createTrackbar("Sat_Upper", "Adjust", &Saturation_Upper_Value, 255);//track-bar for higher saturation//
    createTrackbar("Val_Lower", "Adjust", &Value_Lower, 255);//track-bar for lower value//
    createTrackbar("Val_Upper", "Adjust", &Value_Upper, 255);//track-bar for upper value//

  // Countour variables 
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Point> rectPoints(2);
  Rect rect;
    while (1) {
        Mat frame;//matrix to load actual image//
        cap.read(frame);
        // bool temp = video_load.read(frame);//loading actual image to matrix from video stream//
        
        Mat convert_to_HSV;//declaring a matrix to store converted image//
        cvtColor(frame, convert_to_HSV, COLOR_BGR2HSV);//converting BGR image to HSV and storing it in convert_to_HSV matrix//
        Mat detection_screen;//declaring matrix for window where object will be detected//
        inRange(convert_to_HSV,Scalar(Hue_Lower_Value,Saturation_Lower_Value, Value_Lower),Scalar(Hue_Lower_Upper_Value,Saturation_Upper_Value, Value_Upper), detection_screen);
        // Cleaning matrix
        erode(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological opening for removing small objects from foreground//
        dilate(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological opening for removing small object from foreground//
        dilate(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological closing for filling up small holes in foreground//
        erode(detection_screen, detection_screen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));//morphological closing for filling up small holes in foreground//

        // Finding Contours
        findContours(detection_screen, contours, RETR_LIST, CHAIN_APPROX_NONE);
        for(int i=0; i<contours.size(); i++){
            float area = contourArea(contours[i]);
            if (area >2000){
                //printf("%f \n", area);
                rect = boundingRect(contours[i]); 
                rectPoints[0].x = rect.x; 
                rectPoints[0].y = rect.y; 
                rectPoints[1].x = rect.x + rect.width; 
                rectPoints[1].y = rect.y + rect.height; 
//                rectangle(showingframe, rectPoints[0], rectPoints[1], CV_RGB(0,255,0), 2);
                drawContours(frame, contours, i, CV_RGB(0,0,255), 2);
            }
        }

        // Showing only red in image
        //Mat resultImage;
        //bitwise_and(frame, frame, resultImage, detection_screen);
        //imshow("Bitwise", resultImage);//showing detected object//
        // Showing Threeshold and original image
        imshow("Threesholded Image", detection_screen);//showing detected object//
        imshow("Original", frame);//showing actual image//

        if (waitKey(30) == 27){ //if esc is press break the loop//
            break;
        }
    }
    return 0;
}
