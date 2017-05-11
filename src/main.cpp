///////////////////////////////////////////////////////////////////////////
//
// Author: Akkas Uddin Haque
//
//
///////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include<stdint.h>
#include <opencv2/opencv.hpp>

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include "FuzzyController.h"
using namespace std;

#define DEBUG 1

   FuzzyController fuzzy;
typedef struct mouseOCVStruct {
    float* data;
    uint32_t step;
    cv::Size _image;
    cv::Size _resize;
    std::string name;
    std::string unit;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        mouseOCVStruct* data = (mouseOCVStruct*) param;

        int y_int = (y * data->_image.height / data->_resize.height);
        int x_int = (x * data->_image.width / data->_resize.width);

        float* ptr_image_num = (float*) ((int8_t*) data->data + y_int * data->step);
        float dist = ptr_image_num[x_int];

        if (isValidMeasure(dist))
            printf("\n%s : %2.2f %s\n", data->name.c_str(), dist, data->unit.c_str());
        else {
            if (dist == TOO_FAR)
                printf("\n%s is too far.\n", data->name.c_str(), dist, data->unit.c_str());
            else if (dist == TOO_CLOSE)
                printf("\n%s is too close.\n", data->name.c_str(), dist, data->unit.c_str());
            else
                printf("\n%s not avaliable\n", data->name.c_str(), dist, data->unit.c_str());
        }
    }
}


void calculateFuzzyInputs(float* fuzzyinputs){
    int height = mouseStruct._image.height;
    int width = mouseStruct._image.width;
    
    float* MinValueTable = new float(height*width);
    float* data = mouseStruct.data;
    
    
    float minDist = FLT_MAX;
    for(int i=0;i<9;i++)
        fuzzyinputs[i] = FLT_MAX;
    
    for(int i=0; i<height;i++)
    {
        
        int ii = (i*3)/height;
        for(int j=0; j<width;j++)
        {
            float dist = *(data + (i*width+j));
            if(isValidMeasure(dist))
            {
                int jj = (j*3)/width;
                if(fuzzyinputs[ii*3+jj]>dist)
                    fuzzyinputs[ii*3+jj]=dist;
            }
        }
            
        
    }
    for(int i=0;i<9;i++)
    {fuzzyinputs[i]/=3000;
        if(fuzzyinputs[i]>1) fuzzyinputs[i]=1;
    }
#if DEBUG
    for(int i=0;i<9;i++)
        cout<<" "<<fuzzyinputs[i];
    cout<<endl;
#endif

}



void testfuzzy2(){
    
   float pitch, yaw;
   fuzzy.evaluate(.25,.25,.75,.25,.75,pitch,yaw);
   cout<<endl<<"Pitch : "<<pitch<<"  Yaw  : "<<yaw<<endl;
    //engine->configure("", "", "Minimum", "Maximum", "Centroid");

    

    
    
}

int main(int argc, char **argv) {

    //testfuzzy();
   // testfuzzy2();
  //  exit(0);
  
    if (argc > 3) {
        std::cout << "Only the path of a SVO or a InitParams file can be passed in arg." << std::endl;
        return -1;
    }

    // Quick check input arguments
    bool readSVO = false;
    std::string SVOName;
    bool loadParams = false;
    std::string ParamsName;
    if (argc > 1) {
        std::string _arg;
        for (int i = 1; i < argc; i++) {
            _arg = argv[i];
            if (_arg.find(".svo") != std::string::npos) {
                // If a SVO is given we save its name
                readSVO = true;
                SVOName = _arg;
            }
            if (_arg.find(".ZEDinitParam") != std::string::npos) {
                // If a parameter file is given we save its name
                loadParams = true;
                ParamsName = _arg;
            }
        }
    }

    sl::zed::Camera* zed;

    if (!readSVO) // Live Mode
        zed = new sl::zed::Camera(sl::zed::HD720);
    else // SVO playback mode
        zed = new sl::zed::Camera(SVOName);

    // Define a struct of parameters for the initialization
    sl::zed::InitParams params;

    if (loadParams) // A parameters file was given in argument, we load it
        params.load(ParamsName);

    // Enables verbosity in the console
    params.verbose = true;


    sl::zed::ERRCODE err = zed->init(params);
    std::cout << "Error code : " << sl::zed::errcode2str(err) << std::endl;
    if (err != sl::zed::SUCCESS) {
        // Exit if an error occurred
        delete zed;
        return 1;
    }

    // Save the initialization parameters
    // The file can be used later in any zed based application
    params.save("MyParam");

    char key = ' ';
    int viewID = 0;
    int confidenceThres = 100;

    bool displayDisp = true;
    bool displayConfidenceMap = false;

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    cv::Mat disp(height, width, CV_8UC4);
    cv::Mat anaglyph(height, width, CV_8UC4);
    cv::Mat confidencemap(height, width, CV_8UC4);

    cv::Size displaySize(720, 404);
    cv::Mat dispDisplay(displaySize, CV_8UC4);
    cv::Mat anaglyphDisplay(displaySize, CV_8UC4);
    cv::Mat confidencemapDisplay(displaySize, CV_8UC4);

    sl::zed::SENSING_MODE dm_type = sl::zed::STANDARD;

    // Mouse callback initialization
    sl::zed::Mat depth;
    zed->grab(dm_type);
    depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer
    // Set the structure
    mouseStruct._image = cv::Size(width, height);
    mouseStruct._resize = displaySize;
    mouseStruct.data = (float*) depth.data;
    mouseStruct.step = depth.step;
    mouseStruct.name = "DEPTH";
    mouseStruct.unit = unit2str(params.unit);

    // The depth is limited to 20 METERS, as defined in zed::init()
    zed->setDepthClampValue(10000);

    // Create OpenCV Windows
    // NOTE: You may encounter an issue with OpenGL support, to solve it either
    // 	use the default rendering by removing ' | cv::WINDOW_OPENGL' from the flags
    //	or recompile OpenCV with OpenGL support (you may also need the gtk OpenGL Extension
    //	on Linux, provided by the packages libgtkglext1 libgtkglext1-dev)
	int wnd_flag = cv::WINDOW_AUTOSIZE /*| cv::WINDOW_OPENGL*/;
	cv::namedWindow(mouseStruct.name, wnd_flag);
    cv::setMouseCallback(mouseStruct.name, onMouseCallback, (void*) &mouseStruct);
	cv::namedWindow("VIEW", wnd_flag);

    std::cout << "Press 'q' to exit" << std::endl;

    // Jetson only. Execute the calling thread on core 2
    //sl::zed::Camera::sticktoCPUCore(2);

    sl::zed::ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = sl::zed::SELF_CALIBRATION_NOT_CALLED;

    float fuzzyInputs[9];
    float &center = fuzzyInputs[4];
    float &left = fuzzyInputs[3];
    float &right = fuzzyInputs[5];
    float &up = fuzzyInputs[1];
    float &down = fuzzyInputs[7];
    float &upleft = fuzzyInputs[0];
    float &upright = fuzzyInputs[2];
    float &downleft = fuzzyInputs[6];
    float &downright = fuzzyInputs[8];
    
    
    cv::VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(width,height),true);
    
    cv::VideoWriter dis("disparity.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(width,height),true);
    
    bool paused = false;
    // Loop until 'q' is pressed
    while (key != 'q') {
        // Disparity Map filtering

        zed->setConfidenceThreshold(confidenceThres);
        bool res  = false;

        // Get frames and launch the computation
        if(!paused)
            res = zed->grab(dm_type);

        if (!res) {
            if (old_self_calibration_status != zed->getSelfCalibrationStatus()) {
                std::cout << "Self Calibration Status : " << sl::zed::statuscode2str(zed->getSelfCalibrationStatus()) << std::endl;
                old_self_calibration_status = zed->getSelfCalibrationStatus();
            }

            depth = zed->retrieveMeasure(sl::zed::MEASURE::DEPTH); // Get the pointer

            
            calculateFuzzyInputs(fuzzyInputs);
            
            float pitch, yaw,_pitch,_yaw;
            fuzzy.evaluate(center,left,right,up,down,pitch,yaw);
            if(isnan(pitch)||isnan(yaw)){                
                fuzzy.evaluate(center,upleft,downright,upright,downleft,_pitch,_yaw);
                if(!isnan(_pitch) && !isnan(_yaw)){
                    float _1bysqrt2 = 1.0/sqrt(2);
                    pitch  = _1bysqrt2*(_pitch-_yaw);
                    yaw = _1bysqrt2*(_pitch + _yaw);
                }
                
            }
#if DEBUG
            cout<<"C "<<center<<" L "<<left<<" R "<<right<<" U "<<up<<" D "<<down<<endl;
            cout<<"Pitch : "<<pitch<<"  Yaw  : "<<yaw<<endl;
#endif

            // The following is the best way to retrieve a disparity map / image / confidence map in OpenCV Mat.
            // If the buffer is not duplicated, it will be replaced by a next retrieve (retrieveImage, normalizeMeasure, getView...)
            // Disparity, depth, confidence are 32F buffer by default and 8UC4 buffer in normalized format (displayable grayscale)


            // -- The next part is about displaying the data --

            // Normalize the disparity / depth map in order to use the full color range of gray level image
            if (displayDisp)
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DISPARITY)).copyTo(disp);
            else
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(disp);

            // To get the depth at a given position, click on the disparity / depth map image
            cv::resize(disp, dispDisplay, displaySize);
            imshow(mouseStruct.name, dispDisplay);

            if (displayConfidenceMap) {
                slMat2cvMat(zed->normalizeMeasure(sl::zed::MEASURE::CONFIDENCE)).copyTo(confidencemap);
                cv::resize(confidencemap, confidencemapDisplay, displaySize);
                imshow("confidence", confidencemapDisplay);
            }

            // 'viewID' can be 'SIDE mode' or 'VIEW mode'
            if (viewID >= sl::zed::LEFT && viewID < sl::zed::LAST_SIDE)
                slMat2cvMat(zed->retrieveImage(static_cast<sl::zed::SIDE> (viewID))).copyTo(anaglyph);
            else
                slMat2cvMat(zed->getView(static_cast<sl::zed::VIEW_MODE> (viewID - (int) sl::zed::LAST_SIDE))).copyTo(anaglyph);
//*
            if(!isnan(yaw)&&!isnan(pitch))
                cv::line(anaglyph,cv::Point(width/2,height/2),cv::Point(width/2+yaw*width/2,height/2-pitch*height/2),cv::Scalar(0,0,255),10);
            
            if(!isnan(yaw))
                cv::line(anaglyph,cv::Point(width/2,height-5),cv::Point(width/2+yaw*width/2,height-5),cv::Scalar(0,0,255),10);
            if(!isnan(pitch))
                cv::line(anaglyph,cv::Point(width-5,height/2),cv::Point(width-5,height/2-pitch*height/2),cv::Scalar(0,0,255),10);
           //*/
            

            if(isnan(yaw)&&isnan(pitch))
                cv::putText(anaglyph,"Turn Around",cv::Point(width/2-100,height/2),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,0,255),2);
            video.write(anaglyph);
            dis.write(disp);
            if(viewID == 2){
                cv::resize(anaglyph, anaglyphDisplay, cv::Size(1440, 404));
            }
            else
            cv::resize(anaglyph, anaglyphDisplay, displaySize);
            imshow("VIEW", anaglyphDisplay);


            key = cv::waitKey(5);

            // Keyboard shortcuts
            switch (key) {
                case 'b':
                    if (confidenceThres >= 10)
                        confidenceThres -= 10;
                    break;
                case 'n':
                    if (confidenceThres <= 90)
                        confidenceThres += 10;
                    break;
                    // From 'SIDE' enum
                case '0': // Left
                    viewID = 0;
                    std::cout << "Current View switched to Left (rectified/aligned)" << std::endl;
                    break;
                case '1': // Right
                    viewID = 1;
                    std::cout << "Current View switched to Right (rectified/aligned)" << std::endl;
                    break;
                    // From 'VIEW' enum
                case '2': // Side by Side
                    viewID = 10;
                    std::cout << "Current View switched to Side by Side mode" << std::endl;
                    break;
                case '3': // Overlay
                    viewID = 11;
                    std::cout << "Current View switched to Overlay mode" << std::endl;
                    break;
                case '4': // Difference
                    viewID = 9;
                    std::cout << "Current View switched to Difference mode" << std::endl;
                    break;
                case '5': // Anaglyph
                    viewID = 8;
                    std::cout << "Current View switched to Anaglyph mode" << std::endl;
                    break;
                case 'c':
                    displayConfidenceMap = !displayConfidenceMap;
                    break;
                case 's':
                    dm_type = (dm_type == sl::zed::SENSING_MODE::STANDARD) ? sl::zed::SENSING_MODE::FILL : sl::zed::SENSING_MODE::STANDARD;
                    std::cout << "SENSING_MODE " << sensing_mode2str(dm_type) << std::endl;
                    break;
                case 'd':
                    displayDisp = !displayDisp;
                    break;
                case ' ':
                    paused = !paused;
                break;
            }
        } else key = cv::waitKey(5);
    }

    delete zed;
    return 0;
}
