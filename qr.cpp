Author:         Minh-Chung HOANG
Email:          MINHCHUN001@e.ntu.edu.sg
Last modified:  Tue 5th Jan 16
Purposes:
- Subscribe to raw image
- Process QR code, if found, in raw image
- The QR code must contain required text information, in the format
    [size t_x t_y t_z r_x r_y r_z type]
        <param> <unit>  <explantion>
        size    meter   true physical distance between two adjacent corners of the QR marker
        t_x     meter   translation from \world to \marker, in world's x-axis
        t_y     meter   translation from world to marker, in world's y-axis
        t_z     meter   translation from \world to \marker, in world's z-axis
        r_x     radian  rotation from world to marker, by world's x-axis (roll)
        r_y     radian  rotation from \world to \marker, by world's y-axis (pitch)
        r_z     radian  rotation from world to marker, by world's z-axis (yaw)
        type    int     (optional) for user's purpose

/*Standard C libs*/
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <string>

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//OpenCV libs
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>

//Namespaces
using namespace ros;
using namespace cv;
using namespace tf;
using namespace std;
using namespace zbar;

//Functions declaration
int read_camera_params(String filename);
void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles);
bool decodeQR(string info, Vec3d &tvec, Vec3d &rvec, double *size, int *type);
double degToRad(double deg);
double radToDeg(double rad);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void broadcastTF_markerToCam(Vec3d translation, Vec3d rotation);
void broadcastTF_worldToMarker(Vec3d translation, Vec3d rotation);
void processingImage();

//Constants
static const std::string OPENCV_WINDOW = "Image window";
#define PI 3.14159265
#define EPSILON 0.001

//Camera params
Mat cameraMatrix, distCoeffs;

//Debug Flags
bool FLAG_SHOW_CPP_DEBUG = false; //Allow printing out important cv::Mat data
bool FLAG_SHOW_QR_IMAGE = true;
bool FLAG_SHOW_CAMERA_PARAM = false;
bool FLAG_SHOW_DECODED_INFO = false;
bool FLAG_SHOW_TF_QR_TO_CAM = true;
bool FLAG_SHOW_TF_WORLD_TO_QR = true;
bool FLAG_VIZ_QR = true;
bool FLAG_VIZ_CENTER = true;

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//Image processing vars
bool newImage = false;
cv::Mat frame, qr_sample;
ImageScanner scanner;

Mat rvec, tvec, rvec_1, tvec_1;
Vec3d eulerAngles;


double half_size = 60.0;
int locationType;

string QRCodeInfo;
string QRCodeType;

//TF vars
double t_x, t_y, t_z, r_x, r_y, r_z;
tf::TransformBroadcaster *broadcaster_worldToMarker;
tf::TransformBroadcaster *broadcaster_cameraToMarker;
Vec3d t_worldToMarker, r_worldToMarker;
Vec3d t_camearToMarker, r_cameraToMarker;

int main(int argc, char** argv)
{
    //Read camera parameters
    //read_camera_params("//home//chung//catkin_ws//out_camera_data.xml");

    //Initializing QR Code scanner
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    //ROS routines
    ros::init(argc, argv, "qr_decoder");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/edrone/camera/image_raw", 1, imageCb);
    image_transport::Publisher pub = it.advertise("/qr_decoder/image_qrDecoded", 1);
    broadcaster_worldToMarker = new tf::TransformBroadcaster();
    broadcaster_cameraToMarker = new tf::TransformBroadcaster();
    ros::Rate r(40);

    while (nh.ok()){
        if (newImage){
            processingImage();
            pub.publish(cv_ptr->toImageMsg());
        }
        ros::spinOnce();
        r.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}

int read_camera_params(String filename){
    ROS_INFO("READ CAMERA PARAM - STARTED");
    FileStorage fs;
    fs.open(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
      //cerr << "Failed to open " << filename << endl;
      return 0;
      ROS_FATAL("FAILED TO READ CAMERA PARAMS. PLEASE CHECK FILE out_camera_data.xml");
    }

    fs["distortion_coefficients"] >> distCoeffs;   // Read cv::Mat
    fs["camera_matrix"] >> cameraMatrix;
    if (FLAG_SHOW_CAMERA_PARAM){
        cout << "distortion_coefs = " << distCoeffs << endl;
        cout << "camera_matrix 	 = " << cameraMatrix << endl << endl;
    }
    ROS_INFO("READ CAMERA PARAM - SUCCESSFUL");
}

void getEulerAngles(Mat &rotCamerMatrix,Vec3d &eulerAngles){
    Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

void broadcastTF_markerToCam(Vec3d translation, Vec3d rotation){
    tf::Transform myTransform;
    t_x = translation[0];
    t_y = translation[1];
    t_z = translation[2];
    r_x = rotation[0];
    r_y = rotation[1];
    r_z = rotation[2];
    myTransform.setOrigin(tf::Vector3(t_x, t_y, t_z));
    myTransform.setRotation(tf::createQuaternionFromRPY(r_x, r_y, r_z));
    broadcaster_cameraToMarker->sendTransform(
        tf::StampedTransform(
            myTransform,
            ros::Time::now(),
            "marker",
            "camera"
        )
    );
}

void broadcastTF_worldToMarker(Vec3d translation, Vec3d rotation){
    tf::Transform myTransform;
    t_x = translation[0];
    t_y = translation[1];
    t_z = translation[2];
    r_x = rotation[0];
    r_y = rotation[1];
    r_z = rotation[2];
    myTransform.setOrigin(tf::Vector3(t_x, t_y, t_z));
    myTransform.setRotation(tf::createQuaternionFromRPY(r_x, r_y, r_z));
    broadcaster_worldToMarker->sendTransform(
        tf::StampedTransform(
            myTransform,
            ros::Time::now(),
            "world",
            "marker"
        )
    );
}

bool decodeQR(string info, Vec3d &tvec, Vec3d &rvec, double *size, int *type){

    char* message = (char*)info.c_str();

    vector<Point3f> markerPoints;
    double tfWorldToQR[6];
    int locationType = 0;
    double true_marker_side = 0.0;
    char *tempToken;
    int count = 0;
    tempToken = strtok(message, " ");
    while (tempToken != NULL){
        switch (count) {
        case 0: true_marker_side = atof(tempToken); break;  //QR side
        case 1: tfWorldToQR[0] = atof(tempToken); break;    //t_x
        case 2: tfWorldToQR[1] = atof(tempToken); break;    //t_y
        case 3: tfWorldToQR[2] = atof(tempToken); break;    //t_z
        case 4: tfWorldToQR[3] = atof(tempToken); break;    //r_x
        case 5: tfWorldToQR[4] = atof(tempToken); break;    //r_y
        case 6: tfWorldToQR[5] = atof(tempToken); break;    //r_z
        case 7: locationType = atoi(tempToken);             //location type
        default: break;
        }
        count++;
        tempToken = strtok(NULL, " ");
    }
    if (count >= 6){
        *size = true_marker_side / 2 * 1000.0; //convert to mm

        tvec[0] = tfWorldToQR[0];
        tvec[1] = tfWorldToQR[1];
        tvec[2] = tfWorldToQR[2];

        rvec[0] = tfWorldToQR[3];
        rvec[1] = tfWorldToQR[4];
        rvec[2] = tfWorldToQR[5];

        *type = locationType;

        return true;
    }
    return false;
}

double degToRad(double deg){
    return deg / 180.0 * PI;
}

double radToDeg(double rad){
    return rad / PI * 180.0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Get the image in OpenCV format
    frame = cv_ptr->image;
    newImage = true;
}

void processingImage(){
    float r_x, r_y, r_z, t_x, t_y, t_z;
    Mat grey;
    cvtColor(frame,grey,CV_BGR2GRAY);

    int width = frame.cols;
    int height = frame.rows;
    uchar *raw = (uchar *)grey.data;

    // wrap image data
    Image image(width, height, "Y800", raw, width * height);

    //scan image for QR code
    int n = scanner.scan(image);

    // extract results
    for(Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end();
        ++symbol)
    {
        QRCodeInfo = symbol->get_data();
        QRCodeType = symbol->get_type_name();
        if (FLAG_SHOW_DECODED_INFO){
            ROS_INFO("QR code decoded: ");
            ROS_INFO("Type:\t%s", QRCodeType.c_str());
            ROS_INFO("Info:\t%s", QRCodeInfo.c_str());
        }

            int center_x = 0;
            int center_y = 0;

        //Number of corners
        int n = symbol->get_location_size();

        //Get the raw image points of QR-marker corners
        vector<Point2f> vp;
        for(int i=0; i<n; i++){
            vp.push_back(Point2f(symbol->get_location_x(i),symbol->get_location_y(i)));
            if (FLAG_VIZ_QR){
                center_x += symbol->get_location_x(i);
                center_y += symbol->get_location_y(i);
            }
        }
        int corner_count = vp.size();
        ROS_DEBUG("Detected raw corners, count: %d", corner_count);
        if (FLAG_SHOW_CPP_DEBUG){
            cout << "Raw corners positions = " << endl;
            cout << vp << endl;
        }


        //Visual debug
            //Draw center, corners 1, 2, 3, and edges of QR marker
        if (FLAG_VIZ_QR){
            center_x = center_x / 4;
            center_y = center_y / 4;
            cv::Point center = cv::Point(center_x, center_y);
            circle(frame, center, 3, Scalar(255,0,0), 8, 0);
            circle(frame, vp[0], 3, Scalar(255,0,0), 8, 0);
            circle(frame, vp[1], 3 ,Scalar(0,255,0), 8, 0);
            circle(frame, vp[2], 3 ,Scalar(0,0,255), 8, 0);
            line(frame,vp[0],vp[1],Scalar(255,0,0),3);
            line(frame,vp[1],vp[2],Scalar(0,255,0),3);
            line(frame,vp[2],vp[3],Scalar(0,0,255),3);
            line(frame,vp[3],vp[0],Scalar(255,255,0),3);
        }
            //Draw center of FOV
        if (FLAG_VIZ_CENTER){
            cv::Point frame_center = cv::Point(480 / 2, 640/ 2);
            circle(frame, frame_center, 3, Scalar(0,255,0), 8, 0);
        }

        //Prepare image points into Mat suitable for solvePnP
        cv::Mat iPoints(4, 1, CV_64FC2);
        for (size_t i = 0, end = vp.size(); i < end; ++i) {
            iPoints.at<double>(i, 0) = vp[i].x;
            iPoints.at<double>(i, 1) = vp[i].y;
        }

        //Prepare object points into Mat suitable for solvePnP
        //Object points
        //Coordinate system aligned with camera: x: right, y: downward, z: into-the-paper
        //PnP points: 1: upper-left, 2: lower-left, 3: lower-right, 4: upper-right
        //Points sequence depends on zbar detection
        cv::vector<cv::Point3f> markerPoints;

        decodeQR(QRCodeInfo, t_worldToMarker, r_worldToMarker, &half_size, &locationType);
        if (FLAG_SHOW_TF_WORLD_TO_QR){
            ROS_INFO("WORLD_TO_QR");
            ROS_INFO("translation:\tt_x:\t %lf \tt_y:\t %lf \tt_z:\t %lf", t_worldToMarker[0], t_worldToMarker[1], t_worldToMarker[2]);
            ROS_INFO("rotation:\tr_x:\t %lf \tr_y:\t %lf \tr_z:\t %lf", r_worldToMarker[0], r_worldToMarker[1], r_worldToMarker[2]);
        }

            //Prepare object point for solvePnP
        markerPoints.push_back(Point3f(-half_size, -half_size, 0.0));
        markerPoints.push_back(Point3f(-half_size, half_size, 0.0));
        markerPoints.push_back(Point3f(half_size, half_size, 0.0));
        markerPoints.push_back(Point3f(half_size, -half_size, 0.0));
            //Compatibility patching
        cv::Mat oPoints(4, 1, CV_64FC3);
        for (size_t i = 0, end = markerPoints.size(); i < end; ++i) {
            oPoints.at<double>(i, 0) = markerPoints[i].x;
            oPoints.at<double>(i, 1) = markerPoints[i].y;
            oPoints.at<double>(i, 2) = markerPoints[i].z;
        }

            //Check suitability with solvePnP
        int oPoint_check = max(oPoints.checkVector(3, CV_32F), oPoints.checkVector(3, CV_64F));
        int iPoint_check = max(iPoints.checkVector(2, CV_32F), iPoints.checkVector(2, CV_64F));
        ROS_DEBUG("oPoint checkVector = %d", oPoint_check);
        ROS_DEBUG("iPoint checkVector = %d", iPoint_check);
        if(FLAG_SHOW_CPP_DEBUG){
            cout << oPoints << endl;
            cout << iPoints << endl;
        }

            //Solve perspective-n-point problem
        cv::solvePnP(oPoints, iPoints, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
        if (FLAG_SHOW_CPP_DEBUG){
            cout << "rvec: " << rvec << endl;
            cout << "tvec: " << tvec << endl;
        }

        //Get Eulerian angles of camera w.r.t marker coordinate system
        cv::Rodrigues(rvec, rvec_1);
        if (FLAG_SHOW_CPP_DEBUG) cout << "rvec_1: " << rvec_1 << endl;
        getEulerAngles(rvec_1,eulerAngles);
        r_x 	= -degToRad(eulerAngles[0]); //rad
        r_y 	= -degToRad(eulerAngles[1]); //rad
        r_z 	= -degToRad(eulerAngles[2]); //rad
        r_cameraToMarker[0] = r_x;
        r_cameraToMarker[1] = r_y;
        r_cameraToMarker[2] = r_z;

        //Get translation of QR w.r.t camera coordinate system
        transpose(rvec_1, rvec_1);
        tvec_1 = rvec_1 * tvec;
        tvec_1 = -tvec_1;
        if (FLAG_SHOW_CPP_DEBUG) cout << "tvec_1: " << tvec_1 << endl;
        t_x = tvec_1.at<double>(0,0) / 1000.0;  //meter
        t_y = tvec_1.at<double>(0,1) / 1000.0;  //meter
        t_z = tvec_1.at<double>(0,2) / 1000.0;  //meter
        t_camearToMarker[0] = t_x;
        t_camearToMarker[1] = t_y;
        t_camearToMarker[2] = t_z;

        //Performing TF
        ROS_INFO("Performing TF --------------------------- ");
            //Send TF marker to camera
        broadcastTF_markerToCam(t_camearToMarker, r_cameraToMarker);
            //Send TF world to marker
        broadcastTF_worldToMarker(t_worldToMarker, r_worldToMarker);

        //Show results
        if(FLAG_SHOW_TF_QR_TO_CAM){
            ROS_INFO("CAM_TO_QR");
            ROS_INFO("translation:\tt_x:\t %lf \tt_y:\t %lf \tt_z:\t %lf", t_x, t_y, t_z);
            ROS_INFO("rotation:\tr_x:\t %lf \tr_y:\t %lf \tr_z:\t %lf", r_x, r_y, r_z);

        }
        //Publish virtual image strea
        cv_ptr->image = frame;
//        frame.toImageMsg(cv_ptr);
    }

//    if (FLAG_SHOW_QR_IMAGE) {
//        imshow("QR Test", qr_sample);
//        moveWindow("QR Test", 900, 100);
//        cv::waitKey(3);
//    }
    newImage = false;
}
