/**
 *                    Face id
 * Face id is a ros node that regonises faces and may be used to identify costumors
 *
 * it uses openCV as a basis for the image prosseing and cv bridge to take the ros
 * msgs and convert them into image for use with openCV.
 *
 * openCV has face detection libareis and trained data sets that can use
 * to detect faces for use in this project.
 *
 * The node will publish an angle and a distance to the costumor.
 *
 * this is made for a fifth semester project
 *
 * made by Grp. 19gr566 at AaU
 *
 * based on http://www.shervinemami.info/faceRecognition.html and
 *          openCV resources on face detection
 *
 *
 *
 * Last compiled 12/11-19
 *
 */

// ROS required libareis

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <signal.h>
//#include <std_msgs/double.h>

//openCV required libareis

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"


//cpp required libareis

#include <vector>
#include <iostream>
#include <math.h>
//costume msg header

#include "faceid/face_detector.h"

using namespace cv;
using namespace std;


// if realsense use 43.5 if asus xtion pro use 29.0
#define CAMANGLE 43.5


const double pi = 3.14159265358979323846;

// variables from openCV to hold CascadeClassifier
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;

// varibles to hold scale to convert between a point in
// color image to a point in the depth image
int centerXDepth = 0;
int centerYDepth = 0;

// varible to hold Cartisian space coordinates calculatede
std::vector< geometry_msgs::Point > vecPoint;

// varible to hold for calculating the varinacse
std::vector< geometry_msgs::Point > vecSamples;
//  varible to hold to determine the size of the sampel
//  varincas to be calculatede
int varSize = 25;

// vector to hold the totalDepths calculatede
std::vector< float > vecDepth;

// varible to hold the angle calculatede
float angle = 0.0;
std::vector< float > vecAngle;

// vectors to hold a confidence score on the face detection
std::vector< int > face_reject_levels, eye_reject_levels;
std::vector< double > face_level_weights, eye_level_weights;
// openCV varible to hold the center of the face of the detected person
Point centerDepth(0, 0);

// openCV varible to hold the images captured
Mat capture_image, depth_image;

// varible to check if the depth image is availabel to work on
bool depth_set = false;

// sequence ID: consecutively increasing ID
uint64 seq = 0;

// set this to true if output is required
const bool viewImage = false;
const bool debugging = false;

double rejectCheck = 0.0;

// a reasons to use a class to hold the functions is
// that this only requires one NodeHandle to be decleared
class faceDetector
{
public:

faceDetector()
{
	// path to the cascade files on the system
	String face_cascade_name = ros::package::getPath("faceid");

	face_cascade_name += "/include/faceid/data/haarcascades/haarcascade_frontalface_alt.xml";

	String eyes_cascade_name = ros::package::getPath("faceid");
	eyes_cascade_name += "/include/faceid/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";

	// 1. Load the cascades
	if (!face_cascade.load(face_cascade_name)) {
		cout << "--(!)Error loading face cascade\n";
		exit(-1);
	}

	if (!eyes_cascade.load(eyes_cascade_name)) {
		cout << "--(!)Error loading eyes cascade\n";
		exit(-1);
	}

	// ImageTransport is handler to communicate with the cv_bridge
	// that is function to help ROS and openCV to be used together.
	image_transport::ImageTransport it(nh);

	// when chatter_pub is used to pulish a msgs it is done on the topic chatter
	chatter_pub = nh.advertise<faceid::face_detector>("/facedetector", 1000);
	rgb_pub = it.advertise("/facedetector/rgb", 1);
	depth_pub = it.advertise("/facedetector/depth", 1);
	// setting a run cycle at 10 Hz
	ros::Rate r(10);

	ros::spinOnce();
	while (ros::ok()) {
		//every time a msgs is published on topic / d400 / color / image_raw
		// the imageCallback function is invoked
		// if openNI2 is used use ("/openni2_camera/rgb/image_raw") as topic
		// if realsense is used use	("/camera/color/image_raw") as topic
		sub = it.subscribe("/camera/color/image_raw", 1, &faceDetector::imageCallback, this);

		// every time a msgs is published on topic /d400/depth/image_rect_raw
		// the depthImageCallback function is invoked
		// if openNI2 is used use ("/openni2_camera/depth/image_raw") as topic
		// if realsense is used use ("/camera/depth/image_rect_raw") as topic
		subd = it.subscribe("/camera/depth/image_rect_raw", 1, &faceDetector::depthImageCallback, this);

		r.sleep();


		ros::spinOnce();
	}

	ros::spinOnce();


	exit(0);
}


~faceDetector()
{
	ros::shutdown();
}


/**
 * This function get a msg and transforms the msg in to and workable image to be
 * used with openCV
 * @param msg The topic of a color image
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//Using cv brigde to work on the image in OpenCV
		capture_image = cv_bridge::toCvShare(msg, "bgr8")->image;

		/**
		 * capture_pre_image = cv_bridge::toCvShare(msg)->image;
		 * double min = 0;
		 * double max = 1000;
		 * cv::Mat(cv_bridge::toCvShare(msg)->image - min).convertTo(capture_image, CV_8UC3, 255. / (max - min));
		 */
		//cout << capture_image.cols << "rows" << capture_image.rows << endl;
		faceDetector::detectAndDisplay(capture_image, msg);

		//waitKey();
		ros::spinOnce();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception in RGB image : %s /n  continuing dont worry :)", e.what());
		return;
	}
	ros::spinOnce();
}
/**
 * This function get a msg and transforms the msg in to and workable depth image
 * to be used with openCV
 * @param msg The topic of a depth image
 */
void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		// using cv brigde to work on the image in OpenCV
		depth_image = cv_bridge::toCvCopy(msg)->image;


		// this check is set when a depth image is able to used to messeaure a distance
		depth_set = true;
		//circle(depth_image, centerDepth, 50.0, Scalar(60000, 255, 255), 2);

		// drawing a circle on the depth image based on the place where the depth is
		// messeaured
		Mat depth_draw = depth_image.clone();
		circle(depth_draw, cv::Point(centerDepth.x * 0.667, centerDepth.y * 0.667), 20.0, Scalar(60000, 255, 255), 5);
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
		out_msg.image = depth_draw;
		depth_pub.publish(out_msg.toImageMsg());
		if (viewImage) {
			// Showing the image and waits one milisecond
			imshow("Capture - depth", depth_draw);


			waitKey(1);
		}
		//cout << depth_image.cols << "rows" << depth_image.rows << endl;

		ros::spinOnce();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception in depth image : %s /n  continuing dont worry :)", e.what());
		return;
	}

	ros::spinOnce();
}
/**
 * this function recognizes faces in a image
 * @param frame Image where faces has to be detected
 */
void detectAndDisplay(Mat frame, const sensor_msgs::ImageConstPtr& inmsg)
{
	try {
		Mat frame_gray;

		// converting color image to grey scaled image, this used in the equalizeHist()
		cvtColor(frame, frame_gray, COLOR_BGR2GRAY);



		const int DETECTION_WIDTH = 320;
		// Possibly shrink the image, to run much faster.
		Mat smallImg;
		float scale = frame.cols / (float)DETECTION_WIDTH;
		if (frame.cols > DETECTION_WIDTH) {
			// Shrink the image while keeping the same aspect ratio.
			int scaledHeight = cvRound(frame.rows / scale);
			resize(frame_gray, smallImg, Size(DETECTION_WIDTH, scaledHeight));
		} else {
			// Access the input directly since it is already small.
			smallImg = frame_gray;
		}

		equalizeHist(smallImg, smallImg);
		// Detect faces
		std::vector<Rect> faces;

/**
 * detectMultiScale takes a scaled image and tries to locate faces with the loade CascadeClassifier
 * @param smallImg           resized image
 * @param faces              vector of rectangles that frames the face
 * @param face_reject_levels vector of int : that tells at what lvl the classifier dropped
 * @param face_level_weights vector of double : that holds the score of each face
 * @param scala              parameter specifying how much the image size is reduced at each image scale.
 * @param min size           minimum size of head
 * @param max size           maximum size of head
 * @param level_weights      bool : if true output score
 */
		face_cascade.detectMultiScale(smallImg, faces, face_reject_levels, face_level_weights, 1.1, 3, false, Size(15, 15), Size(50, 50), true);
		//	printf("amount of faces:  %d ", faces.size());
		for (size_t i = 0; i < faces.size(); i++) {
			//cout << face_level_weights[i] << endl;

			if (face_level_weights[i] > rejectCheck) {
				Point center((faces[i].x + faces[i].width / 2) * scale, (faces[i].y + faces[i].height / 2) * scale);
				centerDepth = center;
				//printf("face width:  %d  face height : %d", faces[i].width, faces[i].height);
				// if the face is detected on the edge return
				if (!(((0 + faces[i].width / 2) * scale) < center.x && ((frame.size().width - faces[i].width / 2) * scale) > center.x && ((0 + faces[i].height / 2) * scale) < center.y && ((frame.size().height - faces[i].height / 2) * scale) > center.y))
					return;
				centerDepth = center;
				ellipse(frame, center, Size((faces[i].width / 2) * scale, (faces[i].height / 2) * scale), 0, 0, 360, Scalar(255, 0, 255), 2);
				// Enlarge the results if the image was temporarily shrunk.
				if (frame.cols > smallImg.cols) {
					for (int i = 0; i < (int)faces.size(); i++) {
						faces[i].x = cvRound(faces[i].x * scale);
						faces[i].y = cvRound(faces[i].y * scale);
						faces[i].width = cvRound(faces[i].width * scale);
						faces[i].height = cvRound(faces[i].height * scale);
					}
					// calculate a depth and angle for the face
				}
				//cout << "frame height: " << frame.rows << "frame witdh: " << frame.cols << endl;
				depthAngle(frame, smallImg);

				// If the object is on a border, keep it in the image.
				for (int i = 0; i < (int)faces.size(); i++) {
					if (faces[i].x < 0)
						faces[i].x = 0;
					if (faces[i].y < 0)
						faces[i].y = 0;
					if (faces[i].x + faces[i].width > frame.cols)
						faces[i].x = frame.cols - faces[i].width;
					if (faces[i].y + faces[i].height > frame.rows)
						faces[i].y = frame.rows - faces[i].height;
				}
				//ellipse(frame, center, Size((faces[i].width / 2) * scale, (faces[i].height / 2) * scale), 0, 0, 360, Scalar(255, 0, 255), 2);

				Mat faceROI = frame_gray(faces[i]);

				// In each face, detect eyes
				std::vector<Rect> eyes;
				/**
				 * detectMultiScale takes a scaled image and tries to locate eye with the loade CascadeClassifier
				 * @param smallImg           resized image
				 * @param faces              vector of rectangles that frames the eye
				 * @param face_reject_levels vector of int : that tells at what lvl the classifier dropped
				 * @param face_level_weights vector of double : that holds the score of each eye
				 * @param scala              parameter specifying how much the image size is reduced at each image scale.
				 * @param min size           minimum size of eye
				 * @param max size           maximum size of eye
				 * @param level_weights      bool : if true output score
				 */

				eyes_cascade.detectMultiScale(faceROI, eyes, eye_reject_levels, eye_level_weights, 1.1, 3, false, Size(0, 0), Size(0, 0), true);

				for (size_t j = 0; j < eyes.size(); j++) {
					// setup for drawing a circle on the eyes
					Point eye_center((faces[i].x + eyes[j].x + eyes[j].width / 2) * scale, (faces[i].y + eyes[j].y + eyes[j].height / 2) * scale);
					int radius = cvRound(((eyes[j].width + eyes[j].height) * 0.25) * scale);
					// drawing a circle on the eyes
					circle(frame, eye_center, radius, Scalar(255, 0, 0), 2);
				}
				if (eyes.size() == 0) {
					eye_level_weights.emplace_back(0.0);
					eye_level_weights.emplace_back(0.0);
				}
				if (eyes.size() == 0)
					eye_level_weights.emplace_back(0.0);

				//publish the message for the kalmanfilter
				pubMsg();
			}


			// Show what you got
			if (viewImage) {
				imshow("Capture - Face detection", frame);
				waitKey(1);
			}
		}
		cv_bridge::CvImage out_msg;
		out_msg.header = inmsg->header;
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
		out_msg.image = frame;
		rgb_pub.publish(out_msg.toImageMsg());
	}
	catch (cv_bridge::Exception& e) {
		//ROS_ERROR(e);
		return;
	}catch (cv::Exception& e) {
		ROS_ERROR("Error in detectAndDisplay %s  /n  continuing dont worry :)", e.what());

		return;
	}
}
/**
 * This function maps a value between a max and a min value
 * to a value between a set max and min output
 * @param  x       value that has to be mapped
 * @param  in_min  lower limit for input
 * @param  in_max  upper limit for input
 * @param  out_min lower limit for output
 * @param  out_max upper limit for output
 * @return         mapped value
 */
float mapp(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * This function combine all the data in a message and publishes the message
 */
void pubMsg()
{
	// declaring the costume msg that will be used to communicate with the
	// the kalmanfilter
	faceid::face_detector msg;

	// writting the differnet values as depth and angle to msg before publishing
	if (vecAngle.size() > 0) {
		seq += 1;
		msg.header.seq = seq;

		msg.header.stamp = ros::Time::now();

		msg.header.frame_id = "";
		//cout << "size of depth vector" << vecDepth.size() << endl;
		for (float f : vecDepth)
			msg.depth.emplace_back(f);

		for (float f : vecAngle)
			msg.angle.emplace_back(f);

		for (double d : face_level_weights)
			msg.face_score.emplace_back(d);

		for (double d : eye_level_weights)
			msg.depth.emplace_back(d);


		for (geometry_msgs::Point p : vecPoint)
			msg.pos.emplace_back(sampleVarCovVariance(p));



		//publishing the msg
		chatter_pub.publish(msg);
	}
	//clearing vectors
	face_level_weights.clear();
	eye_level_weights.clear();
	vecAngle.clear();
	vecDepth.clear();
	vecPoint.clear();


	// debugging
	if (debugging) {
		int size = face_level_weights.size();

		printf("the vector size of face %d \n", size);

		for (int j = 0; j < face_level_weights.size(); j++)
			printf("the level weight of face %d is:  %f\n", j, face_level_weights.at(j));

		for (int j = 0; j < face_reject_levels.size(); j++)
			printf("the reject levels of face %d is:  %d\n", j, face_reject_levels.at(j));

		size = eye_level_weights.size();
		printf("the vector size of eyes %d \n", size);

		for (int j = 0; j < eye_level_weights.size(); j++)
			printf("the level weight of eye %d is:  %f\n", j, eye_level_weights.at(j));

		for (int j = 0; j < eye_reject_levels.size(); j++)
			printf("the reject levels of eye %d is:  %d\n", j, eye_reject_levels.at(j));
	}
}

/**
 * This function calculate a depth from the depth images
 * based on the center of a face and calculates an angle
 * to the face
 * @param frame    color image
 * @param smallImg the resized color image
 */
void depthAngle(Mat frame, Mat smallImg)
{
	// varible to hold the totalDepth calculatede
	double totalDepth = 0.0;
	float _totalDepth = 0.0;
	// the detection width of the depths
	int width_det = 14 / 2;
	// a scalar between depth frame and the frame size of the image where the faces is found
	float scl = 1.0;
	int offset = 0;

	// cout << "center x: " << centerDepth.x * scl << "center y: " << centerDepth.y * scl << endl;
	try {
		if (0 + width_det < floor(centerDepth.x * scl) && depth_image.cols - width_det > floor(centerDepth.x * scl) && 0 + width_det < floor(centerDepth.y * scl) && depth_image.rows - width_det > floor(centerDepth.y * scl)) {
			// if a depth image is accuired calculate the depth and angle
			if (depth_set) {
				// setting the scale probaility between color image and depth image
				scl = 1.30;
				// holder to count the amount of pixels that has to be messeaured
				int divCounter = 0;

				// calculating an avarage of depths to ensure a more presics depths
				// it calculatede from a box based on the center of the face that has
				// the width of width_det
				// cout << "center x: " << centerDepth.x << "center y: " << centerDepth.y << endl;


				for (int y = floor(centerDepth.y) - width_det; y < floor(centerDepth.y) + width_det; y++) {
					for (int x = (floor(centerDepth.x * scl) - width_det) + offset; x < (floor(centerDepth.x * scl) + width_det) + offset; x++) {
						//std::cout << "x is : " << x << "y is : " << y << '\n';
						if ((double)depth_image.at<uint16_t>(y, x) > 0.1) {
							totalDepth += (double)depth_image.at<uint16_t>(y, x);
							divCounter += 1;
							depth_image.at<uchar>(y, x) = (uchar)255;
						}
					}
				}

				// getting the avarage of the depths, the real realsense camera
				// gives a distance in cm, ros works in meters hence th 100
				_totalDepth = ((totalDepth / divCounter) / 1000.0);

				// mapping the value of center of the face to be between 0 and the width
				// of the image to a value between -1.0 to 1.0, this makes it easier
				// to calculate angle.
				angle = faceDetector::mapp(centerDepth.x, 0.0, frame.cols, -1.0, 1.0);
				// calculating the angle based on the mapped value
				angle *= CAMANGLE;

				if (debugging) {
					namedWindow("Display frame", WINDOW_NORMAL);
					imshow("Display frame", depth_image);
					waitKey(100);
					ROS_INFO("Mean distance to the center of the face: %f", _totalDepth);
					ROS_INFO("The faces is at angle: %f", angle);
				}

				// to find the x and y coordinates in a cartisian space, ROS is
				// also working in meters rather then cencimeter, hecne
				geometry_msgs::Point point;
				point.x = -(_totalDepth * cos(angle * pi / 180.0));
				point.y = -(_totalDepth * sin(angle * pi / 180.0));
				vecPoint.emplace_back(point);
				vecDepth.emplace_back(_totalDepth);
				vecAngle.emplace_back(angle);
			}
		}
		return;
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("Error in depthAngel  %s  /n  continuing dont worry :)", e.what());
		return;
	}
}

/**
 * This function calculates the variance for each xy
 * coordinates and a covariance of the xy coordinates
 * @param point xy coordinates of the person
 */
geometry_msgs::PoseWithCovarianceStamped sampleVarCovVariance(geometry_msgs::Point point)
{
	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base_link";
	msg.header.seq = seq;
	msg.pose.pose.position.x = point.x;
	msg.pose.pose.position.y = point.y;
	msg.pose.pose.position.z = 1.30;
	vecSamples.emplace_back(point);
	if (vecSamples.size() > varSize)
		vecSamples.erase(vecSamples.begin());
	double SX = 0.0;
	double SY = 0.0;
	for (geometry_msgs::Point p : vecSamples) {
		SX += p.x;
		SY += p.y;
	}

	// calculating sample mean
	double sampleMeanX = SX / (double)vecSamples.size();
	double sampleMeanY = SY / (double)vecSamples.size();

	double varX = 0.0;
	double varY = 0.0;

	for (geometry_msgs::Point p : vecSamples) {
		SX += p.x - sampleMeanX * p.x - sampleMeanX;
		SY += p.y - sampleMeanY * p.y - sampleMeanY;
	}

	// calculating sample variance
	varX = SX / ((double)vecSamples.size() - 1.0);
	varY = SY / ((double)vecSamples.size() - 1.0);

	msg.pose.covariance[0] = varX;
	msg.pose.covariance[7] = varY;

	// calculating the covariance
	double covXY = 0.0;
	for (geometry_msgs::Point p : vecSamples)
		covXY += p.x - sampleMeanX * p.y - sampleMeanX;

	covXY = covXY / ((double)vecSamples.size() - 1.0);
	double covYX = covXY;
	msg.pose.covariance[1] = covXY;
	msg.pose.covariance[6] = covYX;

	return msg;
}

private:

ros::NodeHandle nh;

ros::Publisher chatter_pub;

image_transport::Subscriber sub;

image_transport::Subscriber subd;

image_transport::Publisher depth_pub;

image_transport::Publisher rgb_pub;
};


int main(int argc, char **argv)
{
	// initializing the ros node with the name face_detector
	ros::init(argc, argv, "face_detector");

	// creating a object of faceDetector class
	faceDetector fDObject;

	return 0;
}
