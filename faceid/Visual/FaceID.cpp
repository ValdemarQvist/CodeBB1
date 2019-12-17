#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core.hpp>
#include <iostream>
using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;

int main(int argc, const char** argv)
{
	String face_cascade_name = "data/haarcascades/haarcascade_frontalface_alt.xml";
	String eyes_cascade_name = "data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
	//-- 1. Load the cascades
	if (!face_cascade.load(face_cascade_name))
	{
		cout << "--(!)Error loading face cascade\n";
		return -1;
	};
	if (!eyes_cascade.load(eyes_cascade_name))
	{
		cout << "--(!)Error loading eyes cascade\n";
		return -1;
	};
	int camera_device = 0;
	VideoCapture capture;
	//-- 2. Read the video stream
	capture.open(camera_device);
	if (!capture.isOpened())
	{
		cout << "--(!)Error opening video capture\n";
		return -1;
	}
	Mat frame;
	while (capture.read(frame))
	{
		if (frame.empty())
		{
			cout << "--(!) No captured frame -- Break!\n";
			break;
		}
		//-- 3. Apply the classifier to the frame
		detectAndDisplay(frame);
		if (waitKey(10) == 27)
		{
			break; // escape
		}
	}
	return 0;
}
void detectAndDisplay(Mat frame)
{
	Mat frame_gray;
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);


	const int DETECTION_WIDTH = 320;
	// Possibly shrink the image, to run much faster.
	Mat smallImg;
	float scale = frame.cols / (float)DETECTION_WIDTH;
	if (frame.cols > DETECTION_WIDTH) {
		// Shrink the image while keeping the same aspect ratio.
		int scaledHeight = cvRound(frame.rows / scale);
		resize(frame_gray, smallImg, Size(DETECTION_WIDTH, scaledHeight));
	}
	else {
		// Access the input directly since it is already small.
		smallImg = frame;
	}


	equalizeHist(smallImg, smallImg);
	//-- Detect faces
	std::vector<Rect> faces;
	face_cascade.detectMultiScale(smallImg, faces);
	for (size_t i = 0; i < faces.size(); i++)
	{
		Point center((faces[i].x + faces[i].width / 2)* scale, (faces[i].y + faces[i].height / 2)* scale);
		ellipse(frame, center, Size((faces[i].width / 2) * scale, (faces[i].height / 2) * scale), 0, 0, 360, Scalar(255, 0, 255), 2);
		
		// Enlarge the results if the image was temporarily shrunk.
		if (frame.cols > smallImg.cols) {
			for (int i = 0; i < (int)faces.size(); i++) {
				faces[i].x = cvRound(faces[i].x * scale);
				faces[i].y = cvRound(faces[i].y * scale);
				faces[i].width = cvRound(faces[i].width * scale);
				faces[i].height = cvRound(faces[i].height * scale);
			}
		}

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
		
		Mat faceROI = frame_gray(faces[i]);
		//-- In each face, detect eyes
		std::vector<Rect> eyes;
		eyes_cascade.detectMultiScale(faceROI, eyes);
		for (size_t j = 0; j < eyes.size(); j++)
		{
			Point eye_center((faces[i].x + eyes[j].x + eyes[j].width / 2), (faces[i].y + eyes[j].y + eyes[j].height / 2));
			int radius = cvRound(((eyes[j].width + eyes[j].height) * 0.25));
			circle(frame, eye_center, radius, Scalar(255, 0, 0), 2);
		}
	}
	//-- Show what you got
	imshow("Capture - Face detection", frame);
	//Add Ros message with Point center
}