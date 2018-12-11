#include <iostream>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not successful, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

    //Robot 1 Color Mask (also used for testing)
    //This is the orange rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH1 = 0;
    int iHighH1 = 43;

    int iLowS1 = 45; 
    int iHighS1 = 114;

    int iLowV1 = 185;
    int iHighV1 = 255;
	
	//Robot 2 Color Mask
    //This is the red rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH2 = 154;
    int iHighH2 = 179;

    int iLowS2 = 58; 
    int iHighS2 = 255;

    int iLowV2 = 60;
    int iHighV2 = 255;
	
	//Robot 3 Color Mask
    //This is the green in the ME507.docx file on drive. Printed on the 192-118 printer.
    int iLowH3 = 30;
    int iHighH3 = 84;

    int iLowS3 = 35; 
    int iHighS3 = 114;

    int iLowV3 = 78;
    int iHighV3 = 255;
	

    //Create trackbars in "Control" window for Robot 1 (this is used for calibration)
    createTrackbar("LowH", "Control", &iLowH1, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH1, 179);

    createTrackbar("LowS", "Control", &iLowS1, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS1, 255);

    createTrackbar("LowV", "Control", &iLowV1, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV1, 255);

    //last location of robot1
	int iLastX1 = -1; 
    int iLastY1 = -1;
	//last location of robot2
	int iLastX2 = -1;
	int iLastY2 = -1;
	//last location of robot3
	int iLastX3 = -1;
	int iLastY3 = -1;
	
    //Capture a temporary image from the camera (used to scale black image to correct size)
    Mat imgTmp;
    cap.read(imgTmp); 

    //Create a black image with the size as the camera output
    Mat imgLines1 = Mat::zeros( imgTmp.size(), CV_8UC3 );; //Robot1
	Mat imgLines2 = imgLines1; //Robot2
	Mat imgLines3 = imgLines1; //Robot3

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		//Code Below is specific to each robot location.
		
		
		Mat imgThresholded;
		// Robot 1 Image Processing
		inRange(imgHSV, Scalar(iLowH1, iLowS1, iLowV1), Scalar(iHighH1, iHighS1, iHighV1), imgThresholded); //Threshold the image
		  
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;        
			
				if (iLastX1 >= 0 && iLastY1 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines1, Point(posX, posY), Point(iLastX1, iLastY1), Scalar(255,0,0), 2); //Scalar(B,G,R) for line color
				}

			iLastX1 = posX;
			iLastY1 = posY;
		}

		imshow("Thresholded Image - Robot 1", imgThresholded); //show the thresholded image
		
		//tracking line on image
		Mat imgOriginalDisp;
		imgOriginalDisp = imgOriginal + imgLines1;
		imshow("Original - R1", imgOriginalDisp); //show the original image with tracking line added
		
		
		//Robot 2 Image Processing
		
		
		inRange(imgHSV, Scalar(iLowH2, iLowS2, iLowV2), Scalar(iHighH2, iHighS2, iHighV2), imgThresholded); //Threshold the image
		  
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;        
			
				if (iLastX2 >= 0 && iLastY2 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines2, Point(posX, posY), Point(iLastX2, iLastY2), Scalar(255,0,0), 1);
				}

			iLastX2 = posX;
			iLastY2 = posY;
		}

		imshow("Thresholded Image - Robot 2", imgThresholded); //show the thresholded image
		
		//tracking line on image
		//Mat imgOriginalDisp; //only need this declaration in robot1 since it's reused
		imgOriginalDisp = imgOriginal + imgLines2;
		imshow("Original - R2", imgOriginalDisp); //show the original image with tracking line added
		
		
		//Robot 3 Image Processing
		
		
		inRange(imgHSV, Scalar(iLowH3, iLowS3, iLowV3), Scalar(iHighH3, iHighS3, iHighV3), imgThresholded); //Threshold the image
		  
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000)
		{
			//calculate the position of the ball (note these posX & posY need to be declared everytime since they're cleared each time if loop runs.
			int posX = dM10 / dArea;
			int posY = dM01 / dArea;
			
				if (iLastX3 >= 0 && iLastY3 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines3, Point(posX, posY), Point(iLastX3, iLastY3), Scalar(255,0,0), 2);
				}

			iLastX3 = posX;
			iLastY3 = posY;
		}

		imshow("Thresholded Image - Robot 3", imgThresholded); //show the thresholded image
		
		//tracking line on image
		//Mat imgOriginalDisp; //only need this declaration in robot1 since it's reused
		imgOriginalDisp = imgOriginal + imgLines3;
		imshow("Original - R3", imgOriginalDisp); //show the original image with tracking line added

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break; 
		}
    }

   return 0;
}
