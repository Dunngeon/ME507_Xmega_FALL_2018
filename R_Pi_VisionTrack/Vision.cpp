//**************************************************************************************
/** \file Vision.cpp
 *    This file contains source code for vision tracking squares in 2D space, that is resolved into robot positions.
 *
 *  Revisions:
 *    \li 11-25-18 RGD - intiial creation
 *    \li 11-28-18 RGD - added support for multiple colored squares
 *    \li 12-08-18 RGD - added support for finding heading of the robot.
 *
 *  License:
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *    GNU General Public License for more details.
*/
//**************************************************************************************

#include <iostream>
#include <math.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    ///This is the effective State0 of the vision system
    ///Testing to ensure frames can be read from camera.
    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not successful, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    bool _ControlDebug = false; // set to true to display control window
    bool _ThreshedDebug = false; //set to true to display Threshed windows

    if(_ControlDebug == true)
    {
        namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
    }
    ///robot squares color masks. A is the front, B is the rear of each robot. All square colors can be found in the ME507Squares doc on git.
    //Robot 1-R Color Mask
    //This is the red rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH1A = 154;
    int iHighH1A = 179;

    int iLowS1A = 109;
    int iHighS1A = 255;

    int iLowV1A = 60;
    int iHighV1A = 255;

	//Robot 1-G Color Mask
    //This is the green rectangle in the ME507.docx file on drive. Printed on the 192-118 printer.
    int iLowH1B = 30;
    int iHighH1B = 84;

    int iLowS1B = 49;
    int iHighS1B = 116;

    int iLowV1B = 159;
    int iHighV1B = 255;


    //Robot 2-O Color Mask (also used for testing)
    //This is the orange rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH2A = 0;
    int iHighH2A = 9;

    int iLowS2A = 79;
    int iHighS2A = 178;

    int iLowV2A = 201;
    int iHighV2A = 255;

    //Robot 2-B Color Mask (also used for testing)
    //This is the blue rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH2B = 101;
    int iHighH2B = 117;

    int iLowS2B = 102;
    int iHighS2B = 225;

    int iLowV2B = 168;
    int iHighV2B = 255;


    //Robot 3A Color Mask
    //This is the YELLOW rectangle that is present in the ME507.docx on Ryan's drive. Printed on 192-118 printer.
    int iLowH3A = 14;
    int iHighH3A = 28;

    int iLowS3A = 79;
    int iHighS3A = 255;

    int iLowV3A = 168;
    int iHighV3A = 255;

	//Robot 3B Color Mask
    //This is the Purple rectangle in the ME507.docx file on drive. Printed on the 192-118 printer.
    int iLowH3B = 128;
    int iHighH3B = 154;

    int iLowS3B = 102;
    int iHighS3B = 225;

    int iLowV3B = 127;
    int iHighV3B = 255;

    ///create trackbars so that the color masks can be tuned
    if(_ControlDebug==true)
    {
        //Create trackbars in "Control" window for Robot 1A (this is used for calibration)
        createTrackbar("LowH", "Control", &iLowH1A, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH1A, 179);

        createTrackbar("LowS", "Control", &iLowS1A, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS1A, 255);

        createTrackbar("LowV", "Control", &iLowV1A, 255);//Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV1A, 255);
    }


	double iLastX[6]; //data array to store the locations of the squares FORMAT: [1AX, 1Bx,...]
	double iLastY[6];
    //Capture a temporary image from the camera (used to scale black image to correct size)
    /*
    Mat imgTmp;
    cap.read(imgTmp);
    */
    //Create a black image with the size as the camera output
    /*
    Mat imgLines1 = Mat::zeros( imgTmp.size(), CV_8UC3 );; //Robot1
	Mat imgLines2 = imgLines1; //Robot2
	Mat imgLines3 = imgLines1; //Robot3
    */


    while (true)
    {
        ///This is the effective state 1 of the vision system, it loops forever.

        ///Capture image from camera.
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
        //doxygen documentation below.
        /** The way the vision system works is as follows
        1. imgOriginal is converted to HSV color format
        2. imgHSV is thresholded to create a black/white mask
        3. Moments are taken about the white color on the mask (centroids and second moments of area)
        4. the center of the masked shape can be determined using the moments
        5. This is repeated for all 6 squares
        6. Some basic trignometry is applied to compute the angle of each robot, along with actual center position of the robot.
        */
		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		//Code Below is specific to each square location.

		//square 1A location
		Mat imgThresholded;
		// Robot 1 Image Processing
		inRange(imgHSV, Scalar(iLowH1A, iLowS1A, iLowV1A), Scalar(iHighH1A, iHighS1A, iHighV1A), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
        Point cntr1A;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
			cntr1A = Point(posX, posY);
                /*
                if (iLastX1 >= 0 && iLastY1 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines1, Point(posX, posY), Point(iLastX1, iLastY1), Scalar(255,0,0), 2); //Scalar(B,G,R) for line color
				}
                */
			iLastX[0] = posX;
			iLastY[0] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square1A", imgThresholded); //show the thresholded image
        }
        //square 1B img processing


		inRange(imgHSV, Scalar(iLowH1B, iLowS1B, iLowV1B), Scalar(iHighH1B, iHighS1B, iHighV1B), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;
        Point cntr1B;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
			cntr1B = Point(posX, posY);
                /*
                if (iLastX1 >= 0 && iLastY1 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines1, Point(posX, posY), Point(iLastX1, iLastY1), Scalar(255,0,0), 2); //Scalar(B,G,R) for line color
				}
                */
			iLastX[1] = posX;
			iLastY[1] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square1B", imgThresholded); //show the thresholded image
        }

		//Square 2A Image Processing

		inRange(imgHSV, Scalar(iLowH2A, iLowS2A, iLowV2A), Scalar(iHighH2A, iHighS2A, iHighV2A), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;
        Point cntr2A;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
			cntr2A = Point(posX, posY);
				/*
                if (iLastX2 >= 0 && iLastY2 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines2, Point(posX, posY), Point(iLastX2, iLastY2), Scalar(255,0,0), 1);
				}
                */
			iLastX[2] = posX;
			iLastY[2] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square2A", imgThresholded); //show the thresholded image
        }
        //Square 2B Image Processing

		inRange(imgHSV, Scalar(iLowH2B, iLowS2B, iLowV2B), Scalar(iHighH2B, iHighS2B, iHighV2B), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;
        Point cntr2B;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
			cntr2B = Point(posX, posY);
				/*
                if (iLastX2 >= 0 && iLastY2 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines2, Point(posX, posY), Point(iLastX2, iLastY2), Scalar(255,0,0), 1);
				}
                */
			iLastX[3] = posX;
			iLastY[3] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square2B", imgThresholded); //show the thresholded image
        }
		//Square 3A Image Processing


		inRange(imgHSV, Scalar(iLowH3A, iLowS3A, iLowV3A), Scalar(iHighH3A, iHighS3A, iHighV3A), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;
        Point cntr3A;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball (note these posX & posY need to be declared everytime since they're cleared each time if loop runs.
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
            cntr3A = Point(posX, posY);

				/*
                if (iLastX3 >= 0 && iLastY3 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines3, Point(posX, posY), Point(iLastX3, iLastY3), Scalar(255,0,0), 2);
				}
                */
			iLastX[4] = posX;
			iLastY[4] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square 3A", imgThresholded); //show the thresholded image
        }
		//tracking line on image
		/*
        //Mat imgOriginalDisp; //only need this declaration in robot1 since it's reused
		imgOriginalDisp = imgOriginal + imgLines3;
		imshow("Original - R3", imgOriginalDisp); //show the original image with tracking line added
        */

        //Square 3B Image Processing


		inRange(imgHSV, Scalar(iLowH3B, iLowS3B, iLowV3B), Scalar(iHighH3B, iHighS3B, iHighV3B), imgThresholded); //Threshold the image
        /*
		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        */
		//Calculate the moments of the thresholded image
		oMoments = moments(imgThresholded);

		dM01 = oMoments.m01;
		dM10 = oMoments.m10;
		dArea = oMoments.m00;
        Point cntr3B;
		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
		if (dArea > 10000)
		{
			//calculate the position of the ball (note these posX & posY need to be declared everytime since they're cleared each time if loop runs.
			double posX = dM10 / dArea;
			double posY = dM01 / dArea;
            cntr3B = Point(posX, posY);

				/*
                if (iLastX3 >= 0 && iLastY3 >= 0 && posX >= 0 && posY >= 0)
				{
					//Draw a red line from the previous point to the current point
					line(imgLines3, Point(posX, posY), Point(iLastX3, iLastY3), Scalar(255,0,0), 2);
				}
                */
			iLastX[5] = posX;
			iLastY[5] = posY;
		}
        if(_ThreshedDebug==true)
        {
            imshow("Thresholded Image - Square3B", imgThresholded); //show the thresholded image
        }

		///Calculate actual robot center position from two data points
        double robotpositionX[3];
        double robotpositionY[3];
        double robotangle[3]; //data array to store the robot angle. Note that color a A is front, and B is back of robot.
        Point robotcenters[3]; //data array for plotting the points.
        for(uint8_t i=0;i<6;i++)
        {
            robotpositionX[i/2] = (iLastX[i+1] - iLastX[i])/2.0 + iLastX[i];
            robotpositionY[i/2] = (iLastY[i+1] - iLastY[i])/2.0 + iLastY[i];
            robotcenters[i/2] = Point(int(robotpositionX[i/2]),int(robotpositionY[i/2])); //compact data into Point structure for plotting robot centre.
            //calculate some angles yo!
            if((iLastX[i+1] - iLastX[i]) < 0.1 && (iLastX[i+1] - iLastX[i]) > -0.1)
            {
                robotangle[i/2] = 90; // TODO: deal with 270deg edge case.
            }
            else if((iLastY[i+1]-iLastY[i]) < 0.1 && (iLastY[i+1]-iLastY[i])> -0.1)
            {
                robotangle[i/2] = 0; //TODO: deal with 180deg edge case.
            }
            else
                robotangle[i/2] = atan2((iLastY[i+1]-iLastY[i]),(iLastX[i+1] - iLastX[i])) * 180.0 /3.14159; //calculate robot angle in radians

            i++;

        }

		//tracking line on image
		/*
        //Mat imgOriginalDisp; //only need this declaration in robot1 since it's reused
		imgOriginalDisp = imgOriginal + imgLines3;
		imshow("Original - R3", imgOriginalDisp); //show the original image with tracking line added
        */
        ///Image tracking display
        ///show circles that track squares
        circle(imgOriginal, cntr1A, 3, Scalar(255, 0, 255), 2);
        circle(imgOriginal, cntr1B, 3, Scalar(255, 0, 255), 2);
        circle(imgOriginal, cntr2A, 3, Scalar(255, 0, 255), 2);
        circle(imgOriginal, cntr2B, 3, Scalar(255, 0, 255), 2);
        circle(imgOriginal, cntr3A, 3, Scalar(255, 0, 255), 2);
        circle(imgOriginal, cntr3B, 3, Scalar(255, 0, 255), 2);

        ///show circles that track robot position
        circle(imgOriginal, robotcenters[0], 3, Scalar(255, 255, 255), 4);
        circle(imgOriginal, robotcenters[1], 3, Scalar(255, 255, 255), 4);
        circle(imgOriginal, robotcenters[2], 3, Scalar(255, 255, 255), 4);

        imshow("With centers" , imgOriginal);

        ///Print robot state to serial
        cout << "Position of Robot 1: " << robotpositionX[0] << "," << robotpositionY[0] << endl << "Position of Robot 2: " << robotpositionX[1] << "," << robotpositionY[1] << endl << "Position of Robot 3: " << robotpositionX[2] << "," << robotpositionY[2] << endl;
        cout << "Angle of Robot 1: " << robotangle[0] << endl << "Angle of RObot 2: " << robotangle[1] << endl << "Angle of Robot 3: " << robotangle[2] << endl;
        ///Program can be ended if esc is pressed by user.
        if (waitKey(30) == 27) //+wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
    }

   return 0;
}
