package org.firstinspires.ftc.teamcode.VisionTestingForSim;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {

    @Override
    public void init(Mat input) {
        // Executed before the first call to processFrame
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        Mat mat = new Mat();
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define the HSV range for blue color
        Scalar lowerBlue = new Scalar(100, 125, 100); // HSV lower threshold for blue
        Scalar upperBlue = new Scalar(120, 255, 255); // HSV high threshold for blue

        // Threshold the HSV image to get only blue colors
        Core.inRange(hsvMat, lowerBlue, upperBlue, mat);

        // Return the processed image
        return mat;
//        Mat mat = new Mat();
//        Mat hsvMat = new Mat();
//        Mat mask1 = new Mat();
//        Mat mask2 = new Mat();
//        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
//
//        // Define the HSV range for red color (low range)
//        Scalar lowerRed1 = new Scalar(0, 150, 40); // HSV lower threshold for red (low range)
//        Scalar upperRed1 = new Scalar(20, 255, 255); // HSV high threshold for red (low range)
//
//        // Define the HSV range for red color (high range)
//        Scalar lowerRed2 = new Scalar(150, 150, 40); // HSV lower threshold for red (high range)
//        Scalar upperRed2 = new Scalar(180, 255, 255); // HSV high threshold for red (high range)
//
//        // Threshold the HSV image to get only red colors (low range)
//        Core.inRange(hsvMat, lowerRed1, upperRed1, mask1);
//
//        // Threshold the HSV image to get only red colors (high range)
//        Core.inRange(hsvMat, lowerRed2, upperRed2, mask2);
//
//        // Combine the masks to capture all shades of red
//        Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mat);
//
//        // Return the processed image
//        return mat;

        // Optionally, apply additional processing like erode/dilate here


//        return input; // Return the image that will be displayed in the viewport
        // (In this case the input mat directly)
    }
}