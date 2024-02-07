package org.firstinspires.ftc.teamcode.VisionTestingForSim;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


class RGBBlueDetection extends OpenCvPipeline {
    Telemetry telemetry;

//    public RGBBlueDetection(Telemetry telemetry) {
//        this.telemetry = telemetry;
//
//    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat processedImage = new Mat();
        // Define the range of blue colors in RGB
        Scalar lowerBlue = new Scalar(100, 0, 0); // RGB low threshold for blue
        Scalar upperBlue = new Scalar(255, 120, 100); // RGB high threshold for blue

        // Threshold the input image to get only blue colors
        Core.inRange(input, lowerBlue, upperBlue, processedImage);

        // Optionally, apply additional processing like erode/dilate here

        return processedImage; // Return the processed image
    }
}
