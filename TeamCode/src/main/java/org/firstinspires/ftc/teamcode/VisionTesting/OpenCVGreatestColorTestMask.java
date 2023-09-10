package org.firstinspires.ftc.teamcode.VisionTesting;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
public class OpenCVGreatestColorTestMask extends OpenCvPipeline {

    Telemetry telemetry;

//    int[] lowerColor = OpenCVGreatestColorTest.lowerBrown;
//    int[] upperColor = OpenCVGreatestColorTest.upperBrown;
    int[] lowerColor = OpenCVGreatestColorTest.lowerBlack;
    int[] upperColor = OpenCVGreatestColorTest.upperBlack;

    // Make a Constructor
    public OpenCVGreatestColorTestMask(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat frame) {
        telemetry.addLine("running");


        // Converts the RGB colors from the video to HSV, which is more useful for image analysis
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV_FULL);

        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(lowerColor[0], lowerColor[1], lowerColor[2]), new Scalar(upperColor[0], upperColor[1], upperColor[2]), mask);

        // Find the contours of the objects in the frame that are within the mask
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw the contours on the original frame
//            Imgproc.drawContours(hsv, contours, -1, white, 10);
        telemetry.addData("Contours found: ", contours.size());
        telemetry.update();

        // Find the contour with the largest area
        double maxArea = 0;
        MatOfPoint maxContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                maxContour = contour;
            }
        }

        // Draw a bounding box around the object with the largest area
        if (maxContour != null) {
            telemetry.addLine("Contours found!!! :)");
            Rect boundingBox = Imgproc.boundingRect(maxContour);
            int x = boundingBox.x;
            int y = boundingBox.y;
            int w = boundingBox.width;
            int h = boundingBox.height;

            Imgproc.rectangle(hsv, new Point(x, y), new Point(x + w, y + h), OpenCVGreatestColorTest.green, 2);

            // Display the coordinates of the center of the bounding box
            int center_x = x + w/2;
            int center_y = y + h/2;

            int circRad;
            if(Math.abs(x-(x+w)) > Math.abs(y-(y+h))){
                circRad = Math.abs(x-(x+w))/2;
            }
            else{
                circRad = Math.abs(y-(y+h))/2;
            }

            Imgproc.circle(hsv, new Point(center_x, center_y), circRad, OpenCVGreatestColorTest.red, 2);
            Imgproc.circle(hsv, new Point(center_x, center_y), 3, OpenCVGreatestColorTest.red, -1);

            Imgproc.line(hsv, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), OpenCVGreatestColorTest.blue, 1);
            Imgproc.line(hsv, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), OpenCVGreatestColorTest.blue, 1);

            Imgproc.putText(hsv, "(" + center_x + ", " + center_y + ")", new Point(frame.width(), frame.height() - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
            // System.out.println("Center coordinates: (" + center_x + ", " + center_y + ")");
        }
        else{
            telemetry.addLine("None found :(");
        }
        telemetry.update();

        return mask;
//        return hsv;
    }
}


