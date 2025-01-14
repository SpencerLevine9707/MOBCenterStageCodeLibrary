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
public class OpenCVDetectRed extends OpenCvPipeline {
    public static final Scalar green = new Scalar(0, 255, 0);
    public static final Scalar blue = new Scalar(0, 0, 255);
    public static final Scalar red = new Scalar(255, 0, 0);
    public static final Scalar pink = new Scalar(255, 0, 142);
    public static final Scalar white = new Scalar(255, 255, 255);
    public static double webcamSplitDist = 160;
    public boolean isDetected = false;
    public static double minArea = 50;
    public static double minWidth = 20;
    public static double minHeight = 50;

    public int centerX;
    public int centerY;
    public static int xDist;
    public static int yDist;
    public static double thetaX;

    public static int rectArea;

    Telemetry telemetry;
    // Make a Constructor
    public OpenCVDetectRed(Telemetry telemetry) {
        this.telemetry = telemetry;

    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat frame) {
        Mat mask = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat output = new Mat();

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV_FULL);

        Scalar lowerRed = new Scalar(0, 120, 70);
        Scalar upperRed = new Scalar(20, 255, 255);
        Core.inRange(hsv, lowerRed, upperRed, mask1);

        lowerRed = new Scalar(160, 120, 70);
        upperRed = new Scalar(180, 255, 255);
        Core.inRange(hsv, lowerRed, upperRed, mask2);
        Core.bitwise_or(mask1, mask2, mask);


        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours on the original image
        frame.copyTo(output);

        // Draw the contours on the original frame
//            Imgproc.drawContours(hsv, contours, -1, white, 10);
//            telemetry.addData("Contours found: ", contours.size());
//            telemetry.update();

        // Find the contour with the largest area
        double maxArea = 0;
        MatOfPoint maxContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect boundingBox = Imgproc.boundingRect(contour);
            int w = boundingBox.width;
            int h = boundingBox.height;
            if (area > maxArea && area >= minArea && w >= minWidth && h>= minHeight) {
                maxArea = area;
                maxContour = contour;
            }
        }

        // Draw a bounding box around the object with the largest area
        if (maxContour != null) {
            isDetected = true;
//            telemetry.addLine("Contours found!!! :)");
            Rect boundingBox = Imgproc.boundingRect(maxContour);
            int x = boundingBox.x;
            int y = boundingBox.y;
            int w = boundingBox.width;
            int h = boundingBox.height;

            rectArea = w*h;

            Imgproc.rectangle(frame, new Point(x, y), new Point(x + w, y + h), green, 2);

            // Display the coordinates of the center of the bounding box
            int center_x = x + w/2;
            int center_y = y + h/2;

            centerX = center_x;
            centerY = center_y;
            xDist = center_x - (frame.width()/2);
            yDist = center_y - (frame.height()/2);
            thetaX = (double)((Math.abs(xDist)-160)/4)+30;
            if(xDist > 0){
                thetaX *= -1;
            }

//            int circRad;
//            if(Math.abs(x-(x+w)) > Math.abs(y-(y+h))){
//                circRad = Math.abs(x-(x+w))/2;
//            }
//            else{
//                circRad = Math.abs(y-(y+h))/2;
//            }

//                Imgproc.circle(hsv, new Point(center_x, center_y), circRad, red, 2);
//                Imgproc.circle(hsv, new Point(center_x, center_y), 3, red, -1);

            Imgproc.line(frame, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), blue, 1);
            Imgproc.line(frame, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), blue, 1);

//                Imgproc.putText(hsv, "(" + center_x + ", " + center_y + ")", new Point(frame.width(), frame.height() - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
            // System.out.println("Center coordinates: (" + center_x + ", " + center_y + ")");
        }
        else {
            isDetected = false;
//            telemetry.addLine("None found :(");
        }
//            telemetry.update();

            return hsv;
//        return frame;
    }
}