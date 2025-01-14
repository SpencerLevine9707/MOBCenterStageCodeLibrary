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
public class DetectgTeamPropTRest extends OpenCvPipeline {
    public static final Scalar green = new Scalar(0, 255, 0);
    public static final Scalar blue = new Scalar(0, 0, 255);
    public static final Scalar red = new Scalar(255, 0, 0);
    public static final Scalar pink = new Scalar(255, 0, 142);
    public static final Scalar white = new Scalar(255, 255, 255);
    public static double webcamSplitDist = 160;
    public static boolean isDetected = false;
    public static double minArea = 50;
    public static double minWidth = 20;
    public static double minHeight = 50;

    public static int[] lowerBlue = {100, 100, 100};
    public static int[] upperBlue = {175, 255, 255};

    public static int[] lowerRed = {170, 100, 100};
    public static int[] upperRed = {180, 255, 255};

    public static int[] lowerYellow = {20, 100, 100};
    public static int[] upperYellow = {50, 255, 255};

    public static int[] lowerWhite = {0, 0, 200};
    public static int[] upperWhite = {180, 30, 255};

    public static int[] lowerGray = {0, 0, 30};
    public static int[] upperGray = {180, 30, 200};


    public static int[] lowerBlack = {0, 0, 0};
    public static int[] upperBlack = {180, 255, 50};

    public static int[] lowerBrown = {0, 30, 20};
    public static int[] upperBrown = {40, 150, 150};


    public int[] lowerColor = OpenCVGreatestColorTest.lowerRed;
    public int[] upperColor = OpenCVGreatestColorTest.upperRed;

    public static int centerX;
    public static int centerY;
    public static int xDist;
    public static int yDist;
    public static double thetaX;

    public static int rectArea;

    Telemetry telemetry;
    // Make a Constructor
    public DetectgTeamPropTRest(Telemetry telemetry) {
        this.telemetry = telemetry;

    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat frame) {
//            telemetry.addLine("running");

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
                telemetry.addLine("Contours found!!! :)");
            Rect boundingBox = Imgproc.boundingRect(maxContour);
            int x = boundingBox.x;
            int y = boundingBox.y;
            int w = boundingBox.width;
            int h = boundingBox.height;

            rectArea = w*h;

            Imgproc.rectangle(hsv, new Point(x, y), new Point(x + w, y + h), green, 2);

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

            int circRad;
            if(Math.abs(x-(x+w)) > Math.abs(y-(y+h))){
                circRad = Math.abs(x-(x+w))/2;
            }
            else{
                circRad = Math.abs(y-(y+h))/2;
            }

                Imgproc.circle(hsv, new Point(center_x, center_y), circRad, red, 2);
                Imgproc.circle(hsv, new Point(center_x, center_y), 3, red, -1);

            Imgproc.line(hsv, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), blue, 1);
            Imgproc.line(hsv, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), blue, 1);

//                Imgproc.putText(hsv, "(" + center_x + ", " + center_y + ")", new Point(frame.width(), frame.height() - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
            // System.out.println("Center coordinates: (" + center_x + ", " + center_y + ")");
        }
        else{
            isDetected = false;
//                telemetry.addLine("None found :(");
        }
        int zoneDetected = 0;
        if(OpenCVDetectTeamProp.centerX < 160){
            zoneDetected = 2;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            zoneDetected = 3;
        }
        else if(!OpenCVDetectTeamProp.isDetected){
            zoneDetected = 1;
        }
        telemetry.addLine("zoneDetected: " + zoneDetected);
        telemetry.update();

        return hsv;
    }
}