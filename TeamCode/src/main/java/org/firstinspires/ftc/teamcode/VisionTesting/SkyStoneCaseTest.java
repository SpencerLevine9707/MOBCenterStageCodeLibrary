package org.firstinspires.ftc.teamcode.VisionTesting;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
public class SkyStoneCaseTest extends OpenCvPipeline {
    Telemetry telemetry;

    public static double minArea = 150;
    public static double maxArea = 5000;
    public static ArrayList<String> case0List = new ArrayList<>();
    public static ArrayList<String> case2List = new ArrayList<>();

    public int caseForStone;

    // Make a Constructor
    public SkyStoneCaseTest(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat frame) {
        ArrayList<StoneColorAndPoint> stoneColorOrder = new ArrayList<>();
        telemetry.addLine("running");


        // Converts the RGB colors from the video to HSV, which is more useful for image analysis
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV_FULL);

        Mat maskYellow = new Mat();
        Core.inRange(hsv, new Scalar(OpenCVGreatestColorTest.lowerYellow[0], OpenCVGreatestColorTest.lowerYellow[1], OpenCVGreatestColorTest.lowerYellow[2]), new Scalar(OpenCVGreatestColorTest.upperYellow[0], OpenCVGreatestColorTest.upperYellow[1], OpenCVGreatestColorTest.upperYellow[2]), maskYellow);

        // Find the contours of the objects in the frame that are within the mask
        ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
        Mat hierarchyYellow = new Mat();
        Imgproc.findContours(maskYellow, yellowContours, hierarchyYellow, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw the contours on the original frame
//            Imgproc.drawContours(hsv, contours, -1, white, 10);
        telemetry.addData("Yellow Contours found: ", yellowContours.size());

        Mat maskBlack = new Mat();
        Core.inRange(hsv, new Scalar(OpenCVGreatestColorTest.lowerBrown[0], OpenCVGreatestColorTest.lowerBrown[1], OpenCVGreatestColorTest.lowerBrown[2]), new Scalar(OpenCVGreatestColorTest.upperBrown[0], OpenCVGreatestColorTest.upperBrown[1], OpenCVGreatestColorTest.upperBrown[2]), maskBlack);

        // Find the contours of the objects in the frame that are within the mask
        ArrayList<MatOfPoint> blackContours = new ArrayList<>();
        Mat hierarchyBlack = new Mat();
        Imgproc.findContours(maskBlack, blackContours, hierarchyBlack, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw the contours on the original frame
//            Imgproc.drawContours(hsv, contours, -1, white, 10);
        telemetry.addData("Black Contours found: ", blackContours.size());

        // Find the contour with the largest area
        ArrayList<MatOfPoint> goodYellowContours = new ArrayList<>();
        for (MatOfPoint contour : yellowContours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea && area < maxArea) {
                goodYellowContours.add(contour);
            }
        }
        ArrayList<MatOfPoint> goodBlackContours = new ArrayList<>();
        for (MatOfPoint contour : blackContours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea && area < maxArea) {
                goodBlackContours.add(contour);
            }
        }

        // Draw a bounding box around the object with the largest area
        if (goodYellowContours != null) {
            telemetry.addLine("Contours found!!! :)");
            for (int i = 0; i < goodYellowContours.size(); i++) {
                Rect boundingBox = Imgproc.boundingRect(goodYellowContours.get(i));
                int x = boundingBox.x;
                int y = boundingBox.y;
                int w = boundingBox.width;
                int h = boundingBox.height;

                Imgproc.rectangle(hsv, new Point(x, y), new Point(x + w, y + h), OpenCVGreatestColorTest.green, 2);

                // Display the coordinates of the center of the bounding box
                int center_x = x + w / 2;
                int center_y = y + h / 2;

                int circRad;
                if (Math.abs(x - (x + w)) > Math.abs(y - (y + h))) {
                    circRad = Math.abs(x - (x + w)) / 2;
                } else {
                    circRad = Math.abs(y - (y + h)) / 2;
                }

                Imgproc.circle(hsv, new Point(center_x, center_y), circRad, OpenCVGreatestColorTest.red, 2);
                Imgproc.circle(hsv, new Point(center_x, center_y), 3, OpenCVGreatestColorTest.red, -1);

                Imgproc.line(hsv, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), OpenCVGreatestColorTest.blue, 1);
                Imgproc.line(hsv, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), OpenCVGreatestColorTest.blue, 1);

                stoneColorOrder.add(new StoneColorAndPoint(new Point(center_x, center_y), "yellow"));

                telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
            }
        }
        else{
            telemetry.addLine("None Yellow Found :(");
        }
        if (goodBlackContours != null) {
            telemetry.addLine("Contours found!!! :)");
            for (int i = 0; i < goodBlackContours.size(); i++) {
                Rect boundingBox = Imgproc.boundingRect(goodBlackContours.get(i));
                int x = boundingBox.x;
                int y = boundingBox.y;
                int w = boundingBox.width;
                int h = boundingBox.height;

                Imgproc.rectangle(hsv, new Point(x, y), new Point(x + w, y + h), OpenCVGreatestColorTest.green, 2);

                // Display the coordinates of the center of the bounding box
                int center_x = x + w / 2;
                int center_y = y + h / 2;

                int circRad;
                if (Math.abs(x - (x + w)) > Math.abs(y - (y + h))) {
                    circRad = Math.abs(x - (x + w)) / 2;
                } else {
                    circRad = Math.abs(y - (y + h)) / 2;
                }

                Imgproc.circle(hsv, new Point(center_x, center_y), circRad, OpenCVGreatestColorTest.red, 2);
                Imgproc.circle(hsv, new Point(center_x, center_y), 3, OpenCVGreatestColorTest.red, -1);

                Imgproc.line(hsv, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), OpenCVGreatestColorTest.blue, 1);
                Imgproc.line(hsv, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), OpenCVGreatestColorTest.blue, 1);

                stoneColorOrder.add(new StoneColorAndPoint(new Point(center_x, center_y), "black"));

                telemetry.addData("Center X: ", center_x + " Center Y: " + center_y);
            }
        }
        else{
            telemetry.addLine("None Black Found :(");
        }
        telemetry.addData("Points list (unsorted) = ", stoneColorOrder);

        for(int i = 0; i < stoneColorOrder.size(); i++){
            int n = stoneColorOrder.size();
            for(int j = 0; j < n - i - 1; j++){
                if(stoneColorOrder.get(j).stonePoint.x > stoneColorOrder.get(j+1).stonePoint.x){
                    StoneColorAndPoint temp = stoneColorOrder.get(j);
                    stoneColorOrder.set(j, stoneColorOrder.get(j+1));
                    stoneColorOrder.set(j+1, temp);
                }
            }
        }
        ArrayList<String> colors = new ArrayList<>();
        for(StoneColorAndPoint c: stoneColorOrder){
            colors.add(c.getStoneColor());
        }
        telemetry.addLine("Points list but SORTED = " + stoneColorOrder);
        telemetry.addLine("Colors: " + colors);
        if(colors.size()==5){
            caseForStone = 1;
        }
        else if(colors.size() == 4){
            if(colors.get(0).equals("yellow")){
                caseForStone = 2;
            }
            else if(colors.get(0).equals("black")){
                caseForStone = 0;
            }
        }
        else{
            caseForStone = -1;
        }
        telemetry.addLine("Case for Stone is " + caseForStone);


        telemetry.update();

        return hsv;
    }
}


