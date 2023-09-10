package org.firstinspires.ftc.teamcode.VisionTesting;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SamplePipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public SamplePipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public static double anchorPointX1 = 119;
    public static double anchorPointY1 = 45;

    public static int boxWidth = 50;

    public static int boxHeight = 60;

    static final Scalar pissOrange = new Scalar(0, 255, 0);

    Mat HSV = new Mat();

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(anchorPointX1, anchorPointY1);
    static final int REGION1_WIDTH = boxWidth;
    static final int REGION1_HEIGHT = boxHeight;


    // Create the points that will be used to make the rectangles for the region

    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);



    // Creates a field of type "Mat"
    Mat region1;

    // Creating an array for each region which have an element for each channel of interest
    public int[] HSV_Value_2 = new int[3];

    @Override
    public void init(Mat input) {

        // Executed before the first call to processFrame
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);

        // Creates the regions and finds the HSV values for each of the regions
        region1 = HSV.submat(new Rect(region1_pointA, region1_pointB));

        // Loops through each channel of interest
        for (int i = 0; i < 3; i++) {
            // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
            HSV_Value_2[i] = (int) Core.mean(region1).val[i];
        }

        // Draws rectangles representing the regions in the camera stream
        Imgproc.rectangle(HSV, region1_pointA, region1_pointB, pissOrange, 1);
//        Imgproc.rectangle(HSV, new Point(50, 50), new Point(100, 100), pissOrange, 1);

//        telemetry.addLine("Abc");
        telemetry.addData("Region 1", "%7d, %7d, %7d", HSV_Value_2[0], HSV_Value_2[1], HSV_Value_2[2]);
        //telemetry.update();

        if ((HSV_Value_2[0] < 88)) {

            telemetry.addLine("Yellow");

        } else if (HSV_Value_2[0] < 183) {


            telemetry.addLine("Cyan");


        } else {
            telemetry.addLine("Magenta");
        }

        telemetry.addLine("\nWaiting for start");
        telemetry.update();

        return HSV;
        // Executed every time a new frame is dispatched

//        return input; // Return the image that will be displayed in the viewport
        // (In this case the input mat directly)
    }

    @Override
    public void onViewportTapped() {
        //Telemetry readings for the HSV values in each region
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

}
