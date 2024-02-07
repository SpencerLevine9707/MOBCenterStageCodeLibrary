package org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


//@Config
@Autonomous(group="Spencer aka mid coder")
@Disabled

public class OpenCVLookForPole extends LinearOpMode {

    //    public static double anchorPointX1 = 100;
//    public static double anchorPointY1 = 50;
    public static double anchorPointX1 = 119;
    public static double anchorPointY1 = 45;

    public static int boxWidth = 10;

    public static int boxHeight = 10;

    public static int cameraWidth = 320;

    public static int cameraHeight = 240;

    //Define Gyr

    // private BNO055IMU imu;


    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCV_Pipeline pipeline;

    @Override
    public void runOpMode() {



// Send telemetry message to indicate successful Encoder reset


        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new OpenCV_Pipeline();
        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (opModeInInit()) {
            //Telemetry readings for the HSV values in each region
            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);

            //telemetry.update();

            if ((pipeline.HSV_Value_2[0] < 88)) {

                telemetry.addLine("Yellow");

            } else if (pipeline.HSV_Value_2[0] < 183) {


                telemetry.addLine("Cyan");


            } else {
                telemetry.addLine("Magenta");
            }

            telemetry.addLine("\nWaiting for start");
            telemetry.update();
        }


//         Telemetry readings for the HSV values in each region
//            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
//            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
//            telemetry.addData("Region 3", "%7d, %7d, %7d", pipeline.HSV_Value_3[0], pipeline.HSV_Value_3[1], pipeline.HSV_Value_3[2]);
//            //telemetry.update();
//            if ((pipeline.HSV_Value_1[0] < pipeline.HSV_Value_2[0]) && (pipeline.HSV_Value_1[0] < pipeline.HSV_Value_3[0])){
//                telemetry.addLine("Left");
//            }
//            if ((pipeline.HSV_Value_2[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_2[0] < pipeline.HSV_Value_3[0])){
//                telemetry.addLine("Mid");
//            }
//            if ((pipeline.HSV_Value_3[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_3[0] < pipeline.HSV_Value_2[0])){
//                telemetry.addLine("Right");
//            }

        while (opModeIsActive()) {
            telemetry.addLine("doing things");
            telemetry.update();
        }
    }

    public static class OpenCV_Pipeline extends OpenCvPipeline {

//        public static double anchorPointX1 = 119;
//        public static double anchorPointY1 = 45;

        public static int cameraWidth = 320;

        public static int cameraHeight = 240;

        public static int boxWidth = 35;

        public static int boxHeight = cameraHeight;

        /**
         * Most important section of the code: Colors
         **/
        static final Scalar pissOrange = new Scalar(0, 0, 0);


        // Create a Mat object that will hold the color data
        Mat HSV = new Mat();

        // Make a Constructor
        public OpenCV_Pipeline() {
        }

        // Define the dimensions and location of each region

//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(anchorPointX1, anchorPointY1);
//        static final int REGION1_WIDTH = boxWidth;
//        static final int REGION1_HEIGHT = boxHeight;


        // Create the points that will be used to make the rectangles for the region

//        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);



        // Creates a field of type "Mat"

        ArrayList<Mat> regions = new ArrayList<Mat>();
//        Mat region1;

        // Creating an array for each region which have an element for each channel of interest
        public int[] HSV_Value_2 = new int[3];
        public ArrayList<HSVStorage> HSVValsStorage = new ArrayList<HSVStorage>();

        @Override
        public Mat processFrame(Mat input) {

            // Converts the RGB colors from the video to HSV, which is more useful for image analysis
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);

            // Creates the regions and finds the HSV values for each of the regions
            for(int i = 0; i < cameraWidth; i+= boxWidth){

                Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(i, 0);

                int REGION1_WIDTH = boxWidth;
                int REGION1_HEIGHT = boxHeight;

                Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);

                Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

                regions.add(HSV.submat(new Rect(region1_pointA, region1_pointB)));

                Imgproc.rectangle(HSV, region1_pointA, region1_pointB, pissOrange, 1);
            }
//            regions = HSV.submat(new Rect(region1_pointA, region1_pointB));

            // Loops through each channel of interest
            for(Mat m: regions){
                int[] tempHSVs = new int[3];
                for (int i = 0; i < 3; i++) {
                    tempHSVs[i] = (int) Core.mean(m).val[i];
                }
                HSVValsStorage.add(new HSVStorage(tempHSVs[0], tempHSVs[1], tempHSVs[2]));
            }


            // Draws rectangles representing the regions in the camera stream


            return HSV;

        }
    }
}