package org.firstinspires.ftc.teamcode.VisionTesting;




import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


//@Config
//@Autonomous(group="Spencer aka mid coder")
//@Disabled

//public class DavidConceptVision {
//
//    //    public static double anchorPointX1 = 100;
////    public static double anchorPointY1 = 50;
//    public static double anchorPointX1 = 70;
//    public static double anchorPointY1 = 45;
//
//    public static double anchorPointX2 = 130;
//    public static double anchorPointY2 = 45;
//
//    public static int boxWidth = 50;
//
//    public static int boxHeight = 60;
//
//    //Define Gyr
//
//    // private BNO055IMU imu;
//
//
//    // Define Webcam
//
//    // Create Pipeline
//    static OpenCV_Pipeline pipeline;
//
////    @Override
////    public void runOpMode() throws InterruptedException {
////
////    }
//
////    @Override
////    public void runOpMode() {
////
////
////
////// Send telemetry message to indicate successful Encoder reset
////
////
////        // Set up webcam
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////
////        // Set up pipeline
////        pipeline = new OpenCV_Pipeline();
////        webcam.setPipeline(pipeline);
////
////        // Start camera streaming
////        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
////            @Override
////            public void onOpened() {
////                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
////            }
////
////            @Override
////            public void onError(int errorCode) {
////                /*
////                 * This will be called if the camera could not be opened
////                 */
////            }
////        });
////        while (opModeInInit()) {
////            //Telemetry readings for the HSV values in each region
////            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
////            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
////            telemetry.addLine("\nWaiting for start");
////            telemetry.update();
////        }
//
//
////         Telemetry readings for the HSV values in each region
////            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.HSV_Value_1[0], pipeline.HSV_Value_1[1], pipeline.HSV_Value_1[2]);
////            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
////            telemetry.addData("Region 3", "%7d, %7d, %7d", pipeline.HSV_Value_3[0], pipeline.HSV_Value_3[1], pipeline.HSV_Value_3[2]);
////            //telemetry.update();
////            if ((pipeline.HSV_Value_1[0] < pipeline.HSV_Value_2[0]) && (pipeline.HSV_Value_1[0] < pipeline.HSV_Value_3[0])){
////                telemetry.addLine("Left");
////            }
////            if ((pipeline.HSV_Value_2[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_2[0] < pipeline.HSV_Value_3[0])){
////                telemetry.addLine("Mid");
////            }
////            if ((pipeline.HSV_Value_3[0] < pipeline.HSV_Value_1[0]) && (pipeline.HSV_Value_3[0] < pipeline.HSV_Value_2[0])){
////                telemetry.addLine("Right");
////            }
//
////        while (opModeIsActive()) {
////            telemetry.addLine("doing things");
////            telemetry.update();
////        }
////    }

public class DavidBoxCodeForPropDetect extends OpenCvPipeline {

    /**
     * Most important section of the code: Colors
     **/
    static final Scalar pissOrange = new Scalar(255, 0, 0);


    // Create a Mat object that will hold the color data
    Mat HSV = new Mat();
    public static double anchorPointX1 = 20;
    public static double anchorPointY1 = 70;

    public static double anchorPointX2 = 240;
    public static double anchorPointY2 = 70;

    public static int boxWidth = 70;

    public static int boxHeight = 70;

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(anchorPointX1, anchorPointY1);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(anchorPointX2, anchorPointY2);
    static final int REGION1_WIDTH = boxWidth;
    static final int REGION1_HEIGHT = boxHeight;

    static final int REGION2_WIDTH = boxWidth;
    static final int REGION2_HEIGHT = boxHeight;
    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

    Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

    // Creates a field of type "Mat"
    Mat region1;
    Mat region2;
    Telemetry telemetry;

    // Creating an array for each region which have an element for each channel of interest
    public int[] HSV_Value_1 = new int[3];
    public int[] HSV_Value_2 = new int[3];
    public int[] lowerColor = {0, 0, 0};
    public int[] upperColor = {0, 0, 0};

    // Make a Constructor
    public DavidBoxCodeForPropDetect(Telemetry telemetry, int[] lowerColor, int[] upperColor) {
        this.telemetry = telemetry;
    }

    // Define the dimensions and location of each region



    // Create the points that will be used to make the rectangles for the region

    @Override
    public Mat processFrame(Mat input) {
        // Converts the RGB colors from the video to HSV, which is more useful for image analysis
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV_FULL);
//        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        // Creates the regions and finds the HSV values for each of the regions

        region1 = HSV.submat(new Rect(region1_pointA, region1_pointB));
        region2 = HSV.submat(new Rect(region2_pointA, region2_pointB));

        // Loops through each channel of interest
        for (int i = 0; i < 3; i++) {
            // Finds the average HSV value for each channel of interest (The "i" representing the channel of interest)
            HSV_Value_1[i] = (int) Core.mean(region1).val[i];
            HSV_Value_2[i] = (int) Core.mean(region2).val[i];
        }

        // Draws rectangles representing the regions in the camera stream
        Imgproc.rectangle(HSV, region1_pointA, region1_pointB, pissOrange, 1);
        Imgproc.rectangle(HSV, region2_pointA, region2_pointB, pissOrange, 1);
        telemetry.addData("Region 1", "%7d, %7d, %7d", HSV_Value_1[0], HSV_Value_1[1], HSV_Value_1[2]);
        telemetry.addData("Region 2", "%7d, %7d, %7d", HSV_Value_2[0], HSV_Value_2[1], HSV_Value_2[2]);
//        telemetry.addData("Region 3", "%7d, %7d, %7d", lowerBlue[0], HSV_Value_3[1], HSV_Value_3[2]);
        telemetry.addData("Region 1 is? ", isInRange(HSV_Value_1));
        telemetry.addData("Region 2 is? ", isInRange(HSV_Value_2));
        //telemetry.update();
//        if ((HSV_Value_1[0] < HSV_Value_2[0]) && (HSV_Value_1[0] < HSV_Value_3[0])){
//            telemetry.addLine("Left");
//        }
//        if ((HSV_Value_2[0] < HSV_Value_1[0]) && (HSV_Value_2[0] < HSV_Value_3[0])){
//            telemetry.addLine("Mid");
//        }
//        if ((HSV_Value_3[0] < HSV_Value_1[0]) && (HSV_Value_3[0] < HSV_Value_2[0])){
//            telemetry.addLine("Right");
//        }
        telemetry.update();

        return HSV;
    }
    public boolean isInRange(int[] hsv_value){
        return hsv_value[0] >= lowerColor[0] && hsv_value[0] <= upperColor[0];
    }
}
