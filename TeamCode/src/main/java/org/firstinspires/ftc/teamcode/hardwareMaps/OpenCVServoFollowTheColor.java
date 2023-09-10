package org.firstinspires.ftc.teamcode.hardwareMaps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.teamcode.hardwareMaps.openCVTestLeft;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;

@TeleOp(group="Vision Testing")
@Disabled

public class OpenCVServoFollowTheColor extends LinearOpMode {

    OpenCvCamera webcam;

    static openCVTestLeft.OpenCV_Pipeline pipeline;

    public static int centerForX;
    public static int centerForY;

    public static double servoXHomePos = 0.5;
    public static double servoYHomePos = 0.5;

    public Servo turretX, turretY;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        turretX = hardwareMap.get(Servo.class, ("turretX"));
        turretY = hardwareMap.get(Servo.class, ("turretY"));


        // Set up pipeline
        pipeline = new openCVTestLeft.OpenCV_Pipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (opModeInInit()) {
            telemetry.addData("Center For X and then Y ", centerForX + " , " + centerForY);
            turretX.setPosition(servoXHomePos);
            turretY.setPosition(servoYHomePos);
            telemetry.addLine("\nWaiting for start");
            telemetry.update();
        }
        while (opModeIsActive()){
            telemetry.addData("Center For X and then Y ", centerForX + " , " + centerForY);
            getServoTurretPos(centerForX, turretX);
            getServoTurretPos(centerForY, turretY);
            telemetry.addData("Turret X: ", turretX.getPosition() + " , Y turret: " + turretY.getPosition());
            telemetry.addLine("Running");
            telemetry.update();
        }
    }
    public void getServoTurretPos(int cameraPos, Servo turret){
        double newServoPos = turret.getPosition() + ((cameraPos - 160)/100);
        if(newServoPos > 1){
            turret.setPosition(1);
        }
        else{
            turret.setPosition(newServoPos);
        }
    }
    public static class OpenCV_Pipeline extends OpenCvPipeline {

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


        // Create the points that will be used to make the rectangles for the regio



        // Creates a field of type "Mat"

        int[] lowerColor = {20, 100, 100};
        int[] upperColor = {30, 255, 255};

        @Override
        public Mat processFrame(Mat frame) {

            // Converts the RGB colors from the video to HSV, which is more useful for image analysis
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, new Scalar(lowerColor[0], lowerColor[1], lowerColor[2]), new Scalar(upperColor[0], upperColor[1], upperColor[2]), mask);

            // Find the contours of the objects in the frame that are within the mask
            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the contour with the largest area
            double maxArea = 0;
            MatOfPoint maxContour = new MatOfPoint(new Point(0, 0));
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    maxContour = contour;
                }
            }

            // Draw a bounding box around the object with the largest area
            if (maxContour != null) {
                Rect boundingBox = Imgproc.boundingRect(maxContour);
                int x = boundingBox.x;
                int y = boundingBox.y;
                int w = boundingBox.width;
                int h = boundingBox.height;

                Imgproc.rectangle(frame, new Point(x, y), new Point(x + w, y + h), new Scalar(0, 255, 0), 2);

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

                Imgproc.circle(frame, new Point(center_x, center_y), circRad, new Scalar(0, 0, 255), 2);
                Imgproc.circle(frame, new Point(center_x, center_y), 3, new Scalar(0, 0, 255), -1);

                centerForX = center_x;
                centerForY = center_y;


                Imgproc.line(frame, new Point(x, y), new Point(Math.abs(x + w), Math.abs(y + h)), new Scalar(255, 0, 0), 1);
                Imgproc.line(frame, new Point(x, Math.abs(y + h)), new Point(Math.abs(x + w), y), new Scalar(255, 0, 0), 1);

                Imgproc.putText(frame, "(" + center_x + ", " + center_y + ")", new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // System.out.println("Center coordinates: (" + center_x + ", " + center_y + ")");
            }

            return hsv;
        }
    }
}

