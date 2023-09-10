package org.firstinspires.ftc.teamcode.VisionTesting;

//import android.graphics.Bitmap;
//import com.acmerobotics.dashboard.FtcDashboard;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.opencv.android.Utils;

import java.util.ArrayList;




public class CenterStageConceptVisionForSim extends OpenCvPipeline {
    Telemetry telemetry;
    public static int pipelineStage = 0;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 0;
    public static double HUE_MAX = 90;
    public static double SATURATION_MIN = 150;
    public static double SATURATION_MAX = 255;
    public static double VALUE_MIN = 150;
    public static double VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 2500;
    public static String BLUR = "Box Blur";
    private boolean enableDashboard;
//    private FtcDashboard dashboard;

    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();

    private int largestX, largestY;
    private double largestArea;

    public CenterStageConceptVisionForSim(Telemetry telemetry) {
//        this.enableDashboard = enableDashboard;

        this.telemetry = telemetry;

//        dashboard = FtcDashboard.getInstance();

        largestX = -1;
        largestY = -1;
        largestArea = -1;
    }

    @Override
    public Mat processFrame(Mat input) {
        // To add:
        // Look into blurring
        // Use smth diff than color red maybe
        // Tweak boxes

        int[] hsvRed = {5, 100, 100};

        int centerX = input.width() / 2;
        int centerY = input.height() / 2;

        int boxWidth = 50; // Width of the box in pixels
        int boxHeight = 65; // Height of the box in pixels

        int topLeftX = centerX - (boxWidth / 2);
        int topLeftY = centerY - (boxHeight / 2);
        int bottomRightX = centerX + (boxWidth / 2);
        int bottomRightY = centerY + (boxHeight / 2);

        int distFromCentY = 30;

        int distFromCentX = 110;

        Rect box2Rect = new Rect(new Point(topLeftX, topLeftY), new Point(bottomRightX, bottomRightY));
        Rect box1Rect = new Rect(new Point(topLeftX - distFromCentX, topLeftY + distFromCentY), new Point(bottomRightX - distFromCentX, bottomRightY + distFromCentY));
        Rect box3Rect = new Rect(new Point(topLeftX + distFromCentX, topLeftY + distFromCentY), new Point(bottomRightX + distFromCentX, bottomRightY + distFromCentY));

        Mat box2 = new Mat(input, box1Rect);
        Mat box1 = new Mat(input, box2Rect);
        Mat box3 = new Mat(input, box3Rect);

        Mat hsvBox2 = new Mat();
        Mat hsvBox1 = new Mat();
        Mat hsvBox3 = new Mat();

        Imgproc.cvtColor(box1, hsvBox1, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(box2, hsvBox2, Imgproc.COLOR_BGR2HSV);
        Imgproc.cvtColor(box3, hsvBox3, Imgproc.COLOR_BGR2HSV);

        Scalar avgHsv2 = calculateAverageHSV(box1);
        Scalar avgHsv1 = calculateAverageHSV(box2);
        Scalar avgHsv3 = calculateAverageHSV(box3);

        double hsvDist2 = calculateHsvDistance(avgHsv1, hsvRed[0], hsvRed[1], hsvRed[2]);
        double hsvDist1 = calculateHsvDistance(avgHsv2, hsvRed[0], hsvRed[1], hsvRed[2]);
        double hsvDist3 = calculateHsvDistance(avgHsv3, hsvRed[0], hsvRed[1], hsvRed[2]);

        Imgproc.rectangle(input, box1Rect, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, box2Rect, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, box3Rect, new Scalar(0, 255, 0), 2);

        if (hsvDist2 < hsvDist1 && hsvDist2 < hsvDist3) {
            telemetry.addLine("box1 is big boy");
            Imgproc.rectangle(input, box1Rect, new Scalar(0, 0, 255), 2);
        } else if (hsvDist1 < hsvDist2 && hsvDist1 < hsvDist3) {
            telemetry.addLine("box2 is big boy");
            Imgproc.rectangle(input, box2Rect, new Scalar(0, 0, 255), 2);
        } else {
            telemetry.addLine("box3 is big boy");
            Imgproc.rectangle(input, box3Rect, new Scalar(0, 0, 255), 2);
        }
        telemetry.update();

        return input;
    }

    public int[] getPosition() {
        return new int[]{largestX, largestY};
    }

    private static Scalar calculateAverageHSV(Mat roi) {
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(roi, hsvImage, Imgproc.COLOR_BGR2HSV);
        return Core.mean(hsvImage);
    }

    // Helper function to calculate the Euclidean distance between two HSV color vectors
    private static double calculateHsvDistance(Scalar hsv1, double hue2, double saturation2, double value2) {
        double hue1 = hsv1.val[0];
        double saturation1 = hsv1.val[1];
        double value1 = hsv1.val[2];

        // Calculate the Euclidean distance
        double deltaHue = hue1 - hue2;
        double deltaSaturation = saturation1 - saturation2;
        double deltaValue = value1 - value2;

        return Math.sqrt(deltaHue * deltaHue + deltaSaturation * deltaSaturation + deltaValue * deltaValue);
    }
}