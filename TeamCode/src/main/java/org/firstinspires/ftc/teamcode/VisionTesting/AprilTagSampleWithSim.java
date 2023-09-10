package org.firstinspires.ftc.teamcode.VisionTesting;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class AprilTagSampleWithSim extends OpenCvPipeline {

    private long nativeAprilTagPtr;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    public AprilTagSampleWithSim() {
        // Initialize the AprilTag detector
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(
                AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
                3, // Minimum number of good pixels per tag (adjust as needed)
                3  // Minimum number of tags in the image (adjust as needed)
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        // Detect AprilTags in the grayscale image
        detectedTags = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                nativeAprilTagPtr,
                gray,         // Grayscale input image
                0.175,        // Expected tag size (adjust as needed)
                320.0, 240.0, // Image center (adjust as needed)
                0.0, 0.0      // No lens distortion
        );

        // Draw AprilTag outlines on the input image
        for (AprilTagDetection tag : detectedTags) {
            Point[] corners = new Point[4]; // Assuming there are four corners in an AprilTag

            for (int i = 0; i < 4; i++) {
                corners[i] = new Point(tag.corners[i].x, tag.corners[i].y);
            }

            for (int i = 0; i < corners.length; i++) {
                Imgproc.line(input, corners[i], corners[(i + 1) % corners.length], new Scalar(0, 255, 0), 2);
            }

        }

        // Return the modified input image
        return input;
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void close() {
        // Release resources when the pipeline is closed
        AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
    }
}