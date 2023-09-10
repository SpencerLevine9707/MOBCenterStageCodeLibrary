package org.firstinspires.ftc.teamcode.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "Levine Local")
public class GoThroughPointsThroughDifClassVIsion extends LinearOpMode {
    OpenCvCamera webcam;
    static OpenCVGreatestColorTest pipeline;
    public static int webcamWidth = 320;
    public static int webcamHeight = 240;
    public static double lineDist = 1;
    public static double poseErrorGoToBlock = 0.75;
    PointFollower follower = new PointFollower(this);
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = new Pose2d(0, 0, Math.toRadians(0));
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(firstPose);
//        posesToGoTo.add(new Pose2d(0, 20, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(90)));
//        posesToGoTo.add(new Pose2d(35, 20, Math.toRadians(180)));
//        posesToGoTo.add(new Pose2d(-10, 4, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(-10, -30, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(-60, -50, -20));
//        posesToGoTo.add(new Pose2d(66, 6, -100));
//        posesToGoTo.add(new Pose2d(52, -60, -2000));
//        posesToGoTo.add(new Pose2d(1, -10, -100));
        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

//        follower.init(posesToGoTo);

        waitForStart();

//        follower.goToPoints();

        ArrayList<Pose2d> newPoses = new ArrayList<>();
        newPoses.add(posesToGoTo.get(posesToGoTo.size()-1));
        double xForPose = (lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX)))+newPoses.get(newPoses.size()-1).getX();
        double yForPose = ((lineDist * Math.cos(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+newPoses.get(newPoses.size()-1).getY();
        newPoses.add(new Pose2d(xForPose, yForPose, Math.toRadians(0)));
        Pose2d endingPose = newPoses.get(newPoses.size()-1);
        follower.init(newPoses);

        telemetry.addLine("newPoses " + newPoses);
        telemetry.update();
        follower.goToPoints(true);
//        while(opModeIsActive()){
//            newPoses.clear();
//            newPoses.add(endingPose);
//            xForPose = (lineDist * Math.cos(Math.toRadians(OpenCVGreatestColorTest.thetaX)))+newPoses.get(newPoses.size()-1).getX();
//            yForPose = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+newPoses.get(newPoses.size()-1).getY();
//            newPoses.add(new Pose2d(xForPose, yForPose, Math.toRadians(0)));
//            endingPose = newPoses.get(newPoses.size()-1);
//            follower.init(newPoses);
//            telemetry.addLine("newPoses " + newPoses);
//            telemetry.update();
//            follower.goToPoints(poseErrorGoToBlock);
//        }


    }
}
