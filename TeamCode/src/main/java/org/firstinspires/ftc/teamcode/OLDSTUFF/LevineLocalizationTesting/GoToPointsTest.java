//package org.firstinspires.ftc.teamcode.LevineLocalizationTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Config
//@Autonomous(group = "Levine Local")
//public class GoToPointsTest extends LinearOpMode {
//    OpenCvCamera webcam;
//    static OpenCVGreatestColorTest pipeline;
//    public static int webcamWidth = 320;
//    public static int webcamHeight = 240;
//    public static double lineDist = 1;
//    public static double poseErrorGoToBlock = 0.75;
//    PointFollower follower = new PointFollower(this);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
//        Pose2d firstPose = new Pose2d(0, 0, Math.toRadians(0));
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        posesToGoTo.add(firstPose);
//        posesToGoTo.add(new Pose2d(0, 20, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(35, 20, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(-10, 15, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(0, 0, Math.toRadians(0)));
////        posesToGoTo.add(new Pose2d(20, 25, Math.toRadians(45)));
////        posesToGoTo.add(new Pose2d(-60, -50, -20));
////        posesToGoTo.add(new Pose2d(66, 6, -100));
////        posesToGoTo.add(new Pose2d(52, -60, -2000));
////        posesToGoTo.add(new Pose2d(1, -10, -100));
////        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));
//
//        follower.init(posesToGoTo, false);
//        waitForStart();
//        follower.goToPoints(true);
//
//        Pose2d endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//
//        while(opModeIsActive()){
//            posesToGoTo.clear();
//            posesToGoTo.add(endingPose);
//            posesToGoTo.add(firstPose);
//            posesToGoTo.add(new Pose2d(0, 20, Math.toRadians(0)));
//            posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));
//            posesToGoTo.add(new Pose2d(35, 20, Math.toRadians(0)));
//            posesToGoTo.add(new Pose2d(-10, 15, Math.toRadians(0)));
//            posesToGoTo.add(new Pose2d(0, 0, Math.toRadians(0)));
//            endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//            follower.init(posesToGoTo, false);
//            telemetry.addLine("newPoses " + posesToGoTo);
//            telemetry.update();
//            follower.goToPoints(true);
//        }
//    }
//}