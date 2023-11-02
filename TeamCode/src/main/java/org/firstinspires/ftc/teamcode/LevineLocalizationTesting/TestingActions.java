//package org.firstinspires.ftc.teamcode.LevineLocalizationTesting;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
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
//public class TestingActions extends LinearOpMode {
//    public static double xPosStartingPos = 0, yPosStartingPos = 0;
//    public static double xPosBeforePole = 5, yPosBeforePole = 0;
//    public static double xPosPole = 10, yPosPole = 0;
//    public static double xPosCones = 0, yPosCones = 0;
//    public static double headingPole = Math.toRadians(0), headingStart = Math.toRadians(0), headingBetween = Math.toRadians(0), headingCones = Math.toRadians(0);
//    public static int sleepTimeWait = 2000;
//    public Servo spencerLikesKids;
//
//
//    PointFollower follower = new PointFollower(this);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
//        Pose2d firstPose = new Pose2d(xPosStartingPos, yPosStartingPos, headingStart);
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        Pose2d endingPose;
//        spencerLikesKids = hardwareMap.get(Servo.class, "spencerLikesKids");
//        spencerLikesKids.setPosition(0);
//
//        posesToGoTo.add(firstPose);
//        posesToGoTo.add(new Pose2d(xPosBeforePole, yPosBeforePole, headingBetween));
//        follower.init(posesToGoTo, false);
//        waitForStart();
//        follower.goToPoints(false);
//        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//        posesToGoTo.clear();
//        posesToGoTo.add(endingPose);
//        posesToGoTo.add(new Pose2d(xPosPole, yPosPole, headingPole));
//        spencerLikesKids.setPosition(1);
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//    }
//}