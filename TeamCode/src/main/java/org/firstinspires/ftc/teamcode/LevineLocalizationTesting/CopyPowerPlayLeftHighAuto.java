package org.firstinspires.ftc.teamcode.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class CopyPowerPlayLeftHighAuto extends LinearOpMode {
    OpenCvCamera webcam;
    static OpenCVGreatestColorTest pipeline;
    public static int webcamWidth = 320;
    public static int webcamHeight = 240;
    public static double lineDist = 1;
    public static double poseErrorGoToBlock = 0.75;
    public static double xPosStartingPos = 36, yPosStartingPos = 63;
    public static double xPosBeforePole = 35, yPosBeforePole = 20;
    public static double xPosPole = 33.5, yPosPole = 5;
    public static double xPosCones = 62, yPosCones = 14;
    public static double headingPole = Math.toRadians(30), headingStart = Math.toRadians(90), headingBetween = Math.toRadians(90), headingCones = Math.toRadians(0);
    public static int sleepTimeWait = 2000;


    PointFollower follower = new PointFollower(this);
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = new Pose2d(xPosStartingPos, yPosStartingPos, headingStart);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(firstPose);
        posesToGoTo.add(new Pose2d(xPosBeforePole, yPosBeforePole, headingBetween));
        posesToGoTo.add(new Pose2d(xPosPole, yPosPole, headingPole));
//        posesToGoTo.add(new Pose2d(20, 25, Math.toRadians(45)));
//        posesToGoTo.add(new Pose2d(-60, -50, -20));
//        posesToGoTo.add(new Pose2d(66, 6, -100));
//        posesToGoTo.add(new Pose2d(52, -60, -2000));
//        posesToGoTo.add(new Pose2d(1, -10, -100));
//        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));

        follower.init(posesToGoTo);
        waitForStart();
        follower.goToPoints(true);

        sleep(sleepTimeWait);

        Pose2d endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(new Pose2d(xPosCones, yPosCones, headingCones));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        sleep(sleepTimeWait);

        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(new Pose2d(xPosPole, yPosPole, headingPole));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(new Pose2d(xPosCones, yPosCones, headingCones));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        sleep(sleepTimeWait);

        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(new Pose2d(xPosPole, yPosPole, headingPole));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

//        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//        posesToGoTo.clear();
//        posesToGoTo.add(endingPose);
//        posesToGoTo.add(new Pose2d(xPosCones, yPosCones, headingCones));
//        follower.reinit(posesToGoTo);
//
//        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//        posesToGoTo.clear();
//        posesToGoTo.add(endingPose);
//        posesToGoTo.add(new Pose2d(xPosCones, yPosCones, headingCones));
//        follower.reinit(posesToGoTo);
//
//        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
//        posesToGoTo.clear();
//        posesToGoTo.add(endingPose);
//        posesToGoTo.add(new Pose2d(xPosCones, yPosCones, headingCones));
//        follower.reinit(posesToGoTo);

    }
}