package org.firstinspires.ftc.teamcode.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
public class MrPeterBigManTest extends LinearOpMode {
    public static double xPosStartingPos = 0, yPosStartingPos = 0, headingStart = Math.toRadians(90);
    public static double xPos1 = 0, yPos1 = 21, heading1 = 90;
    public static double xPos2 = -30, yPos2 = 44, heading2 = 180;
    public static double xPos3 = 80, yPos3 = 44, heading3 = 180;
    public static double xPos4 = 0, yPos4 = 0, heading4 = 0;
    public static int sleepTimeBetween = 1000;
    public Servo spencerLikesKids;


    PointFollower follower = new PointFollower(this);
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = new Pose2d(xPosStartingPos, yPosStartingPos, headingStart);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d endingPose;
        spencerLikesKids = hardwareMap.get(Servo.class, "spencerLikesKids");
        spencerLikesKids.setPosition(0);
        Pose2d pose1 = new Pose2d(xPos1, yPos1, Math.toRadians(heading1));
        Pose2d pose2 = new Pose2d(xPos2, yPos2, Math.toRadians(heading2));
        Pose2d pose3 = new Pose2d(xPos3, yPos3, Math.toRadians(heading3));
        Pose2d pose4 = new Pose2d(xPos4, yPos4, Math.toRadians(heading4));

        posesToGoTo.add(firstPose);
        posesToGoTo.add(pose1);
        follower.init(posesToGoTo);

        waitForStart();

        follower.goToPoints(true);
        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(pose2);

        sleep(sleepTimeBetween);

        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        endingPose = posesToGoTo.get(posesToGoTo.size()-1);
        posesToGoTo.clear();
        posesToGoTo.add(endingPose);
        posesToGoTo.add(pose3);

        sleep(sleepTimeBetween);

        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
    }
}