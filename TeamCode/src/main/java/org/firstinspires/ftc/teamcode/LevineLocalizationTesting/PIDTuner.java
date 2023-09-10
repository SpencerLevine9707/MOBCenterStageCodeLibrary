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
public class PIDTuner extends LinearOpMode {
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
        Pose2d acrossPose = new Pose2d(50, 0, Math.toRadians(0));
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(firstPose);
        posesToGoTo.add(acrossPose);

        follower.init(posesToGoTo);
        waitForStart();
        follower.goToPoints(true);

        Pose2d endingPose = posesToGoTo.get(posesToGoTo.size()-1);

        while(opModeIsActive()){
            posesToGoTo.clear();
            if(endingPose.equals(firstPose)){
                posesToGoTo.add(firstPose);
                posesToGoTo.add(acrossPose);
            }
            else{
                posesToGoTo.add(acrossPose);
                posesToGoTo.add(firstPose);
            }
            endingPose = posesToGoTo.get(posesToGoTo.size()-1);
            follower.reinit(posesToGoTo);
            telemetry.addLine("newPoses " + posesToGoTo);
            telemetry.update();
            follower.goToPoints(true);
        }
    }
}