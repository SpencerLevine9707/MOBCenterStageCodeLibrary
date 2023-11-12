package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;

import java.util.ArrayList;

@Config
@Autonomous(group = "Levine Local")
public class TestingActionsNew extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = true;
    public static double xDist = 20;
    public static double yDist = 0;
    public static double xDist2 = 10;
    public static double yDist2 = 0;
    public static String action = "closeGrabber";
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initPoses();
        wBot.toggleRotator();
        wBot.toggleFlipper();
        wBot.openGrabber();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        PosesAndActions firstPose = new PosesAndActions(new Pose2d(0, 0, Math.toRadians(0)), "");
        PosesAndActions secondPose = new PosesAndActions(new Pose2d(xDist2, yDist2, Math.toRadians(0)), action);
        PosesAndActions acrossPose = new PosesAndActions(new Pose2d(xDist, yDist, Math.toRadians(0)), "");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(firstPose);
        posesToGoTo.add(secondPose);
        posesToGoTo.add(acrossPose);

        follower.init(posesToGoTo, isTest);
        waitForStart();
        follower.goToPoints(true);

        requestOpModeStop();
        while(opModeIsActive()){
//            sleep(10000);
            requestOpModeStop();
        }
    }
}