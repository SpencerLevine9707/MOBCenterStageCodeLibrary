package org.firstinspires.ftc.teamcode.OLDSTUFF.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;

import java.util.ArrayList;

@Config
@Autonomous(group = "Levine Local")
public class BackAndForthW extends LinearOpMode {
    public static double xDist = 50;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = new Pose2d(0, 0, Math.toRadians(0));
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        posesToGoTo.add(new PosesAndActions(new Pose2d(xDist, 0, Math.toRadians(0)), ""));
        posesToGoTo.add(new PosesAndActions(new Pose2d(0, 0, Math.toRadians(0)), ""));
//        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(90)));
//        posesToGoTo.add(new Pose2d(35, 20, Math.toRadians(180)));
//        posesToGoTo.add(new Pose2d(-10, 4, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(20, 25, Math.toRadians(45)));
//        posesToGoTo.add(new Pose2d(-60, -50, -20));
//        posesToGoTo.add(new Pose2d(66, 6, -100));
//        posesToGoTo.add(new Pose2d(52, -60, -2000));
//        posesToGoTo.add(new Pose2d(1, -10, -100));
//        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(0)));

        follower.init(posesToGoTo, false, true);
        waitForStart();
        follower.goToPoints(true);

        while(opModeIsActive()){
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(new Pose2d(xDist, 0, Math.toRadians(0)), ""));
            posesToGoTo.add(new PosesAndActions(new Pose2d(0, 0, Math.toRadians(0)), ""));
            follower.reinit(posesToGoTo);
            telemetry.addLine("newPoses " + posesToGoTo);
            telemetry.update();
            follower.goToPoints(true);
        }
    }
}