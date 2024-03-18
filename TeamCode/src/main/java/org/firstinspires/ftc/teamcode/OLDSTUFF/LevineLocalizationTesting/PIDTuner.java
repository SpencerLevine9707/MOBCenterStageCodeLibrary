package org.firstinspires.ftc.teamcode.OLDSTUFF.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;

import java.util.ArrayList;

@Config
@Autonomous(group = "Levine Local")
public class PIDTuner extends LinearOpMode {
    public static double xDist = 0;
    public static double yDist = -30;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public boolean isAcross = true;
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions   > posesToGoTo = new ArrayList<>();
        PosesAndActions firstPose = new PosesAndActions(new Pose2d(0, 0, Math.toRadians(0)), "");
        PosesAndActions acrossPose = new PosesAndActions(new Pose2d(xDist, yDist, Math.toRadians(0)), "");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        posesToGoTo.add(firstPose);
        posesToGoTo.add(acrossPose);

        follower.init(posesToGoTo, false, true);
        waitForStart();
        follower.goToPoints(true);

//        PosesAndActions endingPose = posesToGoTo.get(posesToGoTo.size()-1);

        while(opModeIsActive()){
            posesToGoTo.clear();
            if(!isAcross){
//                posesToGoTo.add(firstPose);
                posesToGoTo.add(acrossPose);
            }
            else{
//                posesToGoTo.add(acrossPose);
                posesToGoTo.add(firstPose);
            }
            isAcross = !isAcross;
            follower.reinit(posesToGoTo);
            telemetry.addLine("newPoses " + posesToGoTo);
            telemetry.update();
            follower.goToPoints(true);
        }
    }
}