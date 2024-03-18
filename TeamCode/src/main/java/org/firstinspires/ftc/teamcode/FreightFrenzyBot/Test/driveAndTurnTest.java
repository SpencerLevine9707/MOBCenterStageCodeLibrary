package org.firstinspires.ftc.teamcode.FreightFrenzyBot.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.ActionRunnerFreightFrenzy;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.MonkeyOperationsMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.util.Encoder;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "Center Stage")
public class driveAndTurnTest extends LinearOpMode {
    MonkeyOperationsMap wBot = new MonkeyOperationsMap(this);
    ActionRunnerFreightFrenzy actionRunner = new ActionRunnerFreightFrenzy(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static double xPosFirstPose = 0, yPosFirstPose = 0, headingFirstPose = 90;
    public static double xPosSecondPose = 0, yPosSecondPose = 10, headingSecondPose = 0;
    public static double xPosThirdPose = 10, yPosThirdPose = 0, headingThirdPose = 90;
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        Pose2d firstPose = new Pose2d(xPosFirstPose, yPosFirstPose, Math.toRadians(headingFirstPose));
        Pose2d secondPose = new Pose2d(xPosSecondPose, yPosSecondPose, Math.toRadians(headingSecondPose));
        Pose2d thirdPose = new Pose2d(xPosThirdPose, yPosThirdPose, Math.toRadians(headingThirdPose));
        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        posesToGoTo.add(new PosesAndActions(secondPose, ""));
        follower.init(posesToGoTo, false, true);
        follower.goToPoints(true);
//
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(firstPose, ""));
//        posesToGoTo.add(new PosesAndActions(thirdPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
//
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(firstPose, ""));
//        follower.reinit(posesToGoTo);
//

//        while(opModeIsActive()){
//            follower.drive.getPoseEstimate();
//            follower.drive.update();
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(firstPose, ""));
//            posesToGoTo.add(new PosesAndActions(secondPose, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(firstPose, ""));
//            posesToGoTo.add(new PosesAndActions(thirdPose, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(firstPose, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//
//            telemetry.update();
//        }
    }
}

