package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Test;

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
public class LocationIssueTestForMrPeter extends LinearOpMode {
    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    public ElapsedTime timeForAuton = new ElapsedTime();
    public static double xPosFirstPose = 0, yPosFirstPose = 0, headingFirstPose = 90;
    public static double xPosSecondPose = 0, yPosSecondPose = 30, headingSecondPose = 90;
    public static double xPosThirdPose = 30, yPosThirdPose = 0, headingThirdPose = 90;
    public Encoder parallelEncoder, perpendicularEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));
        wBot.init();
        wBot.initForAuton("redAfterTruss");
        wBot.resetSlides();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerRed, OpenCVGreatestColorTest.upperRed);
        webcam.setPipeline(colorPipe);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        Pose2d firstPose = new Pose2d(xPosFirstPose, yPosFirstPose, Math.toRadians(headingFirstPose));
        Pose2d secondPose = new Pose2d(xPosSecondPose, yPosSecondPose, Math.toRadians(headingSecondPose));
        Pose2d thirdPose = new Pose2d(xPosThirdPose, yPosThirdPose, Math.toRadians(headingThirdPose));
        double deadwheelXOGVal = parallelEncoder.getCurrentPosition();
        double deadwheelYOGVal = perpendicularEncoder.getCurrentPosition();
        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        posesToGoTo.add(new PosesAndActions(secondPose, ""));
        follower.init(posesToGoTo, false, true);
        follower.goToPoints(true);
        double deadwheelYValAfterFirstMove = perpendicularEncoder.getCurrentPosition();
        double deadwheelXValAfterFirstMove = parallelEncoder.getCurrentPosition();

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        posesToGoTo.add(new PosesAndActions(thirdPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        double deadwheelYValAfterSecondMove = perpendicularEncoder.getCurrentPosition();
        double deadwheelXValAfterSecondMove = parallelEncoder.getCurrentPosition();

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        double deadwheelYValAfterThirdMove = perpendicularEncoder.getCurrentPosition();
        double deadwheelXValAfterThirdMove = parallelEncoder.getCurrentPosition();

//        sleep(2000);

        while(opModeIsActive()){
            follower.drive.getPoseEstimate();
            follower.drive.update();
            telemetry.addData("deadwheel X OG VAL ", deadwheelXOGVal);
            telemetry.addData("deadwheel Y OG VAL ", deadwheelYOGVal);
            telemetry.addData("deadwheel X First ", deadwheelXValAfterFirstMove);
            telemetry.addData("deadwheel Y First ", deadwheelYValAfterFirstMove);
            telemetry.addData("deadwheel X Second ", deadwheelXValAfterSecondMove);
            telemetry.addData("deadwheel Y Second ", deadwheelYValAfterSecondMove);
            telemetry.addData("deadwheel X Third ", deadwheelXValAfterThirdMove);
            telemetry.addData("deadwheel Y Third ", deadwheelYValAfterThirdMove);
            telemetry.addData("Perp pos ", perpendicularEncoder.getCurrentPosition());
            telemetry.addData("Par Pos ", parallelEncoder.getCurrentPosition());

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            posesToGoTo.add(new PosesAndActions(secondPose, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            posesToGoTo.add(new PosesAndActions(thirdPose, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            telemetry.update();
        }
    }
}

