package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "Center Stage")
public class AutonRedBeforeTrussFarOTHERALLIANCEParkSquare extends LinearOpMode {
    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    public ElapsedTime timeForAuton = new ElapsedTime();
    public static int sleepTimeWaitForFriends = 0; //6500 usually works
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initForAuton("redBeforeTruss");
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

        int zoneDetected = 1;

        Pose2d firstPlacement = new Pose2d();
        Pose2d purplePixelPlacement = new Pose2d();
        int firstPlaceSlidesPos = 0;

        while (opModeInInit()) {
            zoneDetected = wBot.TeamPropDetectionReading();

            if(zoneDetected == 2){
                purplePixelPlacement = wBot.goAcrossForBeforeTrussPurplePixelCloseTrussBeacon;
                firstPlacement = wBot.firstPlacementBeacon1After;
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePos;
            }
            else if(zoneDetected == 1){
                purplePixelPlacement = wBot.goAcrossForBeforeTrussPurplePixelCloseMidBeacon;
                firstPlacement = wBot.firstPlacementBeacon2After;
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePos;
            }
            else{
                purplePixelPlacement = wBot.goAcrossForBeforeTrussPurplePixelCloseWallBeacon;
                firstPlacement = wBot.firstPlacementBeacon3After;
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePos;
            }

            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while (opModeIsActive()) {
            timeForAuton.reset();
            wBot.closeGrabber();
            wBot.flipDownPurplePixel();
            wBot.setRotatorFlush();

            posesToGoTo.add(new PosesAndActions(wBot.startingPosition, ""));
            posesToGoTo.add(new PosesAndActions(purplePixelPlacement, ""));
            follower.init(posesToGoTo, isTest, true);
            follower.goToPoints(true);

            if (zoneDetected == 3) {
                wBot.extendSlidesWallBeaconBefore();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon2BeforePos);
            }
            if (zoneDetected == 1) {
                wBot.extendSlidesMidBeaconBefore();
            }
            if (zoneDetected == 2) {
                wBot.extendSlidesTrussBeaconBefore();
            }
            sleep(MonkeyMap.sleepTimeExtendSlides);
            wBot.openRightGrabber();
            sleep(MonkeyMap.sleepTimePlacePurplePixel);
            wBot.resetSlides();
            wBot.setCorrectorMid();
            wBot.flipAndRotateDown6Pixels();

            posesToGoTo.clear();
            if (zoneDetected == 1) {
                posesToGoTo.add(new PosesAndActions(wBot.goAroundPurplePixelBeacon2, ""));
            }
            else{
                posesToGoTo.add(new PosesAndActions(wBot.goAcrossForBeforeTrussPurplePixelFar, ""));
            }
            posesToGoTo.add(new PosesAndActions(wBot.pickUpPixelFar, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.fullyExtendSlides();
            sleep(MonkeyMap.sleepTimeExtendSlides);
            wBot.closeGrabber();
            sleep(MonkeyMap.sleepTimePickUpPixel);
            wBot.resetSlides();

            sleep(sleepTimeWaitForFriends);

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.lineUpForPlaceFar, "flipUpFirstPlaceOtherAlliance"));
//            posesToGoTo.add(new PosesAndActions(wBot.turnAfterPickUpPixelFar, ""));
            posesToGoTo.add(new PosesAndActions(wBot.startArmExtendPlaceFar, "extendSlidesPlaceFirstPixelOtherAlliance"));
//            posesToGoTo.add(new PosesAndActions(wBot.turnForFirstPlacementAfter, ""));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            sleep(MonkeyMap.sleepTimeWaitToPlaceFirstPlacement);
            wBot.openGrabber();
            sleep(MonkeyMap.sleepTimeYellowPixel);
            wBot.resetArm();
//            sleep(MonkeyMap.sleepTimeWaitToResetAuton);

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.lineUpParkSquare, "closeGrabber"));
            posesToGoTo.add(new PosesAndActions(wBot.parkSquare, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            telemetry.addData("Time for auton ", timeForAuton);
            telemetry.update();
            terminateOpModeNow();
        }
    }
}

