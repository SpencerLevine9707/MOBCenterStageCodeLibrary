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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.BlueAfterTrussPoses;
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
public class AutonBlueAfterTrussFar extends LinearOpMode {
    //Test
    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    public double newXPos, newYPos;
    public ElapsedTime timeForAuton = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initForAuton("blueAfterTruss");
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerBlue, OpenCVGreatestColorTest.upperBlue);
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
        Pose2d purplePixelLineUp = new Pose2d();

        while(opModeInInit()){
            zoneDetected = wBot.TeamPropDetectionReading();

            if(zoneDetected == 3){
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
                purplePixelLineUp = wBot.lineUpPurplePixelAfterTrussBeacon23;
                firstPlacement = wBot.firstPlacementBeacon3After;
            }
            else if(zoneDetected == 1){
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
                purplePixelLineUp = wBot.lineUpPurplePixelAfterTrussBeacon23;
                firstPlacement = wBot.firstPlacementBeacon2After;
            }
            else{
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon1;
                purplePixelLineUp = wBot.lineUpPurplePixelAfterTrussBeacon1;
                firstPlacement = wBot.firstPlacementBeacon1After;
            }

            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while(opModeIsActive()){
            webcam.setPipeline(pipeline);
            timeForAuton.reset();
            wBot.closeGrabber();
            wBot.flipDownPurplePixel();

            posesToGoTo.add(new PosesAndActions(wBot.startingPosition, ""));
            posesToGoTo.add(new PosesAndActions(purplePixelLineUp, ""));
            posesToGoTo.add(new PosesAndActions(purplePixelPlacement, ""));
            follower.init(posesToGoTo, isTest, true);
            follower.goToPoints(true);

            if(zoneDetected == 3) {
                wBot.extendSlidesFarBeaconAfter();
            }
            if(zoneDetected == 1){
                wBot.extendSlidesMidBeaconAfter();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon2AfterPos);
            }
            if(zoneDetected == 2){
                wBot.extendSlidesCloseBeaconAfter();
            }
            sleep(MonkeyMap.sleepTimeExtendSlides);
            wBot.openLeftGrabber();
            sleep(MonkeyMap.sleepTimePlacePurplePixel);
            wBot.setCorrectorMid();
            wBot.resetSlides();
            wBot.flipUpFirstPlace();

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.startExtendFirstPlacementAfter, ""));
            posesToGoTo.add(new PosesAndActions(wBot.turnForFirstPlacementAfter, "extendSlidesPlaceFirstPixel"));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            sleep(MonkeyMap.sleepTimeWaitToPlaceFirstPlacement);
            wBot.openRightGrabber();
            sleep(MonkeyMap.sleepTimeYellowPixel);
            wBot.resetArm();

            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            if(wBot.detectionRightGrabber.getDistance(DistanceUnit.INCH) > MonkeyMap.distForPickUpDetect && wBot.detectionLeftGrabber.getDistance(DistanceUnit.INCH) > MonkeyMap.distForPickUpDetect){
//                newXPos = follower.drive.getPoseEstimate().getX();
//                newYPos = follower.drive.getPoseEstimate().getY();
//            }
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, false, false);
//            follower.drive.setPoseEstimate(new Pose2d(newXPos, newYPos, MonkeyMap.headingPickUp));
//            wBot.placeInAutonFar(follower, posesToGoTo, true);
            sleep(MonkeyMap.sleepTimeWaitToResetAuton);
            wBot.setRotatorFlush();
            wBot.encodedSlipperySlides(MonkeyMap.slidesParkPos, MonkeyMap.slidePowerEncoder);
            sleep(MonkeyMap.sleepTimeWaitToResetAuton);

            telemetry.addData("Time for auton ", timeForAuton);
            telemetry.update();
            terminateOpModeNow();
        }
    }
}

