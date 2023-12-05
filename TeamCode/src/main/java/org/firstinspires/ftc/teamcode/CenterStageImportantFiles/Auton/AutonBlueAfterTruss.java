package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
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
public class AutonBlueAfterTruss extends LinearOpMode {
    OpenCvCamera webcam, webcam2;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    public ElapsedTime timeForAuton = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initPoses("blueAfterTruss");
        wBot.rotateDown();
        wBot.flipUpAndRotate();
        wBot.openGrabber();
        wBot.resetKnocker();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = wBot.startingPosition;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        webcam2.setPipeline(pipeline);
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
        Pose2d preloadPlacement = new Pose2d();

        while(opModeInInit()){
            zoneDetected = wBot.TeamPropDetectionReadingAfterTruss();

            if(zoneDetected == 1){
                preloadPlacement = wBot.beacon1Preload;
                firstPlacement = wBot.placementBeacon1;
            }
            else if(zoneDetected == 2){
                preloadPlacement = wBot.beacon2Preload;
                firstPlacement = wBot.placementBeacon2;
            }
            else{
                preloadPlacement = wBot.beacon3Preload;
                firstPlacement = wBot.placementBeacon3;
            }
            telemetry.addData("preloadPlacement", preloadPlacement);
            telemetry.addData("firstPlacement", firstPlacement);
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while(opModeIsActive()){
            wBot.closeGrabber();

            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            if(zoneDetected == 3){
                posesToGoTo.add(new PosesAndActions(wBot.beacon3LineUpAfterTruss, ""));
            }
            posesToGoTo.add(new PosesAndActions(preloadPlacement, ""));

            follower.init(posesToGoTo, isTest);

            timeForAuton.reset();
            follower.goToPoints(true);
            wBot.unloadPixel();
            sleep(MonkeyMap.sleepTimePlacePreloadBeacon);
            wBot.stopLoadingPixels();

            posesToGoTo.clear();
            wBot.placeSlidesFirstTime();
            if(zoneDetected == 3){
                posesToGoTo.add(new PosesAndActions(wBot.beacon3LineUpAfterTruss, ""));
            }
            if(zoneDetected == 2){
                posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacementBeacon2, "flipDown"));
                posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacement, "resetSlides"));
            }
            else{
                posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacement, "flipDown"));
            }
            posesToGoTo.add(new PosesAndActions(wBot.putSlidesBackDownBeforePlace, "resetSlides"));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.openGrabber();
            sleep(MonkeyMap.sleepTimePlacePixels);
            wBot.flipUpAndRotate();
            sleep(MonkeyMap.sleepTimeFlipForFirstPlaceAfterTruss);
            wBot.placeSlides();
            sleep(MonkeyMap.sleepTimePutSlidesUpNoBreakFlipper);

//            wBot.goToPickUpInAuton(follower, posesToGoTo, wBot.stackKnockerPos);
//            wBot.knockStack();
//            sleep(MonkeyMap.sleepTimeKnockStack);
//            wBot.resetKnocker();
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(wBot.pickUpSpot, "resetSlides"));
//            wBot.loadPixels();
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//            sleep(MonkeyMap.sleepTimePickUpPixel);
//
////            posesToGoTo.clear();
////            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnocked, ""));
////            follower.reinit(posesToGoTo);
////            follower.goToPoints(true);
////
////            wBot.autonVisionPickUp(follower, posesToGoTo);
//
//            wBot.placeInAuton(follower, posesToGoTo, wBot.placementPos, false);
//
//            for (int i = 0; i < MonkeyMap.timesToRunAuton - 1; i++) {
//                wBot.autonLoop(follower, posesToGoTo, false);
//            }
//            if(!isParkFinal){
//                wBot.goToPickUpInAuton(follower, posesToGoTo, wBot.pickUpSpot);
//            }
//            telemetry.addData("Time for auton ", timeForAuton);
//            telemetry.update();
            terminateOpModeNow();
        }
    }
}

