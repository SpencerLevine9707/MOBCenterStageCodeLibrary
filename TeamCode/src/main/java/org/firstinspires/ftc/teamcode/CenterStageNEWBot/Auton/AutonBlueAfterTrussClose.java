//package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Config
//@Autonomous(group = "Center Stage")
//@Disabled
//public class AutonBlueAfterTrussClose extends LinearOpMode {
//    OpenCvCamera webcam;
//    static OpenCVDetectTeamProp colorPipe;
//    static OpenCVGreatestColorTest pipeline;
//    MonkeyMap wBot = new MonkeyMap(this);
//    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
//    public static int sleepTimeTestAuton = 10000;
//    PointFollower follower = new PointFollower(this, actionRunner);
//    public static boolean isTest = true;
//    public static boolean isParkFinal = true;
//    public ElapsedTime timeForAuton = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        wBot.init();
//        wBot.initForAuton("blueAfterTruss");
//        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new OpenCVGreatestColorTest(telemetry);
//        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerBlue, OpenCVGreatestColorTest.upperBlue);
//        webcam.setPipeline(colorPipe);
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        int zoneDetected = 1;
//
//        Pose2d firstPlacement = new Pose2d();
//        Pose2d purplePixelPlacement = new Pose2d();
//        PosesAndActions firstExtendation = new PosesAndActions(wBot.startExtendFirstPlacementAfter, "");
//        int firstPlaceSlidesPos = 0;
//        double correctorPosFirstPlace = 0;
//
//        while(opModeInInit()){
//            zoneDetected = wBot.TeamPropDetectionReading();
//
//            if(zoneDetected == 1){
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon1;
//                firstPlacement = wBot.firstPlacementBeacon1After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon1";
//                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacons13;
//                correctorPosFirstPlace = MonkeyMap.correctorServoBeacon1PreloadPlace;
//            }
//            else if(zoneDetected == 2){
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
//                firstPlacement = wBot.firstPlacementBeacon2After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon2";
//                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacon2;
//                correctorPosFirstPlace = MonkeyMap.correctorServoMidPos;
//            }
//            else{
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
//                firstPlacement = wBot.firstPlacementBeacon3After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon3";
//                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacons13;
//                correctorPosFirstPlace = MonkeyMap.correctorServoBeacon3PreloadPlace;
//            }
//            telemetry.addData("purplePixelPlacement", purplePixelPlacement);
//            telemetry.addData("firstPlacement", firstPlacement);
//            telemetry.addLine("zoneDetected: " + zoneDetected);
//            telemetry.update();
//        }
//
//        while(opModeIsActive()){
//            timeForAuton.reset();
//            wBot.closeGrabber();
//            wBot.setFlipperPos(MonkeyMap.flipperPosUpPurplePixels);
//            wBot.setRotatorFlush();
//            if(zoneDetected == 2){
//                wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon2AfterPos);
//            }
//
//            posesToGoTo.add(new PosesAndActions(wBot.startingPosition, ""));
//            posesToGoTo.add(new PosesAndActions(purplePixelPlacement, ""));
//            follower.init(posesToGoTo, isTest);
//            follower.goToPoints(true);
//
//            if(zoneDetected == 1) {
//                wBot.extendSlidesFarBeaconAfter();
//            }
//            if(zoneDetected == 2){
//                wBot.extendSlidesMidBeaconAfter();
//            }
//            if(zoneDetected == 3){
//                wBot.extendSlidesCloseBeaconAfter();
//            }
//            sleep(MonkeyMap.sleepTimeExtendSlides);
//            wBot.openRightGrabber();
//            sleep(MonkeyMap.sleepTimePlacePurplePixel);
//            wBot.resetSlides();
//            wBot.flipUpFirstPlace();
//            wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoFirstPlace);
//            wBot.correctorServo.setPosition(correctorPosFirstPlace);
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//            sleep(MonkeyMap.sleepTimeWaitForFlipFirstPlace);
//            wBot.encodedSlipperySlides(firstPlaceSlidesPos, MonkeyMap.slidePowerEncoder);
//            sleep(MonkeyMap.sleepTimeExtendSlides);
//            wBot.openLeftGrabber();
//            sleep(MonkeyMap.sleepTimePlacePixel);
//            wBot.resetArm();
//            wBot.pickUpInAutonClose(follower, posesToGoTo, 0, true, false);
//            wBot.placeInAutonClose(follower, posesToGoTo, true);
////            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, true, false);
////            wBot.placeInAutonFar(follower, posesToGoTo);
////            wBot.pickUpInAutonFar(follower, posesToGoTo, 0, true, true);
////            wBot.placeInAutonFar(follower, posesToGoTo);
////            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, true, true);
////            wBot.placeInAutonFar(follower, posesToGoTo);
//
//            sleep(sleepTimeTestAuton);
//
////            for (int i = 0; i < 2; i++) {
////                wBot.autonLoopFar(follower, posesToGoTo, wBot.wrapPixelTypeInt(i), true, i>1);
////            }
//            telemetry.addData("Time for auton ", timeForAuton);
//            telemetry.update();
//            terminateOpModeNow();
//        }
//    }
//}
//
