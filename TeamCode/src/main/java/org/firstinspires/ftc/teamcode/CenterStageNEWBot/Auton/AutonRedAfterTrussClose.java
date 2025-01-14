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
//public class AutonRedAfterTrussClose extends LinearOpMode {
//    OpenCvCamera webcam;
//    static OpenCVDetectTeamProp colorPipe;
//    static OpenCVGreatestColorTest pipeline;
//    MonkeyMap wBot = new MonkeyMap(this);
//    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
//    PointFollower follower = new PointFollower(this, actionRunner);
//    public static boolean isTest = false;
//    public static boolean isParkFinal = true;
//    public ElapsedTime timeForAuton = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        wBot.initForAuton("redAfterTruss");
//        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        pipeline = new OpenCVGreatestColorTest(telemetry);
//        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerRed, OpenCVGreatestColorTest.upperRed);
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
//
//        while(opModeInInit()){
//            zoneDetected = wBot.TeamPropDetectionReading();
//
//            if(zoneDetected == 1){
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon1;
//                firstPlacement = wBot.firstPlacementBeacon1After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon1";
//            }
//            else if(zoneDetected == 2){
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
//                firstPlacement = wBot.firstPlacementBeacon2After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon2";
//            }
//            else{
//                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
//                firstPlacement = wBot.firstPlacementBeacon3After;
//                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon3";
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
//            wBot.openRightGrabber();
//            sleep(MonkeyMap.sleepTimePlacePurplePixel);
//
//            posesToGoTo.clear();
//            posesToGoTo.add(firstExtendation);
//            posesToGoTo.add(new PosesAndActions(purplePixelPlacement, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//            wBot.openGrabber();
//            sleep(MonkeyMap.sleepTimePlacePixel);
//            wBot.resetSlides();
////            for (int i = 0; i < MonkeyMap.timesToRunAuton; i++) {
////                wBot.autonLoopClose(follower, posesToGoTo, wBot.wrapPixelTypeInt(i), true, i>1);
////            }
//            telemetry.addData("Time for auton ", timeForAuton);
//            telemetry.update();
//            terminateOpModeNow();
//        }
//    }
//}
//
