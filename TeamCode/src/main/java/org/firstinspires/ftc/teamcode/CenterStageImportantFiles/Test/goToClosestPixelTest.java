//package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMapOLDBOT;
//import org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton.ActionRunnerCenterStageAuton;
//import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//@Config
//@Autonomous(group = "Center Stage")
//public class goToClosestPixelTest extends LinearOpMode {
//    OpenCvCamera webcam;
//    public static int webcamWidth = 320;
//    public static int webcamHeight = 240;
//    public static double lineDist = 20;
//    public static double offsetForPickUp = 8;
//    static OpenCVGreatestColorTest pipeline;
//    MonkeyMap wBot = new MonkeyMap(this);
//    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
//    PointFollower follower = new PointFollower(this, actionRunner);
//    public static boolean isTest = false;
//    public static boolean isParkFinal = false;
//    public ElapsedTime timeForAuton = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        wBot.init();
//        wBot.initPoses("blueAfterTruss");
////        wBot.toggleRotator();
//        wBot.flipUp();
//        wBot.openGrabber();
//        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
////
//        pipeline = new OpenCVGreatestColorTest(telemetry);
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//        double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX)))) + wBot.beforePickUpAfterKnocked.getY() + offsetForPickUp;
////
//        posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnocked, ""));
//        posesToGoTo.add(new PosesAndActions(new Pose2d(wBot.xPosPickUpPosAfterKnocked, yPosAfterSeeing, MonkeyMapOLDBOT.headingPlaceAndPickUp), ""));
//
//        follower.init(posesToGoTo, isTest);
//        waitForStart();
//
//        timeForAuton.reset();
//        follower.goToPoints(true);
////        wBot.unloadPixel();
////        sleep(MonkeyMap.sleepTimePlacePreloadBeacon);
////        wBot.stopLoadingPixels();
////
////        posesToGoTo.clear();
////        posesToGoTo.add(new PosesAndActions(wBot.stackKnockerPosBlue, ""));
////        follower.reinit(posesToGoTo);
////        follower.goToPoints(true);
////        wBot.knockStack();
////        sleep(MonkeyMap.sleepTimeKnockStack);
////        wBot.resetKnocker();
////
////        posesToGoTo.clear();
////        posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, "resetSlides"));
////        wBot.loadPixels();
////        follower.reinit(posesToGoTo);
////        follower.goToPoints(true);
////        sleep(MonkeyMap.sleepTimePickUpPixel);
////
////        posesToGoTo.clear();
////        posesToGoTo.add(new PosesAndActions(wBot.underTrussGoingBackBlue, "stopLoadingPixels and closeGrabber"));
////        posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "placeSlidesFirstTime"));
////        posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlaceBlue, "flipDown"));
////        posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
////        follower.reinit(posesToGoTo);
////        follower.goToPoints(true);
////        wBot.openGrabber();
////        sleep(MonkeyMap.sleepTimePlacePixels);
////        wBot.flipUp();
////        wBot.resetKnocker();
////
////        for (int i = 0; i < MonkeyMap.timesToRunAuton; i++) {
////            posesToGoTo.clear();
////            posesToGoTo.add(new PosesAndActions(wBot.afterPlacePosForNoCrashBlue, ""));
////            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "resetSlides"));
////            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedBlue, ""));
////            posesToGoTo.add(new PosesAndActions(wBot.pickUpPosAfterKnockedBlue, ""));
////            follower.reinit(posesToGoTo);
////            wBot.loadPixels();
////            follower.goToPoints(true);
////            sleep(MonkeyMap.sleepTimePickUpPixel);
////
////            posesToGoTo.clear();
////            posesToGoTo.add(new PosesAndActions(wBot.underTrussGoingBackBlue, "stopLoadingPixels and closeGrabber"));
////            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "placeSlides"));
////            posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlaceBlue, "flipDown"));
////            posesToGoTo.add(new PosesAndActions(wBot.placementBlue, ""));
////            follower.reinit(posesToGoTo);
////            follower.goToPoints(true);
////            wBot.openGrabber();
////            sleep(MonkeyMap.sleepTimePlacePixels);
////            wBot.flipUp();
////        }
////
////        if(!isParkFinal){
////            posesToGoTo.clear();
////            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "resetSlides"));
////            posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, ""));
////            follower.reinit(posesToGoTo);
////            follower.goToPoints(true);
////        }
//
//
//        while(opModeIsActive()){
//            telemetry.update();
//        }
//    }
//}