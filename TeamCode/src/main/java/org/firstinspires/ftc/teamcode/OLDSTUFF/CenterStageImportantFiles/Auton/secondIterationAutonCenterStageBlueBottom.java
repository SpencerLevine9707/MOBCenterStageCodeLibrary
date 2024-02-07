//package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton;
//
//import static java.lang.Thread.sleep;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.AprilTagReader;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
//import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@Config
//@Autonomous(group = "Center Stage")
//public class secondIterationAutonCenterStageBlueBottom extends OpMode {
//    OpenCvCamera webcam;
//    AprilTagReader aRead;
//    public static int webcamWidth = 320;
//    public static int webcamHeight = 240;
//    public static double lineDist = 6;
//    static OpenCVGreatestColorTest pipeline;
//    MonkeyMap wBot = new MonkeyMap(this);
//    ActionRunnerFirstIterationCenterStageBlueBottem actionRunner = new ActionRunnerFirstIterationCenterStageBlueBottem(this, wBot);
//    PointFollower follower = new PointFollower(this, actionRunner);
//    public static boolean isTest = false;
//    public static boolean isParkFinal = false;
//    public ElapsedTime timeForAuton = new ElapsedTime();
//    ArrayList<PosesAndActions> posesToGoTo;
//    public int tagDetected = 0;
//    Pose2d firstPlacement;
//    Pose2d preloadPlacement;
//    Pose2d firstPose = wBot.startingPositionBeforeTrussBlue;
//
//    @Override
//    public void init() {
//        wBot.init();
//        wBot.initPoses();
//        wBot.toggleRotator();
//        wBot.flipUp();
//        wBot.openGrabber();
//        posesToGoTo = new ArrayList<>();
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
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
//
//        aRead = new AprilTagReader(this, telemetry);
//
//        aRead.initAprilTag();
//
//    }
//
//    @Override
//    public void init_loop() {
//        aRead.telemetryAprilTag();
////        sleep(20);
//        List<AprilTagDetection> currentDetections = aRead.aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if(detection.id == AprilTagReader.myTag){
//                double distFrom2 = Math.abs(AprilTagReader.beacon1XPos - detection.center.x);
//                double distFrom3 = Math.abs(AprilTagReader.beacon3XPos - detection.center.x);
////                    double distFrom3 = Math.abs(AprilTagReader.beacon3XPos - detection.center.x);
//                if(distFrom2 < distFrom3){
//                    tagDetected = 2;
//                }
//                else if(distFrom3 < distFrom2){
//                    tagDetected = 3;
//                }
//            }
//        }
//        if(currentDetections.size()<1){
//            tagDetected = 1;
//        }
//        telemetry.addLine("tagDetected: " + tagDetected);
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        if(tagDetected == 1){
//            preloadPlacement = wBot.beacon1BeforeTrussBlue;
//            firstPlacement = wBot.placementBlueBeacon1;
//        }
//        else if(tagDetected == 2){
//            preloadPlacement = wBot.beacon2BeforeTrussBlue;
//            firstPlacement = wBot.placementBlueBeacon2;
//        }
//        else{
//            preloadPlacement = wBot.beacon3BeforeTrussBlue;
//            firstPlacement = wBot.placementBlueBeacon3;
//        }
//
//        aRead.visionPortal.close();
//
//        posesToGoTo.add(new PosesAndActions(firstPose, ""));
////        if(tagDetected == 1){
////            posesToGoTo.add(new PosesAndActions(wBot.beacon1LineUpKnockBlue, ""));
////        }
//        posesToGoTo.add(new PosesAndActions(preloadPlacement, ""));
//
//        follower.init(posesToGoTo, isTest);
////        waitForStart();
//
//        timeForAuton.reset();
//        follower.goToPoints(true);
//        wBot.unloadPixel();
//        try {
//            sleep(MonkeyMap.sleepTimePlacePreloadBeacon);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        wBot.stopLoadingPixels();
//
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(wBot.stackKnockerPosBlue, ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//        wBot.knockStack();
//        try {
//            sleep(MonkeyMap.sleepTimeKnockStack);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        wBot.resetKnocker();
//
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, "resetSlides"));
//        wBot.loadPixels();
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//        try {
//            sleep(MonkeyMap.sleepTimePickUpPixel);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(wBot.afterPickUpNoPixelCrashBlue, ""));
//        posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussBlue, ""));
//        posesToGoTo.add(new PosesAndActions(wBot.underTrussGoingBackBlue, "stopLoadingPixels and closeGrabber"));
//        posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "placeSlidesFirstTime"));
//        posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlaceBlue, "flipDown"));
//        posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//        wBot.openGrabber();
//        try {
//            sleep(MonkeyMap.sleepTimePlacePixels);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        wBot.flipUp();
//        wBot.resetKnocker();
//
//        for (int i = 0; i < MonkeyMap.timesToRunAuton; i++) {
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(wBot.afterPlacePosForNoCrashBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "resetSlides"));
//            posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedBlue, ""));
//            follower.reinit(posesToGoTo);
//            wBot.loadPixels();
//            follower.goToPoints(true);
//
//            double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+MonkeyMap.yPosBeforePickUpAfterKnockedBlue;
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(new Pose2d(MonkeyMap.xPosPickUpPosAfterKnockedBlue, yPosAfterSeeing,MonkeyMap.headingPlaceAndPickUp), ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//
//            try {
//                sleep(MonkeyMap.sleepTimePickUpPixel);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(wBot.afterPickUpNoPixelCrashBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.underTrussGoingBackBlue, "stopLoadingPixels and closeGrabber"));
//            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "placeSlides"));
//            posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlaceBlue, "flipDown"));
//            posesToGoTo.add(new PosesAndActions(wBot.placementBlue, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//            wBot.openGrabber();
//            try {
//                sleep(MonkeyMap.sleepTimePlacePixels);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//            wBot.flipUp();
//        }
//
//        if(!isParkFinal){
//            posesToGoTo.clear();
//            posesToGoTo.add(new PosesAndActions(wBot.afterPlacePosForNoCrashBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "resetSlides"));
//            posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussBlue, ""));
//            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedBlue, ""));
//            follower.reinit(posesToGoTo);
//            follower.goToPoints(true);
//        }
//    }
//    @Override
//    public void loop() {
//    }
//}