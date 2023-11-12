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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.AprilTagReader;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(group = "Center Stage")
public class AutonRedBeforeTruss extends LinearOpMode {
    OpenCvCamera webcam;
    //    public static double lineDist = 20;
//    public static double offsetForPickUp = 8;
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
        wBot.initPoses();
        wBot.toggleRotator();
        wBot.flipUp();
        wBot.openGrabber();
        wBot.resetKnocker();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = wBot.startingPositionBeforeTrussRed;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        int tagDetected = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        webcam.setPipeline(pipeline);

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

        Pose2d firstPlacement;
        Pose2d preloadPlacement;

        AprilTagReader aRead = new AprilTagReader(this, telemetry);

        aRead.initAprilTag();

        while(opModeInInit()){
            aRead.telemetryAprilTag();
            sleep(20);
            List<AprilTagDetection> currentDetections = aRead.aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                if(detection.id == AprilTagReader.myTag){
                    double distFrom2 = Math.abs(AprilTagReader.beacon1XPos - detection.center.x);
                    double distFrom3 = Math.abs(AprilTagReader.beacon3XPos - detection.center.x);
//                    double distFrom3 = Math.abs(AprilTagReader.beacon3XPos - detection.center.x);
                    if(distFrom2 < distFrom3){
                        tagDetected = 2;
                    }
                    else if(distFrom3 < distFrom2){
                        tagDetected = 3;
                    }
                }
            }
            if(currentDetections.size()<1){
                tagDetected = 1;
            }
            telemetry.addLine("tagDetected: " + tagDetected);
            telemetry.update();
        }

        while(opModeIsActive()){
            if(tagDetected == 1){
                preloadPlacement = wBot.beacon1BeforeTrussRed;
                firstPlacement = wBot.placementRedBeacon1;
            }
            else if(tagDetected == 2){
                preloadPlacement = wBot.beacon2BeforeTrussRed;
                firstPlacement = wBot.placementRedBeacon2;
            }
            else{
                preloadPlacement = wBot.beacon3BeforeTrussRed;
                firstPlacement = wBot.placementRedBeacon3;
            }

            aRead.visionPortal.close();

            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            if(tagDetected == 1){
                posesToGoTo.add(new PosesAndActions(wBot.beacon1LineUpBeforeTrussRed, ""));
            }
            posesToGoTo.add(new PosesAndActions(preloadPlacement, ""));

            follower.init(posesToGoTo, isTest);

            timeForAuton.reset();
            follower.goToPoints(true);
            wBot.unloadPixel();
            sleep(MonkeyMap.sleepTimePlacePreloadBeacon);
            wBot.stopLoadingPixels();

            posesToGoTo.clear();
            if(tagDetected == 1){
                posesToGoTo.add(new PosesAndActions(wBot.beacon1KnockingLineUpBeforeTrussRed, ""));
            }
            posesToGoTo.add(new PosesAndActions(wBot.stackKnockerPosRed, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true, MonkeyMap.velForTurn);
            wBot.knockStack();
            sleep(MonkeyMap.sleepTimeKnockStack);
            wBot.resetKnocker();

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotRed, "resetSlides"));
            wBot.loadPixels();
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            sleep(MonkeyMap.sleepTimePickUpPixel);

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.afterPickUpNoPixelCrashRed, ""));
            posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussRed, ""));
            posesToGoTo.add(new PosesAndActions(wBot.underTrussGoingBackRed, "stopLoadingPixels and closeGrabber"));
            posesToGoTo.add(new PosesAndActions(wBot.underTrussRed, "placeSlidesFirstTime"));
            posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlaceRed, "flipDown"));
            posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacementRed, ""));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.openGrabber();
            sleep(MonkeyMap.sleepTimePlacePixels);
            wBot.flipUp();
            sleep(MonkeyMap.sleepTimeAfterFlip);

            for (int i = 0; i < MonkeyMap.timesToRunAuton; i++) {
                wBot.redAutonLoop(follower, posesToGoTo);
            }

            if(!isParkFinal){
                posesToGoTo.clear();
                posesToGoTo.add(new PosesAndActions(wBot.afterPlacePosForNoCrashRed, ""));
                posesToGoTo.add(new PosesAndActions(wBot.underTrussRed, "resetSlides"));
                posesToGoTo.add(new PosesAndActions(wBot.lineUpForTrussRed,""));
                posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedRed, ""));
                follower.reinit(posesToGoTo);
                follower.goToPoints(true);
            }
            telemetry.addData("Time for auton ", timeForAuton);
            telemetry.update();
            requestOpModeStop();
        }
    }
}

