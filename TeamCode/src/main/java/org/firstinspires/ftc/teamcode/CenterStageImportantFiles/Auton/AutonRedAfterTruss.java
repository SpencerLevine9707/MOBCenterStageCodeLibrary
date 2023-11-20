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
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectRed;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;

@Config
@Autonomous(group = "Center Stage")
public class AutonRedAfterTruss extends LinearOpMode {
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
        wBot.initPoses();
        wBot.toggleRotator();
        wBot.flipUp();
        wBot.openGrabber();
        wBot.resetKnocker();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = wBot.startingPositionAfterTrussRed;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        webcam2.setPipeline(pipeline);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerRed, OpenCVGreatestColorTest.upperRed);
        webcam.setPipeline(colorPipe);

        int zoneDetected = 0;

        Pose2d firstPlacement;
        Pose2d preloadPlacement;

        while(opModeInInit()){
            zoneDetected = wBot.TeamPropDetectionReadingAfterTruss();
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while(opModeIsActive()){
            if(zoneDetected == 1){
                preloadPlacement = wBot.beacon1AfterTrussRed;
                firstPlacement = wBot.placementRedBeacon1;
            }
            else if(zoneDetected == 2){
                preloadPlacement = wBot.beacon2AfterTrussRed;
                firstPlacement = wBot.placementRedBeacon2;
            }
            else{
                preloadPlacement = wBot.beacon3AfterTrussRed;
                firstPlacement = wBot.placementRedBeacon3;
            }
            wBot.closeGrabber();

            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            if(zoneDetected == 3){
                posesToGoTo.add(new PosesAndActions(wBot.beacon3LineUpAfterTrussRed, ""));
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
            posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacementRed, "flipDown"));
            posesToGoTo.add(new PosesAndActions(wBot.putSlidesBackDownBeforePlaceRed, "resetSlides"));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.openGrabber();
            sleep(MonkeyMap.sleepTimePlacePixels);
            wBot.flipUp();
            sleep(MonkeyMap.sleepTimeFlipForFirstPlaceAfterTruss);
            wBot.placeSlides();
            sleep(MonkeyMap.sleepTimePutSlidesUpNoBreakFlipper);

            wBot.goToPickUpInAuton(follower, posesToGoTo, false, wBot.stackKnockerPosRed);
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
            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedRed, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            wBot.autonVisionPickUp(follower, posesToGoTo, false);

            wBot.placeInAuton(follower, posesToGoTo, false, wBot.placementRed, false);

            for (int i = 0; i < MonkeyMap.timesToRunAuton - 1; i++) {
                wBot.autonLoop(follower, posesToGoTo, false, false);
            }
            if(!isParkFinal){
                wBot.goToPickUpInAuton(follower, posesToGoTo, false, wBot.pickUpSpotRed);
            }
            telemetry.addData("Time for auton ", timeForAuton);
            telemetry.update();
            requestOpModeStop();
        }
    }
}

