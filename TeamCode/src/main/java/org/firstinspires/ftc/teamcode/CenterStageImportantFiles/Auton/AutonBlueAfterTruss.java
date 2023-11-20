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
        wBot.initPoses();
        wBot.toggleRotator();
        wBot.flipUp();
        wBot.openGrabber();
        wBot.resetKnocker();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = wBot.startingPositionAfterTrussBlue;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        webcam2.setPipeline(pipeline);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerBlue, OpenCVGreatestColorTest.upperBlue);
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
                preloadPlacement = wBot.beacon1AfterTrussBlue;
                firstPlacement = wBot.placementBlueBeacon1;
            }
            else if(zoneDetected == 2){
                preloadPlacement = wBot.beacon2AfterTrussBlue;
                firstPlacement = wBot.placementBlueBeacon2;
            }
            else{
                preloadPlacement = wBot.beacon3AfterTrussBlue;
                firstPlacement = wBot.placementBlueBeacon3;
            }
            wBot.closeGrabber();

            posesToGoTo.add(new PosesAndActions(firstPose, ""));
            if(zoneDetected == 3){
                posesToGoTo.add(new PosesAndActions(wBot.beacon3LineUpAfterTrussBlue, ""));
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
            posesToGoTo.add(new PosesAndActions(wBot.lineUpPlacementBlue, "flipDown"));
            posesToGoTo.add(new PosesAndActions(wBot.putSlidesBackDownBeforePlaceBlue, "resetSlides"));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.openGrabber();
            sleep(MonkeyMap.sleepTimePlacePixels);
            wBot.flipUp();
            sleep(MonkeyMap.sleepTimeFlipForFirstPlaceAfterTruss);
            wBot.placeSlides();
            sleep(MonkeyMap.sleepTimePutSlidesUpNoBreakFlipper);

            wBot.goToPickUpInAuton(follower, posesToGoTo, true, wBot.stackKnockerPosBlue);
            wBot.knockStack();
            sleep(MonkeyMap.sleepTimeKnockStack);
            wBot.resetKnocker();

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, "resetSlides"));
            wBot.loadPixels();
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            sleep(MonkeyMap.sleepTimePickUpPixel);

            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.beforePickUpAfterKnockedBlue, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            wBot.autonVisionPickUp(follower, posesToGoTo, true);

            wBot.placeInAuton(follower, posesToGoTo, true, wBot.placementBlue, false);

            for (int i = 0; i < MonkeyMap.timesToRunAuton - 1; i++) {
                wBot.autonLoop(follower, posesToGoTo, true, false);
            }
            if(!isParkFinal){
                wBot.goToPickUpInAuton(follower, posesToGoTo, true, wBot.pickUpSpotBlue);
            }
            telemetry.addData("Time for auton ", timeForAuton);
            telemetry.update();
            requestOpModeStop();
        }
    }
}

