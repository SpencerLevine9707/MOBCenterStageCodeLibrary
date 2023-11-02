package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.AprilTagReader;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(group = "Center Stage")
public class firstIterationAutonCenterStageBlueBottem extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerFirstIterationCenterStageBlueBottem actionRunner = new ActionRunnerFirstIterationCenterStageBlueBottem(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initPoses();
        wBot.toggleRotator();
        wBot.toggleFlipper();
        wBot.openGrabber();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Pose2d firstPose = wBot.startingPositionBeforeTrussBlue;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        int tagDetected = 0;

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

        if(tagDetected == 1){
            preloadPlacement = wBot.beacon1BeforeTrussBlue;
            firstPlacement = wBot.placementBlueBeacon1;
        }
        else if(tagDetected == 2){
            preloadPlacement = wBot.beacon2BeforeTrussBlue;
            firstPlacement = wBot.placementBlueBeacon2;
        }
        else{
            preloadPlacement = wBot.beacon3BeforeTrussBlue;
            firstPlacement = wBot.placementBlueBeacon3;
        }

        aRead.visionPortal.close();

        posesToGoTo.add(new PosesAndActions(firstPose, ""));
        posesToGoTo.add(new PosesAndActions(preloadPlacement, ""));

        follower.init(posesToGoTo, isTest);
        waitForStart();
        follower.goToPoints(true);
        wBot.unloadPixel();
        sleep(MonkeyMap.sleepTimePlacePreloadBeacon);
        wBot.stopLoadingPixels();

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, ""));
        follower.reinit(posesToGoTo);
        wBot.loadPixels();
        follower.goToPoints(true);
        sleep(MonkeyMap.sleepTimePickUpPixel);
        wBot.stopLoadingPixels();
        wBot.closeGrabber();

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "placeSlidesFirstTime"));
        posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlace, "flipDown"));
        posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        wBot.openGrabber();
        sleep(MonkeyMap.sleepTimePlacePixels);



        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(wBot.slidesDownAfterPlace, "flipUp"));
        posesToGoTo.add(new PosesAndActions(wBot.underTrussBlue, "resetSlides"));
        posesToGoTo.add(new PosesAndActions(wBot.pickUpSpotBlue, ""));

        follower.reinit(posesToGoTo);
        follower.goToPoints(true);


        requestOpModeStop();
        while(opModeIsActive()){
//            sleep(10000);
            requestOpModeStop();
        }
    }
}