package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMapOLDBOT;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;

import java.util.ArrayList;

public class ActionRunnerCenterStageAuton {
    public LinearOpMode myOpMode;
    MonkeyMap wBot;
    Telemetry telemetry;

    public ActionRunnerCenterStageAuton(LinearOpMode opMode, MonkeyMap wBot){
        myOpMode = opMode;
        this.wBot = wBot;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void runActions(String action){
        telemetry.addLine("Run action");
        telemetry.addLine("action is " + action);
        switch (action) {
            case "flipDown and rotateDown 5Pixels":
                wBot.flipAndRotateDown5Pixels();
                break;
            case "flipDown and rotateDown 4Pixels":
                wBot.flipAndRotateDown4Pixels();
                break;
            case "flipDown and rotateDown 3Pixels":
                wBot.flipAndRotateDown3Pixels();
                break;
            case "flipDown and rotateDown 2Pixels":
                wBot.flipAndRotateDown2Pixels();
                break;
            case "flipDown and rotateDown 1Pixel":
                wBot.flipDown();
                wBot.rotatorPickUpAndPlace();
                break;
            case "flipDown and rotateDown6Pixels and fullyExtendSlides":
                wBot.flipAndRotateDownAndExtend6Pixels();
            case "closeGrabber":
                telemetry.addLine("Closed Grabber");
                wBot.closeGrabber();
                break;
            case "openGrabber":
                wBot.openGrabber();
                break;
            case "fullyExtendSlides":
                wBot.fullyExtendSlides();
                break;
            case "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace (blue)":
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPlaceFarPos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
                break;
            case "fullyExtendSlides and setCorrectorPlaceClose and rotateForPlace":
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPlaceClosePos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
                break;
            case "flipUp":
                wBot.flipUp();
                break;
            case"flipDownForAuton":
                wBot.flipDownForAuton();
                break;
            case"flipDownForAuton and setCorrectorPlaceFar and rotateForPlace":
                wBot.flipDownForAuton();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPlaceFarPos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
                break;
            case"flipDownForAuton and setCorrectorPlaceClose and rotateForPlace":
                wBot.flipDownForAuton();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPlaceClosePos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
                break;
            case "extendSlidesFarBeaconAfter":
                wBot.extendSlidesFarBeaconAfter();
                break;
            case "extendSlidesMidBeaconAfter":
                wBot.extendSlidesMidBeaconAfter();
                break;
            case "extendSlidesCloseBeaconAfter":
                wBot.extendSlidesCloseBeaconAfter();
                break;
            case "extendSlidesWallBeaconBefore":
                wBot.extendSlidesWallBeaconBefore();
                break;
            case "extendSlidesMidBeaconBefore":
                wBot.extendSlidesMidBeaconBefore();
                break;
            case "extendSlidesTrussBeaconBefore":
                wBot.extendSlidesTrussBeaconBefore();
                break;
            case "extendSlidesFirstPlacementAfterBeacon1":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon1PreloadAfter();
                break;
            case "extendSlidesFirstPlacementAfterBeacon2":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon2PreloadAfter();
                break;
            case "extendSlidesFirstPlacementAfterBeacon3":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon3PreloadAfter();
                break;
            case "extendSlidesFirstPlacementBeforeBeacon1":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon1PreloadBefore();
                break;
            case "extendSlidesFirstPlacementBeforeBeacon2":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon2PreloadBefore();
                break;
            case "extendSlidesFirstPlacementBeforeBeacon3":
                wBot.flipUpFirstPlace();
                wBot.extendSlidesBeacon3PreloadBefore();
                break;
            case "fullyExtendSlides and openGrabber":
                wBot.openGrabber();
                wBot.fullyExtendSlides();
                break;
            case "fullyExtendSlides and openGrabber and setCorrectorPickUpClose":
                wBot.openGrabber();
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPickUpClose);
                break;
            case "fullyExtendSlides and openGrabber and setCorrectorPickUpCloseMid":
                wBot.openGrabber();
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPickUpMidClose);
                break;
            case"setCorrectorPickUpCloseMid":
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPickUpMidClose);
                break;
            case "fullyExtendSlides and openGrabber and setCorrectorPickUpCloseFar":
                wBot.openGrabber();
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPickUpMidFar);
                break;
            default:
                break;
        }
        telemetry.update();
    }
}
