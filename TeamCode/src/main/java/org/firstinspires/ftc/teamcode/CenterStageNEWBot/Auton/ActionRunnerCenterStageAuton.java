package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;

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
                wBot.closeGrabber();
                break;
            case "flipDown and rotateDown 4Pixels":
                wBot.flipAndRotateDown4Pixels();
                wBot.closeGrabber();
                break;
            case "flipDown and rotateDown 3Pixels":
                wBot.flipAndRotateDown3Pixels();
                wBot.closeGrabber();
                break;
            case "flipDown and rotateDown 2Pixels":
                wBot.flipAndRotateDown2Pixels();
                wBot.closeGrabber();
                break;
            case "flipDown and rotateDown 1Pixel":
                wBot.flipDown();
                wBot.rotatorPickUpAndPlace();
                wBot.closeGrabber();
                break;
            case "flipDown and rotateDown6Pixels and fullyExtendSlides":
                wBot.flipAndRotateDownAndExtend6Pixels();
                break;
            case "flipDown and rotateDown6Pixels":
                wBot.flipAndRotateDown6Pixels();
                break;
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
            case "extendSlidesPlaceFirstPixel":
                wBot.encodedSlipperySlides(MonkeyMap.slidesFirstPlacePos, MonkeyMap.slidePowerEncoder);
                wBot.setAutoRotator(wBot.flipperMotor.getTargetPosition());
                wBot.closeGrabber();
                break;
            case "extendSlidesPlaceFirstPixelOtherAlliance":
                wBot.encodedSlipperySlides(MonkeyMap.slidesFirstPlacePosOtherAlliance, MonkeyMap.slidePowerEncoder);
                wBot.setAutoRotator(wBot.flipperMotor.getTargetPosition());
                break;
            case "extendSlidesPlaceAuton":
                wBot.encodedSlipperySlides(MonkeyMap.slidesPlaceInAuton, MonkeyMap.slidePowerEncoder);
                wBot.setAutoRotator(wBot.flipperMotor.getTargetPosition());
                break;
            case "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace":
                wBot.fullyExtendSlides();
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoPlaceFarPos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
                break;
            case "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace (beacon1)":
//                wBot.encodedSlipperySlides(MonkeyMap.slidesFirstPlacePosBeacons13, MonkeyMap.slidePowerEncoder);
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon1PreloadPlace);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoFirstPlace);
                break;
            case "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace (beacon2)":
//                wBot.encodedSlipperySlides(MonkeyMap.slidesFirstPlacePosBeacon2, MonkeyMap.slidePowerEncoder);
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoMidPos);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoFirstPlace);
                break;
            case "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace (beacon3)":
//                wBot.encodedSlipperySlides(MonkeyMap.slidesFirstPlacePosBeacons13, MonkeyMap.slidePowerEncoder);
                wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon3PreloadPlace);
                wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoFirstPlace);
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
            case"flipUpFirstPlace":
                wBot.flipUpFirstPlace();
                break;
            case"flipUpPlaceInAuton":
                wBot.setFlipperPos(MonkeyMap.flipperPosUpPlaceInAuton, MonkeyMap.flipperPower);
                break;
            case"flipUpFirstPlaceOtherAlliance":
                wBot.setFlipperPos(MonkeyMap.flipperPosFirstPlaceOtherAlliance, MonkeyMap.flipperPower);
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
            case "fullyExtendSlides and openGrabber and setCorrectorMid and setRotatorFlush":
                wBot.openGrabber();
                wBot.fullyExtendSlides();
                wBot.setCorrectorMid();
                wBot.setRotatorFlush();
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
