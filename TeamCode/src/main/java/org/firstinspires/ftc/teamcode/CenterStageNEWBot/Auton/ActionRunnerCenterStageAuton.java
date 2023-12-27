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
            case "flipForFirstPlace":
                wBot.placeSlidesFirstTime();
                break;
            case "flipForPlace":
                wBot.placeSlides();
                break;
            case "flipForPickUp5Pixel":
                wBot.resetSlides();
                break;
            case "flipForPickUp3Pixel":
                wBot.flipUp();
                break;
            case "flipForPickUp1Pixel":
                wBot.flipDown();
                break;
            case "closeGrabber":
                telemetry.addLine("Closed Grabber");
                wBot.closeGrabber();
                break;
            case "openGrabber":
                wBot.openGrabber();
                break;
            case "extendSlidesPickUp":
                wBot.stopLoadingPixels();
                break;
            case "extendSlidesPlace":
                wBot.stopLoadingPixels();
                wBot.closeGrabber();
                break;
            case "extendSlidesBeacon1After":
                wBot.unloadPixel();
                wBot.closeGrabber();
                break;
            case "extendSlidesBeacon2After":
                wBot.stopLoadingPixels();
                wBot.placeSlides();
                break;
            case "extendSlidesBeacon3After":
                wBot.loadPixels();
                break;
            case "extendSlidesBeacon1Before":
                wBot.unloadPixel();
                break;
            case "extendSlidesBeacon2Before":
                wBot.stopLoadingPixels();
                break;
            case "extendSlidesBeacon3Before":
                wBot.loadPixels();
                break;
        }
        telemetry.update();
    }
}
