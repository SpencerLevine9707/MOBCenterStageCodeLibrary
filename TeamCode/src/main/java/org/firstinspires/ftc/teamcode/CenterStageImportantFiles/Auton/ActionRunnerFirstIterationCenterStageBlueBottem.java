package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;

import java.util.ArrayList;

public class ActionRunnerFirstIterationCenterStageBlueBottem {
    public LinearOpMode myOpMode;
    ArrayList<String> actionsToRun;
    MonkeyMap wBot;
    Telemetry telemetry;

    public ActionRunnerFirstIterationCenterStageBlueBottem(LinearOpMode opMode, MonkeyMap wBot){
        myOpMode = opMode;
        this.wBot = wBot;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void runActions(String action){
        telemetry.addLine("Run action");
        telemetry.addLine("action is " + action);
        switch (action) {
            case "placeSlidesFirstTime":
                wBot.placeSlidesFirstTime();
                break;
            case "placeSlides":
                wBot.placeSlides();
                break;
            case "resetSlides":
                wBot.resetSlides();
                break;
            case "flipUp":
                wBot.flipUp();
                break;
            case "flipDown":
                wBot.flipDown();
                break;
            case "closeGrabber":
                telemetry.addLine("Closed Grabber");
                wBot.closeGrabber();
                break;
            case "openGrabber":
                wBot.openGrabber();
                break;
            case "stopLoadingPixels":
                wBot.stopLoadingPixels();
                break;
            case "stopLoadingPixels and closeGrabber":
                wBot.stopLoadingPixels();
                wBot.closeGrabber();
                break;
        }
        telemetry.update();
    }
}
