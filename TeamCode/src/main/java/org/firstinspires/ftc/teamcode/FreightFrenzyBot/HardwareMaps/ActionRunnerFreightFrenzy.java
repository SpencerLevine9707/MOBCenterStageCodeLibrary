package org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;

public class ActionRunnerFreightFrenzy {
    public LinearOpMode myOpMode;
    MonkeyOperationsMap wBot;
    Telemetry telemetry;

    public ActionRunnerFreightFrenzy(LinearOpMode opMode, MonkeyOperationsMap wBot){
        myOpMode = opMode;
        this.wBot = wBot;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void runActions(String action){
    }
}
