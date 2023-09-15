package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;

@Config
@Autonomous
public class ContinuousServosCoveyer extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);

    public static double servoPower = 0.5;
    public static double servoPowerStop = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {

        wBot.init();
        waitForStart();

        while(opModeIsActive()){
            double lt = gamepad1.left_trigger;
            if(lt > 0){
                wBot.conveyerServoLeft.setPosition(lt);
            }
            else{
                wBot.conveyerServoLeft.setPosition(servoPowerStop);
            }


//            wBot.conveyerServoLeft.setPosition(servoPower);
        }
    }
}
