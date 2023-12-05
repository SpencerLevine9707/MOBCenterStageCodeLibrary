package org.firstinspires.ftc.teamcode.usefulTuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;

@TeleOp
@Config
public class setAllMotorPowerForTension extends LinearOpMode {
    public static double wheelPower = 1;
    MonkeyMap wBot = new MonkeyMap(this);
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        waitForStart();
        while (opModeIsActive()){
            wBot.frontRight.setPower(wheelPower);
            wBot.frontLeft.setPower(wheelPower);
            wBot.backRight.setPower(wheelPower);
            wBot.backLeft.setPower(wheelPower);
        }
    }
}
