package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMapOLDBOT;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;

@Config
@TeleOp
@Disabled
public class FortyFiveDeg extends LinearOpMode {
    MonkeyMapOLDBOT wBot = new MonkeyMapOLDBOT(this);
    public static double motorPow = 0.5;
    public static int sleepTime = 1500;


    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();

        MecanumDrive drive = new MecanumDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){
            wBot.frontLeft.setPower(motorPow);
            wBot.backRight.setPower(motorPow);

            sleep(sleepTime);

            wBot.frontLeft.setPower(0);
            wBot.backRight.setPower(0);

            wBot.backLeft.setPower(motorPow);
            wBot.frontRight.setPower(motorPow);

            sleep(sleepTime);

            wBot.backLeft.setPower(0);
            wBot.frontRight.setPower(0);



        }
    }
}
