package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;

@Config
@TeleOp
public class DriveBotFirstIteration extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);


    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();

        MecanumDrive drive = new MecanumDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;

            telemetry.addLine("lx1: " + lx1 + " ly1: " + ly1 + " rx1 " + rx1);
            telemetry.update();

            double powerFromTriggers = Math.abs(lx1) + Math.abs(ly1) + Math.abs(rx1);

            drive.moveInTeleop(lx1, ly1, rx1, powerFromTriggers);

        }
    }
}
