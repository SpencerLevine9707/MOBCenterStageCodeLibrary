package org.firstinspires.ftc.teamcode.OLDSTUFF.usefulTuners;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.JacksJunk;

@Disabled
public class CheckWheelRPMsTest extends LinearOpMode {
    private double prevTicksBackLeft;
    private double prevTicksBackRight;
    private double prevTicksFrontLeft;
    private double prevTicksFrontRight;
    private double prevTime;

    JacksJunk wBot = new JacksJunk(this); // get the hardware map

    private double ticksPerRevolution = 537.7; // set this to the number of encoder ticks per revolution of your motor

    public void runOpMode() {

        wBot.init(true);
        wBot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder
        wBot.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // run the motor without encoder feedback

        wBot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder
        wBot.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // run the motor without encoder feedback

        wBot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder
        wBot.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // run the motor without encoder feedback

        wBot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // reset the encoder
        wBot.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // run the motor without encoder feedback

        prevTicksBackLeft = wBot.backLeft.getCurrentPosition();
        prevTicksFrontLeft = wBot.frontLeft.getCurrentPosition();
        prevTicksBackRight = wBot.backRight.getCurrentPosition();
        prevTicksFrontRight = wBot.frontRight.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // wait for the OpMode to start

        wBot.runtime.reset();

        prevTime = wBot.runtime.seconds();

        while (opModeIsActive()) {

            wBot.frontRight.setPower(1);
            wBot.frontLeft.setPower(1);
            wBot.backRight.setPower(1);
            wBot.backLeft.setPower(1);

            // get the current time and encoder position
            double currentTime = wBot.runtime.seconds();
            double currentTicksFrontLeft = wBot.frontLeft.getCurrentPosition();
            double currentTicksFrontRight = wBot.frontRight.getCurrentPosition();
            double currentTicksBackLeft = wBot.backLeft.getCurrentPosition();
            double currentTicksBackRight = wBot.backRight.getCurrentPosition();

            // calculate the number of encoder ticks that have occurred since the last loop iteration
            double deltaTicksFrontLeft = currentTicksFrontLeft - prevTicksFrontLeft;
            double deltaTicksFrontRight = currentTicksFrontLeft - prevTicksFrontRight;
            double deltaTicksBackLeft = currentTicksFrontLeft - prevTicksBackLeft;
            double deltaTicksBackRight = currentTicksFrontLeft - prevTicksBackRight;

            // calculate the elapsed time since the last loop iteration
            double deltaTime = currentTime - prevTime;

            // calculate the current motor RPM
            double rpmFrontLeft = (deltaTicksFrontLeft / ticksPerRevolution) / (deltaTime / 60.0);
            double rpmFrontRight = (deltaTicksFrontLeft / ticksPerRevolution) / (deltaTime / 60.0);
            double rpmBackLeft = (deltaTicksBackLeft / ticksPerRevolution) / (deltaTime / 60.0);
            double rpmBackRight = (deltaTicksBackRight / ticksPerRevolution) / (deltaTime / 60.0);

            // update the previous time and encoder position
            prevTime = currentTime;
            prevTicksFrontRight = currentTicksFrontRight;
            prevTicksFrontLeft = currentTicksFrontLeft;
            prevTicksBackRight = currentTicksBackRight;
            prevTicksBackLeft = currentTicksBackLeft;

            // send telemetry with the current RPM value
            telemetry.addData("RPMFrontLeft", rpmFrontLeft);
            telemetry.addData("RPMFrontRight", rpmFrontRight);
            telemetry.addData("RPMBackRight", rpmBackRight);
            telemetry.addData("RPMBackLeft", rpmBackLeft);
            telemetry.update();
        }
    }
}