package org.firstinspires.ftc.teamcode.OLDSTUFF.AidanPracticePal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class AidanCodeAnything extends LinearOpMode{

    DcMotor motor1;
    Servo servo1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        servo1.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            motor1.setPower(1);
            servo1.setPosition(1);
        }
    }
}
