package org.firstinspires.ftc.teamcode.OLDSTUFF.Tutuorials;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
@Disabled
public class MecanumMovementTutuorial extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    public static double movementPower = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        double fl = -y1 + x1 + x2;
//
//        double fr = -y1 - x1 - x2;
//
//        double bl = -y1 - x1 + x2;
//
//        double br = -y1 + x1 - x2;
//
//        double mag = Math.abs(y1) + Math.abs(x1) + Math.abs(x2);
//
//        fl /= mag;
//
//        fr /= mag;
//
//        bl /= mag;
//
//        br /= mag;
//
//        fl *= movementPower;
//
//        fr *= movementPower;
//
//        bl *= movementPower;
//
//        br *= movementPower;

        waitForStart();

        while(opModeIsActive()){
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x2 = gamepad2.right_stick_x;

            double powerFromTriggers = x1 + y1;

            mecanumMove(x1, y1, x2, powerFromTriggers);
        }
    }
    public void mecanumMove(double x1, double y1, double x2, double movementPower){
        double fl = -y1 + x1 + x2;
        double fr = -y1 - x1 - x2;

        double bl = -y1 - x1 + x2;

        double br = -y1 + x1 - x2;

        double mag = Math.abs(y1) + Math.abs(x1) + Math.abs(x2);

        fl /= mag;

        fr /= mag;

        bl /= mag;

        br /= mag;

        fl *= movementPower;

        fr *= movementPower;

        bl *= movementPower;

        br *= movementPower;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
