package org.firstinspires.ftc.teamcode.OLDSTUFF.Tutuorials;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Scanner;

@Config
@TeleOp
@Disabled
public class LearningAboutEncodersWithSlides extends LinearOpMode {

    DcMotor slideLeft, slideRight;
    public static int slidePos = 1000;
    public static double motorPowSlides = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean aPressable = true;

        waitForStart();

        while(opModeIsActive()){
            boolean a = gamepad1.a;

            if(a && aPressable){
                slidesWithEncoders(slidePos, motorPowSlides);
            }
            aPressable = !a;
        }
    }
    public void slidesWithEncoders(int targetPos, double motorPower){
        slideRight.setTargetPosition(targetPos);
        slideLeft.setTargetPosition(targetPos);

        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRight.setPower(motorPower);
        slideLeft.setPower(motorPower);
    }
}
