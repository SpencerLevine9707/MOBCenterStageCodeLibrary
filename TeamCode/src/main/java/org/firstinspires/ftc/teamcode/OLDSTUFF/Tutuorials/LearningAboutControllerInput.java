package org.firstinspires.ftc.teamcode.OLDSTUFF.Tutuorials;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Scanner;

@Config
@TeleOp
@Disabled
public class LearningAboutControllerInput extends LinearOpMode {
    DcMotor motor1;
    Servo servo1;
    Servo grabber;
    public static double grabberOpen = 0.5, grabberClosed = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        grabber = hardwareMap.get(Servo.class, "grabber");

        boolean aPressable = true;

        boolean bPressable = true;

        boolean isOn = true;

        servo1.setPosition(0);

        waitForStart();

        while(opModeIsActive()){
            double lt = gamepad1.left_trigger;
            if(lt > 0){
                servo1.setPosition(lt);
            }
            else{
                servo1.setPosition(0);
            }

            double lj = gamepad1.left_stick_x;
            if(lj > 0 || lj < 0){
                motor1.setPower(lj);
            }
            else{
                motor1.setPower(0);
            }

            boolean a = gamepad1.a;

            if(a && aPressable){
                if(grabber.getPosition() >= grabberClosed - 0.1 && grabber.getPosition() <= grabberClosed + 0.1){
                    openGrabber();
                }
                else{
                    grabber.setPosition(grabberClosed);
                }
            }
            aPressable = !a;

            boolean b = gamepad1.b;

            if(b && bPressable){
                if(isOn){

                }
                else{

                }
                isOn = !isOn;
            }
            bPressable = !b;


        }

    }
    public void openGrabber(){
        grabber.setPosition(grabberOpen);
    }
}
