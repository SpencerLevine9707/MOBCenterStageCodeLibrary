package org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.LevineLocalization.MathsAndStuff;
import org.firstinspires.ftc.teamcode.LevineLocalization.WheelPowers;

@Config
public class AntiDriftDrive {
    private DcMotor frontLeftDrive;

    private DcMotor frontRightDrive;

    private DcMotor backLeftDrive;

    private DcMotor backRightDrive;
    public static double headingChangeSpeed = -7.5;
    public static double angError = 0;
    public double targetHeading = Math.toRadians(0);

    public AntiDriftDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){

        //set all the motors

        frontLeftDrive = fl;

        frontRightDrive = fr;

        backLeftDrive = bl;

        backRightDrive = br;
    }

    public void moveInTeleop(double x1, double y1, double x2, double movementPower, double currentHeading){
        //power equals forwards + sideways + rotation

        double fl = -y1 + x1;

        double fr = -y1 - x1;

        double bl = -y1 - x1;

        double br = -y1 + x1;

        double ADPower = movementPower*(-y1 + x1);

        double BCPower = movementPower*(-y1 - x1);

        targetHeading += (x2*Math.toRadians(headingChangeSpeed));

        targetHeading = MathsAndStuff.AngleWrap(targetHeading);

        double totAngDist = targetHeading - MathsAndStuff.AngleWrap(currentHeading);

        double turnPower = MathsAndStuff.AngleWrap(totAngDist);

//        if(totAngDist < Math.toRadians(angError)){
//            turnPower = 0;
//        }



        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0){
            turningScale = 1.0;
        }
//
//        double fl = (ADPower - turnPower) / turningScale;
//        double fr = (BCPower + turnPower) / turningScale;
//        double bl = (BCPower - turnPower) / turningScale;
//        double br = (ADPower + turnPower) / turningScale;

        fl -= turnPower; fl /= turningScale;
        fr += turnPower; fr /= turningScale;
        bl -= turnPower; bl /= turningScale;
        br += turnPower; br /=turningScale;



//        //scale each motor power by the movement power
//
//        fl *= movementPower;
//
//        fr *= movementPower;
//
//        bl *= movementPower;
//
//        br *= movementPower;



        //activate the motion controller

        setMotorPowers(fl, fr, bl, br);

    }

    public void setMotorPowers(double fl, double fr, double bl, double br){
        //set the power of each motor

        frontLeftDrive.setPower(fl);

        frontRightDrive.setPower(fr);

        backLeftDrive.setPower(bl);

        backRightDrive.setPower(br);

    }

    public void moveTowardsAngleTheta(double theta, double movementPower, double currentHeading) {
        // Get dists
//        double totAngDist = targetPose.getHeading() - currPose.getHeading();

        // Get theta
        double totAngDist = targetHeading - MathsAndStuff.AngleWrap(currentHeading);

        // Get Powers
        double ADPower = movementPower * (Math.sin(theta) + Math.cos(theta));
        double BCPower = movementPower * (Math.sin(theta) - Math.cos(theta));

        double turnPower = MathsAndStuff.AngleWrap(totAngDist);
        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0){
            turningScale = 1.0;
        }

        double fl = (ADPower - turnPower) / turningScale;
        double fr = (BCPower + turnPower) / turningScale;
        double bl = (BCPower - turnPower) / turningScale;
        double br = (ADPower + turnPower) / turningScale;

        setMotorPowers(fl, fr, bl, br);
    }
    public void moveTowardsAngleThetaAndChangeAngle(double thetaPos, double thetaAngle, double movementPower) {
        // Get dists

        // Get Powers
        double ADPower = movementPower * (Math.sin(thetaPos) + Math.cos(thetaPos));
        double BCPower = movementPower * (Math.sin(thetaPos) - Math.cos(thetaPos));

        double turnPower = MathsAndStuff.AngleWrap(thetaAngle);
        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0){
            turningScale = 1.0;
        }

        double fl = (ADPower - turnPower) / turningScale;
        double fr = (BCPower + turnPower) / turningScale;
        double bl = (BCPower - turnPower) / turningScale;
        double br = (ADPower + turnPower) / turningScale;

        setMotorPowers(fl, fr, bl, br);
    }



}
