package org.firstinspires.ftc.teamcode.hardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JewishMap {
    public ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode myOpMode;

    //All hardware defined when looking at the bot at the front
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public Servo deadWheelLifter;

    public VoltageSensor batteryVoltageSensor;

    public static double deadWheelsUpPos = 0.4;

    //Define GoofyNoises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public JewishMap(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(boolean isTeleOp){

        if(isTeleOp){
            frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
            frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
            backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
            backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else{
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        //Goofy noises
        matchStart = myOpMode.hardwareMap.appContext.getResources().getIdentifier("match_start", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        endgameStart = myOpMode.hardwareMap.appContext.getResources().getIdentifier("endgamestart", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        wIntro   = myOpMode.hardwareMap.appContext.getResources().getIdentifier("phubintro",   "raw", myOpMode.hardwareMap.appContext.getPackageName());
        yabbaDabbaDo = myOpMode.hardwareMap.appContext.getResources().getIdentifier("yabbadabbadoodle", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        driversPickUp   = myOpMode.hardwareMap.appContext.getResources().getIdentifier("drivers_controllers",   "raw", myOpMode.hardwareMap.appContext.getPackageName());
        funnyFunny = myOpMode.hardwareMap.appContext.getResources().getIdentifier("badtothebone", "raw", myOpMode.hardwareMap.appContext.getPackageName());
        teleStart = myOpMode.hardwareMap.appContext.getResources().getIdentifier("teleop_start", "raw", myOpMode.hardwareMap.appContext.getPackageName());

        telemetry.addData(">", "Jackson Harrison Shapiro is ready to start yelling and get frusturated");
        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }
}
