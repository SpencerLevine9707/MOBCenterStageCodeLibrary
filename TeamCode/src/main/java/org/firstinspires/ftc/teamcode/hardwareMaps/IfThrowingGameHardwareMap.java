package org.firstinspires.ftc.teamcode.hardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class IfThrowingGameHardwareMap {
    public ElapsedTime runtime = new ElapsedTime();

    private LinearOpMode myOpMode;

    //All hardware defined when looking at the bot at the front
    public DcMotor frontLeft, frontRight, backLeft, backRight, flyWheelLeft, flyWheelRight, intakeMotorLeft, intakeMotorRight;

    public Servo deadWheelLifter, outakeUpAndDownServo, outakeLeftAndRightServo;

    public VoltageSensor batteryVoltageSensor;

    public static double deadWheelsUpPos = 0.4;

    public static double outakeUpAndDownMiddlePos = 0.5;

    public static double outakeLeftAndRightMiddlePos = 0.5;

    public static int sleepTimeThrowObject = 100, sleepTimeSuckUpObject = 100, sleepTimeTransfer = 200;

    public Pose2d startingPos, placePos, pickUpPos;

    ArrayList<Integer> a = new ArrayList<Integer>();

    //Left side poses and stuff
    public static double xPosStartingPosLeft = 36, yPosStartingPosLeft = -63, headingStartingPosLeft = Math.toRadians(90);

    public static double xPosPlacePosLeft = 10, yPosPlacePosLeft = -10, headingPlacePosLeft = Math.toRadians(30);

    public static double xPosPickUpPosLeft = 10, yPosPickUpPosLeft = 20, headingPickUpPosLeft = Math.toRadians(180);

    //Right side poses and stuff
    public static double xPosStartingPosRight = 36, yPosStartingPosRight = 63, headingStartingPosRight = -Math.toRadians(90);

    public static double xPosPlacePosRight = 10, yPosPlacePosRight = 10, headingPlacePosRight = -Math.toRadians(30);

    public static double xPosPickUpPosRight = 10, yPosPickUpPosRight = -20, headingPickUpPosRight = -Math.toRadians(180);

    //Tans for Trajectories
    public static double placeTanPreload = Math.toRadians(240), placeTan = Math.toRadians(30), pickUpTan = Math.toRadians(0);

    //Trajectories
    public Trajectory placePreload, placeObject, pickUpObject;

    //Define GoofyNoises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public IfThrowingGameHardwareMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(boolean isTeleOp){
        flyWheelLeft = myOpMode.hardwareMap.get(DcMotor.class, ("flyWheelLeft"));
        flyWheelRight = myOpMode.hardwareMap.get(DcMotor.class, ("flyWheelRight"));
        intakeMotorLeft = myOpMode.hardwareMap.get(DcMotor.class, ("intakeMotorLeft"));
        intakeMotorRight = myOpMode.hardwareMap.get(DcMotor.class, ("intakeMotorRight"));

        flyWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void definePosesAndTrajectoriesAuton(boolean isRightSide, SampleMecanumDrive drive){

        if(isRightSide){
            startingPos = new Pose2d(xPosStartingPosRight, yPosStartingPosRight, headingStartingPosRight);
            placePos = new Pose2d(xPosPlacePosRight, yPosPlacePosRight, headingPlacePosRight);
            pickUpPos = new Pose2d(xPosPickUpPosRight, yPosPickUpPosRight, headingPickUpPosRight);

            placeTanPreload = -Math.abs(placeTanPreload);
            placeTan = -Math.abs(placeTan);
            pickUpTan = -Math.abs(pickUpTan);
        }
        else{
            startingPos = new Pose2d(xPosStartingPosLeft, yPosStartingPosLeft, headingStartingPosLeft);
            placePos = new Pose2d(xPosPlacePosLeft, yPosPlacePosLeft, headingPlacePosLeft);
            pickUpPos = new Pose2d(xPosPickUpPosLeft, yPosPickUpPosLeft, headingPickUpPosLeft);

            placeTanPreload = -Math.abs(placeTanPreload);
            placeTan = -Math.abs(placeTan);
            pickUpTan = -Math.abs(pickUpTan);
        }
        drive.setPoseEstimate(startingPos);

        placePreload = drive.trajectoryBuilder(startingPos, true)
                .splineToSplineHeading(placePos, placeTanPreload)
                .build();

        placeObject = drive.trajectoryBuilder(pickUpPos, true)
                .splineToLinearHeading(placePos, placeTan)
                .build();

        pickUpObject = drive.trajectoryBuilder(placePos, true)
                .splineToLinearHeading(pickUpPos, pickUpTan)
                .build();
    }




}
