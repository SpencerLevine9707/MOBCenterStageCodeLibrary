package org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Config
public class MonkeyOperationsMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;

    //Define all hardware
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public Servo grabberServo, flipperServoLeft, flipperServoRight;
    public VoltageSensor batteryVoltageSensor;

    public HuskyLens huskyLens;
    public static int READ_PERIOD = 1;
    public Deadline rateLimit;

    //Servo Positions
    public static double grabberClosed = 0.62, grabberOpen = 0.75;
    public boolean grabberIsOpen = false;

    public static double flipperServoDownPos = 0.93, flipperServoOverSharedPos = 0.83/*0.12*/, flipperServoOverFirstLayer = 0.05, flipperServoOverSecondLayer = 0.1, flipperServoOverThirdLayer = 0.23, flipperServoPosVisionDetect = 0.83;
    public static double flipperServoOffset = 0, flipperServoUpperLimit = 0.9, flipperServoLowerLimit = 0.1;

    //Goofy noises
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public MonkeyOperationsMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){

        //Init telementry and webcam for dashboard use
        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());


        grabberServo = myOpMode.hardwareMap.get(Servo.class, "grabberServo");
        flipperServoLeft = myOpMode.hardwareMap.get(Servo.class, "flipperServoLeft");
        flipperServoRight = myOpMode.hardwareMap.get(Servo.class, "flipperServoRight");

        flipperServoLeft.scaleRange(flipperServoLowerLimit, flipperServoUpperLimit);
        flipperServoRight.scaleRange(flipperServoLowerLimit + flipperServoOffset, flipperServoUpperLimit + flipperServoOffset);

        flipperServoRight.setDirection(Servo.Direction.REVERSE);

        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, ("backRight"));  //port 0
        huskyLens = myOpMode.hardwareMap.get(HuskyLens.class, "huskylens");
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addLine("HuskyNotFound");
        } else {
            telemetry.addLine("HuskyFound");
        }


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Init voltage sensor
        batteryVoltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

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
    public void openGrabber(){
        grabberServo.setPosition(grabberOpen);
        grabberIsOpen = true;
    }
    public void closeGrabber(){
        grabberServo.setPosition(grabberClosed);
        grabberIsOpen = false;
    }
    public void toggleGrabber(){
        if(grabberIsOpen){
            closeGrabber();
        }
        else{
            openGrabber();
        }
    }
    public void setFlipperPos(double pos){
        flipperServoLeft.setPosition(pos);
        flipperServoRight.setPosition(pos);
    }
    public void flipDown(){
        setFlipperPos(flipperServoDownPos);
    }
    public void flipShared(){
        setFlipperPos(flipperServoOverSharedPos);
    }
    public void flipFirstLayer(){
        setFlipperPos(flipperServoOverFirstLayer);
    }
    public void flipSecondLayer(){
        setFlipperPos(flipperServoOverSecondLayer);
    }
    public void flipThirdLayer(){
        setFlipperPos(flipperServoOverThirdLayer);
    }
    public void flipVisionDetect(){
        setFlipperPos(flipperServoPosVisionDetect);
    }
}
