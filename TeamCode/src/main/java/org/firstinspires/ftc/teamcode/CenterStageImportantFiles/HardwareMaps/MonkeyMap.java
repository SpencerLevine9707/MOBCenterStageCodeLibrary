package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    private LinearOpMode myOpMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public Servo spencerLikesKids, grabberServo, rotatorServo, conveyerServoLeft, conveyerServoRight;

    public VoltageSensor batteryVoltageSensor;

    //Servo Positions
    public static double grabberClosed = 0.02, grabberOpen = 0.5;

    public static double rotatorDown = 0, rotatorUp = 0.5, rotatorTrasfer = 1, rotatorPixel1 = 0, rotatorPixel2 = 0.05, rotatorPixel3 = 0.1, rotatorPixel4 = 0.15, rotatorPixel5 = 0.2;

    public static int[] pixelHeightsForRotator;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;

    public Pose2d startingPosition, beacon1BeforeTrussRed, beacon2BeforeTrussRed, beacon3BeforeTrussRed, beacon1AfterTrussRed, beacon2AfterTrussRed, beacon3AfterTrussRed, beacon1BeforeTrussBlue, beacon2BeforeTrussBlue, beacon3BeforeTrussBlue, beacon1AfterTrussBlue, beacon2AfterTrussBlue, beacon3AfterTrussBlue, pickUpSpotRed, pickUpSpotBlue, placementRed, placementBlue;

    public MonkeyMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
//        conveyerServoLeft = myOpMode.hardwareMap.get(Servo.class, "conveyerServoLeft");
//        conveyerServoRight = myOpMode.hardwareMap.get(Servo.class, "conveyerServoRight");
        grabberServo = myOpMode.hardwareMap.get(Servo.class, "grabberServo");

        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

//        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void openGrabber(){
        grabberServo.setPosition(grabberOpen);
    }
    public void closeGrabber(){
        grabberServo.setPosition(grabberClosed);
    }

    public void toggleGrabber(){
        if(grabberServo.getPosition() >= grabberClosed - 0.1 && grabberServo.getPosition() <= grabberClosed + 0.1){
            openGrabber();
        }
        else{
            closeGrabber();
        }
    }





}
