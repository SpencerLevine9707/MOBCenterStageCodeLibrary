package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public DcMotor frontLeft, frontRight, backLeft, backRight, conveyerMotor, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServo, rotatorServo, intakeNoodleServo, flipperServoLeft, flipperServoRight, wheelServoLeft, wheelServoRight, airplaneServo;

    public VoltageSensor batteryVoltageSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0, grabberServoScalerUp = 1;
    public static double grabberClosed = 0.01, grabberOpen = 0.085;
    public static double wheelServoPow = 1, servoStopPow = 0.5;
    public static double flipperScalarDown = 0.1, flipperScalarUp = 0.9, flipperScalarOffset = 0.05, flipperPosDown = 0.19, flipperPosAcross = 0.96, rotatorPickUp = 0.1075, rotatorPlace = 0.774, rotatorPixel1 = 0, rotatorPixel2 = 0.05, rotatorPixel3 = 0.1, rotatorPixel4 = 0.15, rotatorPixel5 = 0.2;
    public static double airplaneServoLoadedPos = 0.16, airplaneServoGoPos = 0.28;

    //Motor powers and pos
    public static double conveyerPower = -1;

    public static double holdPowerForSlides = -0.1;

    public static int[] pixelHeightsForRotator;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;

    public Pose2d startingPosition, beacon1BeforeTrussRed, beacon2BeforeTrussRed, beacon3BeforeTrussRed, beacon1AfterTrussRed, beacon2AfterTrussRed, beacon3AfterTrussRed, beacon1BeforeTrussBlue, beacon2BeforeTrussBlue, beacon3BeforeTrussBlue, beacon1AfterTrussBlue, beacon2AfterTrussBlue, beacon3AfterTrussBlue, pickUpSpotRed, pickUpSpotBlue, placementRed, placementBlue;
    public boolean grabberIsOpen = true, wheelOn = false, conveyerOn = false, flipperDown = true, airplaneLoaded = true, rotatorDown;

    public MonkeyMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        intakeNoodleServo = myOpMode.hardwareMap.get(Servo.class, "intakeNoodleServo");
        grabberServo = myOpMode.hardwareMap.get(Servo.class, "grabberServo");
        rotatorServo = myOpMode.hardwareMap.get(Servo.class, "rotatorServo");
        flipperServoLeft = myOpMode.hardwareMap.get(Servo.class, "flipperServoLeft");
        flipperServoRight = myOpMode.hardwareMap.get(Servo.class, "flipperServoRight");
        wheelServoLeft = myOpMode.hardwareMap.get(Servo.class, "wheelServoLeft");
        wheelServoRight = myOpMode.hardwareMap.get(Servo.class, "wheelServoRight");
        airplaneServo = myOpMode.hardwareMap.get(Servo.class, "airplaneServo");

        flipperServoLeft.setDirection(Servo.Direction.REVERSE);
        wheelServoRight.setDirection(Servo.Direction.REVERSE);

        flipperServoLeft.scaleRange(flipperScalarDown, flipperScalarUp);
        flipperServoRight.scaleRange(flipperScalarDown + flipperScalarOffset, flipperScalarUp + flipperScalarOffset);
        grabberServo.scaleRange(grabberServoScalerDown, grabberServoScalerUp);


        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0
        conveyerMotor = myOpMode.hardwareMap.get(DcMotor.class, "conveyerMotor");
        armMotorLeft = myOpMode.hardwareMap.get(DcMotor.class, "armMotorLeft");
        armMotorRight = myOpMode.hardwareMap.get(DcMotor.class, "armMotorRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loadPlane();
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
    public void toggleIntakeWheels(){
        if(!wheelOn){
            wheelServoLeft.setPosition(wheelServoPow);
            wheelServoRight.setPosition(wheelServoPow);
            wheelOn = true;
        }
        else{
            wheelServoLeft.setPosition(servoStopPow);
            wheelServoRight.setPosition(servoStopPow);
            wheelOn = false;
        }
    }

    public void toggleConveyer(){
        if(!conveyerOn){
            conveyerMotor.setPower(conveyerPower);
            conveyerOn = true;
        }
        else{
            conveyerMotor.setPower(0);
            conveyerOn = false;
        }
    }

    public void setFlipperPos(double pos){
        flipperServoLeft.setPosition(pos);
        flipperServoRight.setPosition(pos);
    }

    public void toggleFlipper() {
        if (flipperDown) {
            setFlipperPos(flipperPosAcross);
            flipperDown = false;
        } else {
            setFlipperPos(flipperPosDown);
            flipperDown = true;
        }
    }
    public void shootPlane(){
        airplaneServo.setPosition(airplaneServoGoPos);
    }
    public void loadPlane(){
        airplaneServo.setPosition(airplaneServoLoadedPos);
    }
    public void toggleAirplane() {
        if (airplaneLoaded) {
            shootPlane();
            airplaneLoaded = false;
        } else {
            loadPlane();
            airplaneLoaded = true;
        }
    }
    public void encodedSlipperySlides(int pos, double power) {
        armMotorLeft.setTargetPosition(pos);
        armMotorRight.setTargetPosition(pos);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setSlidePowers(power);
    }
    public void setSlidePowers(double power){
        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);
    }
    public void rotateDown(){
        rotatorServo.setPosition(rotatorPickUp);
    }
    public void rotateUp(){
        rotatorServo.setPosition(rotatorPlace);
    }
    public void toggleRotator() {
        if (rotatorDown) {
            rotateUp();
            rotatorDown = false;
        } else {
            rotateDown();
            rotatorDown = true;
        }
    }







}
