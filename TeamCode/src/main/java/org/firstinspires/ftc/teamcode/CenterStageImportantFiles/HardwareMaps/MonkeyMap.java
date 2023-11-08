package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight, conveyerMotor, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServo, rotatorServo, intakeNoodleServo, flipperServoLeft, flipperServoRight, wheelServoLeft, wheelServoRight, airplaneServo, stackKnocker;

    public VoltageSensor batteryVoltageSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0, grabberServoScalerUp = 1;
    public static double grabberClosed = 0.01, grabberOpen = 0.12;
    public static double wheelServoPow = 1, servoStopPow = 0.5;
    public static double flipperScalarDown = 0.1, flipperScalarUp = 0.9, flipperScalarOffset = 0.05, flipperPosDown = 0.19, flipperPosAcross = 0.98, rotatorPickUp = 0.1075, rotatorPlace = 0.774, rotatorPixel1 = 0, rotatorPixel2 = 0.05, rotatorPixel3 = 0.1, rotatorPixel4 = 0.15, rotatorPixel5 = 0.2;
    public static double airplaneServoLoadedPos = 0.13, airplaneServoGoPos = 0.25;
    public static double stackKnockerKnockedPos = 0.35, stackKnockerResetPos = 0.67;

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPower = 1, stopLoadPower = 0;
    public static int resetSlidesPos = 0, placementSlidesPos = -150, slidePosFirstPlace = -150;

    public static double holdPowerForSlides = -0.1, slidePowerDown = 0.3, slidePowerUp = 0.6;

    public static int[] pixelHeightsForRotator;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;

    public Pose2d startingPositionBeforeTrussRed, startingPositionAfterTrussRed, startingPositionBeforeTrussBlue, startingPositionAfterTrussBlue,  beacon1BeforeTrussRed, beacon2BeforeTrussRed, beacon3BeforeTrussRed, beacon1AfterTrussRed, beacon2AfterTrussRed, beacon3AfterTrussRed, beacon1BeforeTrussBlue, beacon2BeforeTrussBlue, beacon3BeforeTrussBlue, beacon1AfterTrussBlue, beacon2AfterTrussBlue, beacon3AfterTrussBlue, pickUpSpotRed, pickUpSpotBlue, placementRed, placementBlue, placementRedBeacon1, placementRedBeacon2, placementRedBeacon3, placementBlueBeacon1, placementBlueBeacon2, placementBlueBeacon3, underTrussRed, underTrussBlue, slidesDownAfterPlaceBlue, slidesDownAfterPlaceRed, underTrussGoingBackBlue, underTrussGoingBackRed, stackKnockerPosBlue, pickUpPosAfterKnockedBlue, beforePickUpAfterKnockedBlue, afterPlacePosForNoCrashBlue;
    public static double xPosStartingPositionBeforeTrussRed, xPosStartingPositionAfterTrussRed, xPosStartingPositionBeforeTrussBlue = -36, xPosStartingPositionAfterTrussBlue;
    public static double yPosStartingPositionBeforeTrussRed, yPosStartingPositionAfterTrussRed, yPosStartingPositionBeforeTrussBlue = 63, yPosStartingPositionAfterTrussBlue;
    public static double xPosBeacon1BeforeTrussBlue = -33, xPosBeacon1BeforeTrussRed, xPosBeacon2BeforeTrussRed, xPosBeacon3BeforeTrussRed, xPosBeacon1AfterTrussRed, xPosBeacon2AfterTrussRed, xPosBeacon3AfterTrussRed,  xPosBeacon2BeforeTrussBlue = -35, xPosBeacon3BeforeTrussBlue = -42, xPosBeacon1AfterTrussBlue, xPosBeacon2AfterTrussBlue, xPosBeacon3AfterTrussBlue;
    public static double yPosBeacon1BeforeTrussBlue = 35, yPosBeacon1BeforeTrussRed, yPosBeacon2BeforeTrussRed, yPosBeacon3BeforeTrussRed, yPosBeacon1AfterTrussRed, yPosBeacon2AfterTrussRed, yPosBeacon3AfterTrussRed, yPosBeacon2BeforeTrussBlue = 39, yPosBeacon3BeforeTrussBlue = 39, yPosBeacon1AfterTrussBlue, yPosBeacon2AfterTrussBlue, yPosBeacon3AfterTrussBlue;
    public static double xPosPickUpSpotRed, xPosPickUpSpotBlue = -64, xPosStackKnockerPosBlue = -53, xPosPickUpPosAfterKnockedBlue = -64, xPosBeforePickUpAfterKnockedBlue = -60;
    public static double xPosPlacementRed, xPosPlacementBlue = 48, xPosPlacementRedBeacon1 = 50, xPosPlacementRedBeacon2 = 50, xPosPlacementRedBeacon3 = 50, xPosPlacementBlueBeacon1 = 48, xPosPlacementBlueBeacon2 = 48, xPosPlacementBlueBeacon3 = 48, xPosUnderTrussRed, xPosUnderTrussBlue = 20, xPosSlidesDownAfterPlaceBlue = 30, xPosSlidesDownAfterPlaceRed = 30, xPosUnderTrussGoingBackBlue = 0, xPosUnderTrussGoingBackRed, xPosAfterPlacePosForNoCrashBlue = 40;

    public static double yPosPickUpSpotRed, yPosPickUpSpotBlue = 37, yPosStackKnockerPosBlue = 32, yPosPickUpPosAfterKnockedBlue = 45, yPosBeforePickUpAfterKnockedBlue = 36;
    public static double yPosPlacementRed, yPosPlacementBlue = 35, yPosPlacementRedBeacon1, yPosPlacementRedBeacon2, yPosPlacementRedBeacon3, yPosPlacementBlueBeacon1 = 41, yPosPlacementBlueBeacon2 = 35, yPosPlacementBlueBeacon3 = 33, yPosUnderTrussRed, yPosUnderTrussBlue = 36, yPosSlidesDownAfterPlaceBlue = 36, yPosSlidesDownAfterPlaceRed = 34, yPosUnderTrussGoingBackBlue = 36, yPosUnderTrussGoingBackRed, yPosAfterPlacePosForNoCrashBlue = 34;
    public static double headingStartingPositionBeforeTrussRed, headingStartingPositionAfterTrussRed, headingStartingPositionBeforeTrussBlue = Math.toRadians(90), headingStartingPositionAfterTrussBlue, headingBeacon1BeforeTrussBlue = Math.toRadians(90), headingBeacon1BeforeTrussRed = Math.toRadians(90), headingBeacon2BeforeTrussRed, headingBeacon3BeforeTrussRed, headingBeacon1AfterTrussRed, headingBeacon2AfterTrussRed, headingBeacon3AfterTrussRed, headingBeacon2BeforeTrussBlue = Math.toRadians(90), headingBeacon3BeforeTrussBlue = Math.toRadians(90), headingBeacon1AfterTrussBlue, headingBeacon2AfterTrussBlue, headingBeacon3AfterTrussBlue, headingPickUpSpotRed, headingPickUpSpotBlue = Math.toRadians(0), headingPlacementRed, headingPlacementBlue = Math.toRadians(0), headingUnderTrussRed, headingUnderTrussBlue = Math.toRadians(0), headingPickUpPosAfterKnockedBlue = Math.toRadians(0);
    public static int sleepTimePlacePreloadBeacon = 500, sleepTimePickUpPixel = 500, sleepTimePlacePixels = 500, sleepTimeKnockStack = 400;
    public boolean grabberIsOpen = true, wheelOn = false, conveyerOn = false, flipperDown = true, airplaneLoaded = true, rotatorDown = false, isKnocked = false;
    public static double timesToRunAuton = 2;
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

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
        stackKnocker = myOpMode.hardwareMap.get(Servo.class, "stackKnocker");

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

        //Init telementry and webcam for dashboard use
        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

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

    public void initPoses(){
        startingPositionBeforeTrussBlue = new Pose2d(xPosStartingPositionBeforeTrussBlue, yPosStartingPositionBeforeTrussBlue, headingStartingPositionBeforeTrussBlue);
        beacon1BeforeTrussBlue = new Pose2d(xPosBeacon1BeforeTrussBlue, yPosBeacon1BeforeTrussBlue, headingBeacon1BeforeTrussBlue);
        beacon2BeforeTrussBlue = new Pose2d(xPosBeacon2BeforeTrussBlue, yPosBeacon2BeforeTrussBlue, headingBeacon2BeforeTrussBlue);
        beacon3BeforeTrussBlue = new Pose2d(xPosBeacon3BeforeTrussBlue, yPosBeacon3BeforeTrussBlue, headingBeacon3BeforeTrussBlue);

        pickUpSpotBlue = new Pose2d(xPosPickUpSpotBlue, yPosPickUpSpotBlue, headingPickUpSpotBlue);
        stackKnockerPosBlue = new Pose2d(xPosStackKnockerPosBlue, yPosStackKnockerPosBlue, headingPickUpSpotBlue);
        pickUpPosAfterKnockedBlue = new Pose2d(xPosPickUpPosAfterKnockedBlue, yPosPickUpPosAfterKnockedBlue, headingPickUpPosAfterKnockedBlue);
        beforePickUpAfterKnockedBlue = new Pose2d(xPosBeforePickUpAfterKnockedBlue, yPosBeforePickUpAfterKnockedBlue, headingPickUpSpotBlue);


        underTrussBlue = new Pose2d(xPosUnderTrussBlue, yPosUnderTrussBlue, headingUnderTrussBlue);
        placementBlue = new Pose2d(xPosPlacementBlue, yPosPlacementBlue, headingPlacementBlue);
        placementBlueBeacon1 = new Pose2d(xPosPlacementBlueBeacon1, yPosPlacementBlueBeacon1, headingPlacementBlue);
        placementBlueBeacon2 = new Pose2d(xPosPlacementBlueBeacon2, yPosPlacementBlueBeacon2, headingPlacementBlue);
        placementBlueBeacon3 = new Pose2d(xPosPlacementBlueBeacon3, yPosPlacementBlueBeacon3, headingPlacementBlue);
        slidesDownAfterPlaceBlue = new Pose2d(xPosSlidesDownAfterPlaceBlue, yPosSlidesDownAfterPlaceBlue, headingPlacementBlue);
        underTrussGoingBackBlue = new Pose2d(xPosUnderTrussGoingBackBlue, yPosUnderTrussGoingBackBlue, headingPlacementBlue);
        afterPlacePosForNoCrashBlue = new Pose2d(xPosAfterPlacePosForNoCrashBlue, yPosAfterPlacePosForNoCrashBlue, headingPlacementBlue);
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
    public void flipDown(){
        setFlipperPos(flipperPosDown);
        flipperDown = true;
    }
    public void flipUp(){
        setFlipperPos(flipperPosAcross);
        flipperDown = false;
    }

    public void toggleFlipper() {
        if (flipperDown) {
            flipUp();
        } else {
            flipDown();
        }
    }
    public void shootPlane(){
        airplaneServo.setPosition(airplaneServoGoPos);
        airplaneLoaded = false;
    }
    public void loadPlane(){
        airplaneServo.setPosition(airplaneServoLoadedPos);
        airplaneLoaded = true;
    }
    public void toggleAirplane() {
        if (airplaneLoaded) {
            shootPlane();
        } else {
            loadPlane();
        }
    }
    public void knockStack(){
        stackKnocker.setPosition(stackKnockerKnockedPos);
        isKnocked = true;
    }
    public void resetKnocker(){
        stackKnocker.setPosition(stackKnockerResetPos);
        isKnocked = false;
    }
    public void toggleKnocker() {
        if (isKnocked) {
            resetKnocker();
        } else {
            knockStack();
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
        rotatorDown = true;
    }
    public void rotateUp(){
        rotatorServo.setPosition(rotatorPlace);
        rotatorDown = false;
    }
    public void toggleRotator() {
        if (rotatorDown) {
            rotateUp();
        } else {
            rotateDown();
        }
    }
    public void unloadPixel(){
        conveyerMotor.setPower(unloadPower);
    }
    public void stopLoadingPixels(){
        conveyerMotor.setPower(stopLoadPower);
    }
    public void loadPixels(){
        conveyerMotor.setPower(conveyerPower);
    }
    public void resetSlides(){
        encodedSlipperySlides(resetSlidesPos, slidePowerDown);
    }
    public void placeSlides(){
        encodedSlipperySlides(placementSlidesPos, slidePowerUp);
    }
    public void placeSlidesFirstTime(){
        encodedSlipperySlides(slidePosFirstPlace, slidePowerUp);
    }
    public void getReadyPlaceAuton(){
        placeSlides();
        flipDown();
        closeGrabber();
    }
    public void readyPickUpAuton(){
        resetSlides();
        openGrabber();
        flipUp();
    }
    public void getReadyFirstPlaceAuton(){
        encodedSlipperySlides(slidePosFirstPlace, slidePowerUp);
        flipDown();
        closeGrabber();
    }

}
