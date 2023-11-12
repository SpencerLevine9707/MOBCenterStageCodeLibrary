package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;

import java.util.ArrayList;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;
    public OpMode myOperatorMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight, conveyerMotor, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServo, rotatorServo, intakeNoodleServo, flipperServoLeft, flipperServoRight, wheelServoLeft, wheelServoRight, airplaneServo, stackKnocker;

    public VoltageSensor batteryVoltageSensor;
    public DistanceSensor lineUpSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0, grabberServoScalerUp = 1;
    public static double grabberClosed = 0, grabberOpen = 0.12;
    public static double wheelServoPow = 1, servoStopPow = 0.5;
    public static double flipperScalarDown = 0.1, flipperScalarUp = 0.9, flipperScalarOffset = 0.05, flipperPosDown = 0.15, flipperPosAcross = 0.98, rotatorPickUp = 0.1075, rotatorPlace = 0.774, rotatorPixel1 = 0, rotatorPixel2 = 0.05, rotatorPixel3 = 0.1, rotatorPixel4 = 0.15, rotatorPixel5 = 0.2;
    public static double airplaneServoLoadedPos = 0.13, airplaneServoGoPos = 0.25;
    public static double stackKnockerKnockedPos = 0.35, stackKnockerResetPos = 0.67;

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPower = 1, stopLoadPower = 0;
    public static int resetSlidesPos = 0, placementSlidesPos = -150, slidePosFirstPlace = -150;

    public static double holdPowerForSlides = -0.1, slidePowerDown = 0.3, slidePowerUp = 0.6;

    public static int[] pixelHeightsForRotator;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;


    //Blue Poses
    public Pose2d startingPositionBeforeTrussBlue, startingPositionAfterTrussBlue, beacon1BeforeTrussBlue, beacon2BeforeTrussBlue, beacon3BeforeTrussBlue, beacon1AfterTrussBlue, beacon2AfterTrussBlue, beacon3AfterTrussBlue, pickUpSpotBlue, placementBlue, placementBlueBeacon1, placementBlueBeacon2, placementBlueBeacon3, underTrussBlue, slidesDownAfterPlaceBlue,  underTrussGoingBackBlue, stackKnockerPosBlue, pickUpPosAfterKnockedBlue, beforePickUpAfterKnockedBlue, afterPlacePosForNoCrashBlue, lineUpForTrussBlue, beacon1LineUpBeforeTrussBlue, afterPickUpNoPixelCrashBlue, beacon1KnockingLineUpBeforeTrussBlue, lineUpPlacementBlue, beacon3LineUpAfterTrussBlue, lineUpForFirstPlacementAfterTrussBlue;

    //X Poses
    public static double xPosStartingPositionBeforeTrussBlue = -36, xPosStartingPositionAfterTrussBlue = 10;
    public static double xPosBeacon1BeforeTrussBlue = -35/*-27*/, xPosBeacon2BeforeTrussBlue = -35, xPosBeacon3BeforeTrussBlue = -42, xPosBeacon1AfterTrussBlue = 24, xPosBeacon2AfterTrussBlue = 14, xPosBeacon3AfterTrussBlue = 0, xPosBeacon1LineUpBeforeTrussBlue = -40/*xPosBeacon1LineUpBlue = -37*/, xPosBeacon1KnockingLineUpBeforeTrussBlue = -50, xPosBeacon3LineUpAfterTrussBlue = 13;
    public static double xPosPickUpSpotBlue = -61, xPosStackKnockerPosBlue = -53, xPosPickUpPosAfterKnockedBlue = -61, xPosBeforePickUpAfterKnockedBlue = -50;
    public static double xPosPlacementBlue = 51, xPosUnderTrussBlue = 20, xPosSlidesDownAfterPlaceBlue = 30, xPosUnderTrussGoingBackBlue = 0, xPosAfterPlacePosForNoCrashBlue = 42, xPosLineUpForTrussBlue = -42, xPosAfterPickUpNoPixelCrashBlue = -40, xPosLineUpPlacementBlue = 36, xPosLineUpForFirstPlacementAfterTrussBlue = 20;
    public static double xPosPlacementBlueBeacon1 = 51, xPosPlacementBlueBeacon2 = 51, xPosPlacementBlueBeacon3 = 51;

    //Y Poses
    public static double yPosStartingPositionBeforeTrussBlue = 63, yPosStartingPositionAfterTrussBlue = 63;
    public static double yPosBeacon1BeforeTrussBlue = 35, yPosBeacon2BeforeTrussBlue = 39, yPosBeacon3BeforeTrussBlue = 39, yPosBeacon1AfterTrussBlue = 36, yPosBeacon2AfterTrussBlue = 36, yPosBeacon3AfterTrussBlue = 36, yPosBeacon1LineUpBeforeTrussBlue = 36, yPosBeacon1KnockingLineUpBeforeTrussBlue = 36, yPosBeacon3LineUpAfterTrussBlue = 36;
    public static double yPosPickUpSpotBlue = 40, yPosStackKnockerPosBlue = 35, yPosPickUpPosAfterKnockedBlue = 45, yPosBeforePickUpAfterKnockedBlue = 42;
    public static double yPosPlacementBlue = 37, yPosUnderTrussBlue = 60.5, yPosSlidesDownAfterPlaceBlue = 60.5, yPosUnderTrussGoingBackBlue = 60.5, yPosAfterPlacePosForNoCrashBlue = 62, yPosLineUpForTrussBlue = 60.5, yPosAfterPickUpNoPixelCrashBlue = 48, yPosLineUpPlacementBlue = 20, yPosLineUpForFirstPlacementAfterTrussBlue = 30;
    public static double yPosPlacementBlueBeacon1 = 42, yPosPlacementBlueBeacon2 = 37, yPosPlacementBlueBeacon3 = 33;
    //Headings

    public static double headingStartingPositionBlue = Math.toRadians(90), headingBeacon1Blue = Math.toRadians(90), headingBeacon2Blue = Math.toRadians(90), headingBeacon3Blue = Math.toRadians(90), headingPickUpSpotBlue = Math.toRadians(0), headingBeacon1LineUpBeforeTrussBlue = Math.toRadians(90);

    //Red Poses
    public Pose2d startingPositionBeforeTrussRed,startingPositionAfterTrussRed, beacon1BeforeTrussRed, beacon2BeforeTrussRed, beacon3BeforeTrussRed, beacon1AfterTrussRed, beacon2AfterTrussRed, beacon3AfterTrussRed, pickUpSpotRed, placementRed,  placementRedBeacon1, placementRedBeacon2, placementRedBeacon3, underTrussRed, slidesDownAfterPlaceRed, underTrussGoingBackRed;

    //X Poses
    public static double xPosStartingPositionBeforeTrussRed, xPosStartingPositionAfterTrussRed;
    public static double xPosBeacon1BeforeTrussRed, xPosBeacon2BeforeTrussRed, xPosBeacon3BeforeTrussRed, xPosBeacon1AfterTrussRed, xPosBeacon2AfterTrussRed, xPosBeacon3AfterTrussRed;
    public static double xPosPickUpSpotRed;
    public static double xPosPlacementRed,xPosUnderTrussRed, xPosSlidesDownAfterPlaceRed, xPosUnderTrussGoingBackRed;
    public static double xPosPlacementRedBeacon1, xPosPlacementRedBeacon2, xPosPlacementRedBeacon3;

    //Y Poses
    public static double yPosStartingPositionBeforeTrussRed, yPosStartingPositionAfterTrussRed;
    public static double yPosBeacon1BeforeTrussRed, yPosBeacon2BeforeTrussRed, yPosBeacon3BeforeTrussRed, yPosBeacon1AfterTrussRed, yPosBeacon2AfterTrussRed, yPosBeacon3AfterTrussRed;
    public static double yPosPickUpSpotRed;
    public static double yPosPlacementRed, yPosUnderTrussRed, yPosSlidesDownAfterPlaceRed, yPosUnderTrussGoingBackRed;
    public static double yPosPlacementRedBeacon1, yPosPlacementRedBeacon2, yPosPlacementRedBeacon3;

    //Headings
    public static double headingStartingPositionRed, headingBeacon1Red = Math.toRadians(90), headingBeacon2Red, headingBeacon3Red;

    //All Poses
    public static double headingPlaceAndPickUp = Math.toRadians(0);

    public static int sleepTimePlacePreloadBeacon = 400, sleepTimePickUpPixel = 0, sleepTimePlacePixels = 400, sleepTimeKnockStack = 300, sleepTimeAfterFlip = 500, sleepTimeFlipForFirstPlaceAfterTruss = 600;
    public boolean grabberIsOpen = true, wheelOn = false, conveyerOn = false, flipperDown = true, airplaneLoaded = true, rotatorDown = false, isKnocked = false;
    public static double timesToRunAuton = 2;
    public static double correctWallDist = 24, roomForWallDistError = 0.25;
    public static double lineDist = 20, offsetForPickUp = 8;
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public MonkeyMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public MonkeyMap (OpMode opmode) {
        myOperatorMode = opmode;
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

        lineUpSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "lineUpSensor");

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
        startingPositionBeforeTrussBlue = new Pose2d(xPosStartingPositionBeforeTrussBlue, yPosStartingPositionBeforeTrussBlue, headingStartingPositionBlue);
        startingPositionAfterTrussBlue = new Pose2d(xPosStartingPositionAfterTrussBlue, yPosStartingPositionAfterTrussBlue, headingStartingPositionBlue);

        beacon1BeforeTrussBlue = new Pose2d(xPosBeacon1BeforeTrussBlue, yPosBeacon1BeforeTrussBlue, headingBeacon1Blue);
        beacon2BeforeTrussBlue = new Pose2d(xPosBeacon2BeforeTrussBlue, yPosBeacon2BeforeTrussBlue, headingBeacon2Blue);
        beacon3BeforeTrussBlue = new Pose2d(xPosBeacon3BeforeTrussBlue, yPosBeacon3BeforeTrussBlue, headingBeacon3Blue);
        beacon1LineUpBeforeTrussBlue = new Pose2d(xPosBeacon1LineUpBeforeTrussBlue, yPosBeacon1LineUpBeforeTrussBlue, headingBeacon1LineUpBeforeTrussBlue);
        beacon1KnockingLineUpBeforeTrussBlue = new Pose2d(xPosBeacon1KnockingLineUpBeforeTrussBlue, yPosBeacon1KnockingLineUpBeforeTrussBlue, headingBeacon1Blue);

        beacon1AfterTrussBlue = new Pose2d(xPosBeacon1AfterTrussBlue, yPosBeacon1AfterTrussBlue, headingBeacon1Blue);
        beacon2AfterTrussBlue = new Pose2d(xPosBeacon2AfterTrussBlue, yPosBeacon2AfterTrussBlue, headingBeacon2Blue);
        beacon3AfterTrussBlue = new Pose2d(xPosBeacon3AfterTrussBlue, yPosBeacon3AfterTrussBlue, headingBeacon3Blue);

        pickUpSpotBlue = new Pose2d(xPosPickUpSpotBlue, yPosPickUpSpotBlue, headingPickUpSpotBlue);
        stackKnockerPosBlue = new Pose2d(xPosStackKnockerPosBlue, yPosStackKnockerPosBlue, headingPickUpSpotBlue);
        pickUpPosAfterKnockedBlue = new Pose2d(xPosPickUpPosAfterKnockedBlue, yPosPickUpPosAfterKnockedBlue, headingPlaceAndPickUp);
        beforePickUpAfterKnockedBlue = new Pose2d(xPosBeforePickUpAfterKnockedBlue, yPosBeforePickUpAfterKnockedBlue, headingPickUpSpotBlue);

        lineUpForTrussBlue = new Pose2d(xPosLineUpForTrussBlue, yPosLineUpForTrussBlue, headingPlaceAndPickUp);
        afterPickUpNoPixelCrashBlue = new Pose2d(xPosAfterPickUpNoPixelCrashBlue, yPosAfterPickUpNoPixelCrashBlue, headingPlaceAndPickUp);
        lineUpPlacementBlue = new Pose2d(xPosLineUpPlacementBlue, yPosLineUpPlacementBlue, headingPlaceAndPickUp);
        lineUpForFirstPlacementAfterTrussBlue = new Pose2d(xPosLineUpForFirstPlacementAfterTrussBlue, yPosLineUpForFirstPlacementAfterTrussBlue, headingPlaceAndPickUp);

        underTrussBlue = new Pose2d(xPosUnderTrussBlue, yPosUnderTrussBlue, headingPlaceAndPickUp);
        placementBlue = new Pose2d(xPosPlacementBlue, yPosPlacementBlue, headingPlaceAndPickUp);
        placementBlueBeacon1 = new Pose2d(xPosPlacementBlueBeacon1, yPosPlacementBlueBeacon1, headingPlaceAndPickUp);
        placementBlueBeacon2 = new Pose2d(xPosPlacementBlueBeacon2, yPosPlacementBlueBeacon2, headingPlaceAndPickUp);
        placementBlueBeacon3 = new Pose2d(xPosPlacementBlueBeacon3, yPosPlacementBlueBeacon3, headingPlaceAndPickUp);
        slidesDownAfterPlaceBlue = new Pose2d(xPosSlidesDownAfterPlaceBlue, yPosSlidesDownAfterPlaceBlue, headingPlaceAndPickUp);
        underTrussGoingBackBlue = new Pose2d(xPosUnderTrussGoingBackBlue, yPosUnderTrussGoingBackBlue, headingPlaceAndPickUp);
        afterPlacePosForNoCrashBlue = new Pose2d(xPosAfterPlacePosForNoCrashBlue, yPosAfterPlacePosForNoCrashBlue, headingPlaceAndPickUp);
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
    public void blueAutonLoop(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPlacePosForNoCrashBlue, ""));
        posesToGoTo.add(new PosesAndActions(underTrussBlue, "resetSlides"));
        posesToGoTo.add(new PosesAndActions(lineUpForTrussBlue, ""));
        posesToGoTo.add(new PosesAndActions(beforePickUpAfterKnockedBlue, ""));
        follower.reinit(posesToGoTo);
        loadPixels();
        follower.goToPoints(true);

        double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+MonkeyMap.yPosBeforePickUpAfterKnockedBlue + offsetForPickUp;

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(new Pose2d(MonkeyMap.xPosPickUpPosAfterKnockedBlue, yPosAfterSeeing,MonkeyMap.headingPlaceAndPickUp), ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(beforePickUpAfterKnockedBlue, ""));
        follower.reinit(posesToGoTo);
        loadPixels();
        follower.goToPoints(true);

        yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+MonkeyMap.yPosBeforePickUpAfterKnockedBlue + offsetForPickUp;

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(new Pose2d(MonkeyMap.xPosPickUpPosAfterKnockedBlue, yPosAfterSeeing,MonkeyMap.headingPlaceAndPickUp), ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        myOpMode.sleep(MonkeyMap.sleepTimePickUpPixel);

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPickUpNoPixelCrashBlue, ""));
        posesToGoTo.add(new PosesAndActions(lineUpForTrussBlue, ""));
        posesToGoTo.add(new PosesAndActions(underTrussGoingBackBlue, "stopLoadingPixels and closeGrabber"));
        posesToGoTo.add(new PosesAndActions(underTrussBlue, "placeSlides"));
        posesToGoTo.add(new PosesAndActions(slidesDownAfterPlaceBlue, "flipDown"));
        posesToGoTo.add(new PosesAndActions(lineUpPlacementBlue, ""));
        posesToGoTo.add(new PosesAndActions(placementBlue, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(MonkeyMap.sleepTimePlacePixels);
        flipUp();
        myOpMode.sleep(MonkeyMap.sleepTimeAfterFlip);
    }

}
