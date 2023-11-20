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
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;

import java.net.PortUnreachableException;
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
    public static double stackKnockerKnockedPos = 0, stackKnockerResetPos = 0.34;

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPower = 1, stopLoadPower = 0;
    public static int resetSlidesPos = 0, placementSlidesPos = -150, slidePosFirstPlace = -150;

    public static double holdPowerForSlides = -0.1, slidePowerDown = 0.3, slidePowerUp = 0.6;

    public static int[] pixelHeightsForRotator;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;


    //Blue Poses
    public Pose2d startingPosition, beacon1Preload, beacon2Preload, beacon3Preload, pickUpSpot, placementPos, placementBeacon1, placementBeacon2, placementBeacon3, underTruss, slidesDownAfterPlace,  underTrussGoingBack, stackKnockerPos, beforePickUpAfterKnocked, afterPlacePosForNoCrash, lineUpForTruss, beacon1LineUpBeforeTruss, afterPickUpNoPixelCrash, beacon1KnockingLineUpBeforeTruss, lineUpPlacement, beacon3LineUpAfterTruss, lineUpForFirstPlacementAfterTruss, putSlidesBackDownBeforePlace;

    //All Poses
    public static double headingPlaceAndPickUp = Math.toRadians(0);
    public double xPosPickUpPosAfterKnocked;

    public static int sleepTimePlacePreloadBeacon = 400, sleepTimePickUpPixel = 0, sleepTimePlacePixels = 400, sleepTimeKnockStack = 300, sleepTimeAfterFlip = 500, sleepTimeFlipForFirstPlaceAfterTruss = 200, sleepTimePutSlidesUpNoBreakFlipper = 300;
    public boolean grabberIsOpen = true, wheelOn = false, conveyerOn = false, flipperDown = true, airplaneLoaded = true, rotatorDown = false, isKnocked = false;
    public static double timesToRunAuton = 1;
    public static double correctWallDist = 24, roomForWallDistError = 0.25;
    public static double lineDist = 20, offsetForPickUp = 0.5;
    public static double velForTurn = 25;
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

    public void initPoses(String autonType){
        if(autonType.equals("blueAfterTruss")){
            startingPosition = new Pose2d(BlueAfterTrussPoses.xPosStartingPosition, BlueAfterTrussPoses.yPosStartingPosition, BlueAfterTrussPoses.headingStartingPositionAndBeacon);

            beacon1Preload = new Pose2d(BlueAfterTrussPoses.xPosBeacon1Preload, BlueAfterTrussPoses.yPosBeacon1Preload, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon2Preload = new Pose2d(BlueAfterTrussPoses.xPosBeacon2Preload, BlueAfterTrussPoses.yPosBeacon2Preload, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon3Preload = new Pose2d(BlueAfterTrussPoses.xPosBeacon3Preload, BlueAfterTrussPoses.yPosBeacon3Preload, BlueAfterTrussPoses.headingStartingPositionAndBeacon);

            beacon1KnockingLineUpBeforeTruss = new Pose2d(BlueAfterTrussPoses.xPosBeacon1KnockingLineUpBeforeTruss, BlueAfterTrussPoses.yPosBeacon1KnockingLineUpBeforeTruss, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon1LineUpBeforeTruss = new Pose2d(BlueAfterTrussPoses.xPosBeacon1LineUpBeforeTruss, BlueAfterTrussPoses.yPosBeacon1LineUpBeforeTruss, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon3LineUpAfterTruss = new Pose2d(BlueAfterTrussPoses.xPosBeacon3LineUpAfterTruss, BlueAfterTrussPoses.yPosBeacon3LineUpAfterTruss, BlueAfterTrussPoses.headingStartingPositionAndBeacon);

            pickUpSpot = new Pose2d(BlueAfterTrussPoses.xPosPickUpSpot, BlueAfterTrussPoses.yPosPickUpSpot, headingPlaceAndPickUp);
            stackKnockerPos = new Pose2d(BlueAfterTrussPoses.xPosStackKnockerPos, BlueAfterTrussPoses.yPosStackKnockerPos, headingPlaceAndPickUp);
            beforePickUpAfterKnocked = new Pose2d(BlueAfterTrussPoses.xPosBeforePickUpAfterKnocked, BlueAfterTrussPoses.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

            lineUpForTruss = new Pose2d(BlueAfterTrussPoses.xPosLineUpForTruss, BlueAfterTrussPoses.yPosLineUpForTruss, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(BlueAfterTrussPoses.xPosAfterPickUpNoPixelCrash, BlueAfterTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
            lineUpPlacement = new Pose2d(BlueAfterTrussPoses.xPosLineUpPlacement, BlueAfterTrussPoses.yPosLineUpPlacement, headingPlaceAndPickUp);
            lineUpForFirstPlacementAfterTruss = new Pose2d(BlueAfterTrussPoses.xPosLineUpForFirstPlacementAfterTruss, BlueAfterTrussPoses.yPosLineUpForFirstPlacementAfterTruss, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(BlueAfterTrussPoses.xPosPutSlidesBackDownBeforePlace, BlueAfterTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

            underTruss = new Pose2d(BlueAfterTrussPoses.xPosUnderTruss, BlueAfterTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementPos = new Pose2d(BlueAfterTrussPoses.xPosPlacement, BlueAfterTrussPoses.yPosPlacement, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(BlueAfterTrussPoses.xPosPutSlidesBackDownBeforePlace, BlueAfterTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(BlueAfterTrussPoses.xPosAfterPickUpNoPixelCrash, BlueAfterTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

            underTruss = new Pose2d(BlueAfterTrussPoses.xPosUnderTruss, BlueAfterTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementBeacon1 = new Pose2d(BlueAfterTrussPoses.xPosPlacementBeacon1, BlueAfterTrussPoses.yPosPlacementBeacon1, headingPlaceAndPickUp);
            placementBeacon2 = new Pose2d(BlueAfterTrussPoses.xPosPlacementBeacon2, BlueAfterTrussPoses.yPosPlacementBeacon2, headingPlaceAndPickUp);
            placementBeacon3 = new Pose2d(BlueAfterTrussPoses.xPosPlacementBeacon3, BlueAfterTrussPoses.yPosPlacementBeacon3, headingPlaceAndPickUp);
            slidesDownAfterPlace = new Pose2d(BlueAfterTrussPoses.xPosSlidesDownAfterPlace, BlueAfterTrussPoses.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
            underTrussGoingBack = new Pose2d(BlueAfterTrussPoses.xPosUnderTrussGoingBack, BlueAfterTrussPoses.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
            afterPlacePosForNoCrash = new Pose2d(BlueAfterTrussPoses.xPosAfterPlacePosForNoCrash, BlueAfterTrussPoses.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);
            xPosPickUpPosAfterKnocked = BlueAfterTrussPoses.xPosPickUpPosAfterKnocked;
        }
        else if(autonType.equals("blueBeforeTruss")){
            startingPosition = new Pose2d(BlueBeforeTrussPoses.xPosStartingPosition, BlueBeforeTrussPoses.yPosStartingPosition, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);

            beacon1Preload = new Pose2d(BlueBeforeTrussPoses.xPosBeacon1Preload, BlueBeforeTrussPoses.yPosBeacon1Preload, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon2Preload = new Pose2d(BlueBeforeTrussPoses.xPosBeacon2Preload, BlueBeforeTrussPoses.yPosBeacon2Preload, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon3Preload = new Pose2d(BlueBeforeTrussPoses.xPosBeacon3Preload, BlueBeforeTrussPoses.yPosBeacon3Preload, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);

            beacon1KnockingLineUpBeforeTruss = new Pose2d(BlueBeforeTrussPoses.xPosBeacon1KnockingLineUpBeforeTruss, BlueBeforeTrussPoses.yPosBeacon1KnockingLineUpBeforeTruss, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon1LineUpBeforeTruss = new Pose2d(BlueBeforeTrussPoses.xPosBeacon1LineUpBeforeTruss, BlueBeforeTrussPoses.yPosBeacon1LineUpBeforeTruss, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon3LineUpAfterTruss = new Pose2d(BlueBeforeTrussPoses.xPosBeacon3LineUpAfterTruss, BlueBeforeTrussPoses.yPosBeacon3LineUpAfterTruss, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);

            pickUpSpot = new Pose2d(BlueBeforeTrussPoses.xPosPickUpSpot, BlueBeforeTrussPoses.yPosPickUpSpot, headingPlaceAndPickUp);
            stackKnockerPos = new Pose2d(BlueBeforeTrussPoses.xPosStackKnockerPos, BlueBeforeTrussPoses.yPosStackKnockerPos, headingPlaceAndPickUp);
            beforePickUpAfterKnocked = new Pose2d(BlueBeforeTrussPoses.xPosBeforePickUpAfterKnocked, BlueBeforeTrussPoses.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

            lineUpForTruss = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForTruss, BlueBeforeTrussPoses.yPosLineUpForTruss, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(BlueBeforeTrussPoses.xPosAfterPickUpNoPixelCrash, BlueBeforeTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
            lineUpPlacement = new Pose2d(BlueBeforeTrussPoses.xPosLineUpPlacement, BlueBeforeTrussPoses.yPosLineUpPlacement, headingPlaceAndPickUp);
            lineUpForFirstPlacementAfterTruss = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForFirstPlacementAfterTruss, BlueBeforeTrussPoses.yPosLineUpForFirstPlacementAfterTruss, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(BlueBeforeTrussPoses.xPosPutSlidesBackDownBeforePlace, BlueBeforeTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

            underTruss = new Pose2d(BlueBeforeTrussPoses.xPosUnderTruss, BlueBeforeTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementPos = new Pose2d(BlueBeforeTrussPoses.xPosPlacement, BlueBeforeTrussPoses.yPosPlacement, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(BlueBeforeTrussPoses.xPosPutSlidesBackDownBeforePlace, BlueBeforeTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(BlueBeforeTrussPoses.xPosAfterPickUpNoPixelCrash, BlueBeforeTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

            underTruss = new Pose2d(BlueBeforeTrussPoses.xPosUnderTruss, BlueBeforeTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementBeacon1 = new Pose2d(BlueBeforeTrussPoses.xPosPlacementBeacon1, BlueBeforeTrussPoses.yPosPlacementBeacon1, headingPlaceAndPickUp);
            placementBeacon2 = new Pose2d(BlueBeforeTrussPoses.xPosPlacementBeacon2, BlueBeforeTrussPoses.yPosPlacementBeacon2, headingPlaceAndPickUp);
            placementBeacon3 = new Pose2d(BlueBeforeTrussPoses.xPosPlacementBeacon3, BlueBeforeTrussPoses.yPosPlacementBeacon3, headingPlaceAndPickUp);
            slidesDownAfterPlace = new Pose2d(BlueBeforeTrussPoses.xPosSlidesDownAfterPlace, BlueBeforeTrussPoses.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
            underTrussGoingBack = new Pose2d(BlueBeforeTrussPoses.xPosUnderTrussGoingBack, BlueBeforeTrussPoses.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
            afterPlacePosForNoCrash = new Pose2d(BlueBeforeTrussPoses.xPosAfterPlacePosForNoCrash, BlueBeforeTrussPoses.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

            xPosPickUpPosAfterKnocked = BlueBeforeTrussPoses.xPosPickUpPosAfterKnocked;
        }
        else if(autonType.equals("redAfterTruss")){
            startingPosition = new Pose2d(RedAfterTrussPoses.xPosStartingPosition, RedAfterTrussPoses.yPosStartingPosition, RedAfterTrussPoses.headingStartingPositionAndBeacon);

            beacon1Preload = new Pose2d(RedAfterTrussPoses.xPosBeacon1Preload, RedAfterTrussPoses.yPosBeacon1Preload, RedAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon2Preload = new Pose2d(RedAfterTrussPoses.xPosBeacon2Preload, RedAfterTrussPoses.yPosBeacon2Preload, RedAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon3Preload = new Pose2d(RedAfterTrussPoses.xPosBeacon3Preload, RedAfterTrussPoses.yPosBeacon3Preload, RedAfterTrussPoses.headingStartingPositionAndBeacon);

            beacon1KnockingLineUpBeforeTruss = new Pose2d(RedAfterTrussPoses.xPosBeacon1KnockingLineUpBeforeTruss, RedAfterTrussPoses.yPosBeacon1KnockingLineUpBeforeTruss, RedAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon1LineUpBeforeTruss = new Pose2d(RedAfterTrussPoses.xPosBeacon1LineUpBeforeTruss, RedAfterTrussPoses.yPosBeacon1LineUpBeforeTruss, RedAfterTrussPoses.headingStartingPositionAndBeacon);
            beacon3LineUpAfterTruss = new Pose2d(RedAfterTrussPoses.xPosBeacon3LineUpAfterTruss, RedAfterTrussPoses.yPosBeacon3LineUpAfterTruss, RedAfterTrussPoses.headingStartingPositionAndBeacon);

            pickUpSpot = new Pose2d(RedAfterTrussPoses.xPosPickUpSpot, RedAfterTrussPoses.yPosPickUpSpot, headingPlaceAndPickUp);
            stackKnockerPos = new Pose2d(RedAfterTrussPoses.xPosStackKnockerPos, RedAfterTrussPoses.yPosStackKnockerPos, headingPlaceAndPickUp);
            beforePickUpAfterKnocked = new Pose2d(RedAfterTrussPoses.xPosBeforePickUpAfterKnocked, RedAfterTrussPoses.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

            lineUpForTruss = new Pose2d(RedAfterTrussPoses.xPosLineUpForTruss, RedAfterTrussPoses.yPosLineUpForTruss, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(RedAfterTrussPoses.xPosAfterPickUpNoPixelCrash, RedAfterTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
            lineUpPlacement = new Pose2d(RedAfterTrussPoses.xPosLineUpPlacement, RedAfterTrussPoses.yPosLineUpPlacement, headingPlaceAndPickUp);
            lineUpForFirstPlacementAfterTruss = new Pose2d(RedAfterTrussPoses.xPosLineUpForFirstPlacementAfterTruss, RedAfterTrussPoses.yPosLineUpForFirstPlacementAfterTruss, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(RedAfterTrussPoses.xPosPutSlidesBackDownBeforePlace, RedAfterTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

            underTruss = new Pose2d(RedAfterTrussPoses.xPosUnderTruss, RedAfterTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementPos = new Pose2d(RedAfterTrussPoses.xPosPlacement, RedAfterTrussPoses.yPosPlacement, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(RedAfterTrussPoses.xPosPutSlidesBackDownBeforePlace, RedAfterTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(RedAfterTrussPoses.xPosAfterPickUpNoPixelCrash, RedAfterTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

            underTruss = new Pose2d(RedAfterTrussPoses.xPosUnderTruss, RedAfterTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementBeacon1 = new Pose2d(RedAfterTrussPoses.xPosPlacementBeacon1, RedAfterTrussPoses.yPosPlacementBeacon1, headingPlaceAndPickUp);
            placementBeacon2 = new Pose2d(RedAfterTrussPoses.xPosPlacementBeacon2, RedAfterTrussPoses.yPosPlacementBeacon2, headingPlaceAndPickUp);
            placementBeacon3 = new Pose2d(RedAfterTrussPoses.xPosPlacementBeacon3, RedAfterTrussPoses.yPosPlacementBeacon3, headingPlaceAndPickUp);
            slidesDownAfterPlace = new Pose2d(RedAfterTrussPoses.xPosSlidesDownAfterPlace, RedAfterTrussPoses.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
            underTrussGoingBack = new Pose2d(RedAfterTrussPoses.xPosUnderTrussGoingBack, RedAfterTrussPoses.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
            afterPlacePosForNoCrash = new Pose2d(RedAfterTrussPoses.xPosAfterPlacePosForNoCrash, RedAfterTrussPoses.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

            xPosPickUpPosAfterKnocked = RedAfterTrussPoses.xPosPickUpPosAfterKnocked;
        }
        else if(autonType.equals("redBeforeTruss")){
            startingPosition = new Pose2d(RedBeforeTrussPoses.xPosStartingPosition, RedBeforeTrussPoses.yPosStartingPosition, RedBeforeTrussPoses.headingStartingPositionAndBeacon);

            beacon1Preload = new Pose2d(RedBeforeTrussPoses.xPosBeacon1Preload, RedBeforeTrussPoses.yPosBeacon1Preload, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon2Preload = new Pose2d(RedBeforeTrussPoses.xPosBeacon2Preload, RedBeforeTrussPoses.yPosBeacon2Preload, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon3Preload = new Pose2d(RedBeforeTrussPoses.xPosBeacon3Preload, RedBeforeTrussPoses.yPosBeacon3Preload, RedBeforeTrussPoses.headingStartingPositionAndBeacon);

            beacon1KnockingLineUpBeforeTruss = new Pose2d(RedBeforeTrussPoses.xPosBeacon1KnockingLineUpBeforeTruss, RedBeforeTrussPoses.yPosBeacon1KnockingLineUpBeforeTruss, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon1LineUpBeforeTruss = new Pose2d(RedBeforeTrussPoses.xPosBeacon1LineUpBeforeTruss, RedBeforeTrussPoses.yPosBeacon1LineUpBeforeTruss, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
            beacon3LineUpAfterTruss = new Pose2d(RedBeforeTrussPoses.xPosBeacon3LineUpAfterTruss, RedBeforeTrussPoses.yPosBeacon3LineUpAfterTruss, RedBeforeTrussPoses.headingStartingPositionAndBeacon);

            pickUpSpot = new Pose2d(RedBeforeTrussPoses.xPosPickUpSpot, RedBeforeTrussPoses.yPosPickUpSpot, headingPlaceAndPickUp);
            stackKnockerPos = new Pose2d(RedBeforeTrussPoses.xPosStackKnockerPos, RedBeforeTrussPoses.yPosStackKnockerPos, headingPlaceAndPickUp);
            beforePickUpAfterKnocked = new Pose2d(RedBeforeTrussPoses.xPosBeforePickUpAfterKnocked, RedBeforeTrussPoses.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

            lineUpForTruss = new Pose2d(RedBeforeTrussPoses.xPosLineUpForTruss, RedBeforeTrussPoses.yPosLineUpForTruss, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(RedBeforeTrussPoses.xPosAfterPickUpNoPixelCrash, RedBeforeTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
            lineUpPlacement = new Pose2d(RedBeforeTrussPoses.xPosLineUpPlacement, RedBeforeTrussPoses.yPosLineUpPlacement, headingPlaceAndPickUp);
            lineUpForFirstPlacementAfterTruss = new Pose2d(RedBeforeTrussPoses.xPosLineUpForFirstPlacementAfterTruss, RedBeforeTrussPoses.yPosLineUpForFirstPlacementAfterTruss, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(RedBeforeTrussPoses.xPosPutSlidesBackDownBeforePlace, RedBeforeTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

            underTruss = new Pose2d(RedBeforeTrussPoses.xPosUnderTruss, RedBeforeTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementPos = new Pose2d(RedBeforeTrussPoses.xPosPlacement, RedBeforeTrussPoses.yPosPlacement, headingPlaceAndPickUp);
            putSlidesBackDownBeforePlace = new Pose2d(RedBeforeTrussPoses.xPosPutSlidesBackDownBeforePlace, RedBeforeTrussPoses.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
            afterPickUpNoPixelCrash = new Pose2d(RedBeforeTrussPoses.xPosAfterPickUpNoPixelCrash, RedBeforeTrussPoses.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

            underTruss = new Pose2d(RedBeforeTrussPoses.xPosUnderTruss, RedBeforeTrussPoses.yPosUnderTruss, headingPlaceAndPickUp);
            placementBeacon1 = new Pose2d(RedBeforeTrussPoses.xPosPlacementBeacon1, RedBeforeTrussPoses.yPosPlacementBeacon1, headingPlaceAndPickUp);
            placementBeacon2 = new Pose2d(RedBeforeTrussPoses.xPosPlacementBeacon2, RedBeforeTrussPoses.yPosPlacementBeacon2, headingPlaceAndPickUp);
            placementBeacon3 = new Pose2d(RedBeforeTrussPoses.xPosPlacementBeacon3, RedBeforeTrussPoses.yPosPlacementBeacon3, headingPlaceAndPickUp);
            slidesDownAfterPlace = new Pose2d(RedBeforeTrussPoses.xPosSlidesDownAfterPlace, RedBeforeTrussPoses.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
            underTrussGoingBack = new Pose2d(RedBeforeTrussPoses.xPosUnderTrussGoingBack, RedBeforeTrussPoses.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
            afterPlacePosForNoCrash = new Pose2d(RedBeforeTrussPoses.xPosAfterPlacePosForNoCrash, RedBeforeTrussPoses.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

            xPosPickUpPosAfterKnocked = RedBeforeTrussPoses.xPosPickUpPosAfterKnocked;
        }
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
    public void placeInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose, boolean isFirstTime){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPickUpNoPixelCrash, ""));
        posesToGoTo.add(new PosesAndActions(lineUpForTruss, ""));
        posesToGoTo.add(new PosesAndActions(underTrussGoingBack, "stopLoadingPixels and closeGrabber"));
        posesToGoTo.add(new PosesAndActions(underTruss, "placeSlides"));
        posesToGoTo.add(new PosesAndActions(slidesDownAfterPlace, "flipDown"));
        posesToGoTo.add(new PosesAndActions(lineUpPlacement, ""));
        if(isFirstTime){
            posesToGoTo.add(new PosesAndActions(putSlidesBackDownBeforePlace, "resetSlides"));
        }
        posesToGoTo.add(new PosesAndActions(finalPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(MonkeyMap.sleepTimePlacePixels);
        if(isFirstTime){
            flipUp();
            myOpMode.sleep(sleepTimeFlipForFirstPlaceAfterTruss);
            placeSlides();
            myOpMode.sleep(sleepTimePutSlidesUpNoBreakFlipper);
        }
        else{
            flipUp();
            myOpMode.sleep(MonkeyMap.sleepTimeAfterFlip);
        }

    }
    public void goToPickUpInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPlacePosForNoCrash, ""));
        posesToGoTo.add(new PosesAndActions(underTruss, "resetSlides"));
        posesToGoTo.add(new PosesAndActions(lineUpForTruss, ""));
        posesToGoTo.add(new PosesAndActions(finalPose, "loadPixels"));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
    }
    public void autonVisionPickUp(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
        double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+ beforePickUpAfterKnocked.getY() + offsetForPickUp;
        posesToGoTo.add(new PosesAndActions(new Pose2d(xPosPickUpPosAfterKnocked, yPosAfterSeeing, MonkeyMap.headingPlaceAndPickUp), ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        myOpMode.sleep(MonkeyMap.sleepTimePickUpPixel);
    }
    public void autonLoop(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, boolean isFirstTime){
        Pose2d finalPose = new Pose2d();
        finalPose = beforePickUpAfterKnocked;
        goToPickUpInAuton(follower, posesToGoTo, finalPose);
        autonVisionPickUp(follower, posesToGoTo);

        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(beforePickUpAfterKnocked, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        autonVisionPickUp(follower, posesToGoTo);

        finalPose = placementPos;

        placeInAuton(follower, posesToGoTo, finalPose, isFirstTime);
    }

    public int TeamPropDetectionReadingBeforeTruss(){
        if(!OpenCVDetectTeamProp.isDetected){
            return 3;
        }
        else if(OpenCVDetectTeamProp.centerX < 160){
            return 2;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            return 1;
        }
        return 0;
    }
    public int TeamPropDetectionReadingAfterTruss(){
        if(!OpenCVDetectTeamProp.isDetected){
            return 3;
        }
        else if(OpenCVDetectTeamProp.centerX < 160){
            return 2;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            return 1;
        }
        return 0;
    }
}
