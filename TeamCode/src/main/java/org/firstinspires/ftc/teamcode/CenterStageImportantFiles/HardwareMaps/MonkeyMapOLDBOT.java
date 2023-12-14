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

import java.util.ArrayList;

//@Config
public class MonkeyMapOLDBOT {
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
    public static double grabberClosed = 0.36, grabberOpen = 0.2, grabberServoOpenPlacement = 0.2;
    public static double wheelServoPow = 1, servoStopPow = 0.5;
    public static double flipperScalarDown = 0.1, flipperScalarUp = 0.9, flipperScalarOffset = 0.05, flipperPosDown = 0.17, flipperPosAcross = 0.98, rotatorPickUp = 0.1835, rotatorPlace = 0.85;//0.6665 is rot dist
    public static double airplaneServoLoadedPos = 0.29, airplaneServoGoPos = 0.16;
    public static double stackKnockerKnockedPos = 0, stackKnockerResetPos = 0.34;

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPowerForAuton = 0.55, stopLoadPower = 0, unloadPower = 1;
    public static int resetSlidesPos = 0, placementSlidesPos = -150, slidePosFirstPlace = -150;

    public static double holdPowerForSlides = -0.1, slidePowerDown = 0.3, slidePowerUp = 0.6;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;


    //Blue Poses
    public Pose2d startingPosition, beacon1Preload, beacon2Preload, beacon3Preload, pickUpSpot, placementPos, placementBeacon1, placementBeacon2, placementBeacon3, underTruss, slidesDownAfterPlace,  underTrussGoingBack, stackKnockerPos, beforePickUpAfterKnocked, afterPlacePosForNoCrash, lineUpForTruss, beacon1LineUpBeforeTruss, afterPickUpNoPixelCrash, beacon1KnockingLineUpBeforeTruss, lineUpPlacement, beacon3LineUpAfterTruss, lineUpForFirstPlacementAfterTruss, putSlidesBackDownBeforePlace, lineUpPlacementBeacon2;

    //All Poses
    public static double headingPlaceAndPickUp = Math.toRadians(0);
    public double xPosPickUpPosAfterKnocked;

    public static int sleepTimePlacePreloadBeacon = 700, sleepTimePickUpPixel = 0, sleepTimePlacePixels = 400, sleepTimeKnockStack = 300, sleepTimeAfterFlip = 500, sleepTimeFlipForFirstPlaceAfterTruss = 200, sleepTimePutSlidesUpNoBreakFlipper = 300;
    public boolean grabberIsOpen = true, wheelOn = false, conveyerOn = false, flipperDown = true, airplaneLoaded = true, rotatorDown = false, isKnocked = false;
    public static double timesToRunAuton = 1;
    public static double lineDist = 20, offsetForPickUp = 0.5;
    public static double velForTurn = 25;
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public MonkeyMapOLDBOT(LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public MonkeyMapOLDBOT(OpMode opmode) {
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
        switch (autonType) {
            case "blueAfterTruss":
                startingPosition = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosStartingPosition, BlueAfterTrussPosesOLDBOT.yPosStartingPosition, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1Preload = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon1Preload, BlueAfterTrussPosesOLDBOT.yPosBeacon1Preload, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon2Preload = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon2Preload, BlueAfterTrussPosesOLDBOT.yPosBeacon2Preload, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3Preload = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon3Preload, BlueAfterTrussPosesOLDBOT.yPosBeacon3Preload, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1KnockingLineUpBeforeTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon1KnockingLineUpBeforeTruss, BlueAfterTrussPosesOLDBOT.yPosBeacon1KnockingLineUpBeforeTruss, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon1LineUpBeforeTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon1LineUpBeforeTruss, BlueAfterTrussPosesOLDBOT.yPosBeacon1LineUpBeforeTruss, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3LineUpAfterTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeacon3LineUpAfterTruss, BlueAfterTrussPosesOLDBOT.yPosBeacon3LineUpAfterTruss, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                lineUpPlacementBeacon2 = new Pose2d(BlueAfterTrussPosesOLDBOT.xPoslineUpPlacementBeacon2, BlueAfterTrussPosesOLDBOT.yPoslineUpPlacementBeacon2, headingPlaceAndPickUp);

                pickUpSpot = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPickUpSpot, BlueAfterTrussPosesOLDBOT.yPosPickUpSpot, headingPlaceAndPickUp);
                stackKnockerPos = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosStackKnockerPos, BlueAfterTrussPosesOLDBOT.yPosStackKnockerPos, headingPlaceAndPickUp);
                beforePickUpAfterKnocked = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosBeforePickUpAfterKnocked, BlueAfterTrussPosesOLDBOT.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

                lineUpForTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosLineUpForTruss, BlueAfterTrussPosesOLDBOT.yPosLineUpForTruss, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, BlueAfterTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
                lineUpPlacement = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosLineUpPlacement, BlueAfterTrussPosesOLDBOT.yPosLineUpPlacement, headingPlaceAndPickUp);
                lineUpForFirstPlacementAfterTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosLineUpForFirstPlacementAfterTruss, BlueAfterTrussPosesOLDBOT.yPosLineUpForFirstPlacementAfterTruss, BlueAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                putSlidesBackDownBeforePlace = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, BlueAfterTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

                underTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosUnderTruss, BlueAfterTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementPos = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPlacement, BlueAfterTrussPosesOLDBOT.yPosPlacement, headingPlaceAndPickUp);
                putSlidesBackDownBeforePlace = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, BlueAfterTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, BlueAfterTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

                underTruss = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosUnderTruss, BlueAfterTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementBeacon1 = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPlacementBeacon1, BlueAfterTrussPosesOLDBOT.yPosPlacementBeacon1, headingPlaceAndPickUp);
                placementBeacon2 = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPlacementBeacon2, BlueAfterTrussPosesOLDBOT.yPosPlacementBeacon2, headingPlaceAndPickUp);
                placementBeacon3 = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosPlacementBeacon3, BlueAfterTrussPosesOLDBOT.yPosPlacementBeacon3, headingPlaceAndPickUp);
                slidesDownAfterPlace = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosSlidesDownAfterPlace, BlueAfterTrussPosesOLDBOT.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
                underTrussGoingBack = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosUnderTrussGoingBack, BlueAfterTrussPosesOLDBOT.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
                afterPlacePosForNoCrash = new Pose2d(BlueAfterTrussPosesOLDBOT.xPosAfterPlacePosForNoCrash, BlueAfterTrussPosesOLDBOT.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);
                xPosPickUpPosAfterKnocked = BlueAfterTrussPosesOLDBOT.xPosPickUpPosAfterKnocked;
                break;
            case "blueBeforeTruss":
                startingPosition = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosStartingPosition, BlueBeforeTrussPosesOLDBOT.yPosStartingPosition, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1Preload = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon1Preload, BlueBeforeTrussPosesOLDBOT.yPosBeacon1Preload, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon2Preload = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon2Preload, BlueBeforeTrussPosesOLDBOT.yPosBeacon2Preload, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3Preload = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon3Preload, BlueBeforeTrussPosesOLDBOT.yPosBeacon3Preload, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1KnockingLineUpBeforeTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon1KnockingLineUpBeforeTruss, BlueBeforeTrussPosesOLDBOT.yPosBeacon1KnockingLineUpBeforeTruss, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon1LineUpBeforeTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon1LineUpBeforeTruss, BlueBeforeTrussPosesOLDBOT.yPosBeacon1LineUpBeforeTruss, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3LineUpAfterTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeacon3LineUpAfterTruss, BlueBeforeTrussPosesOLDBOT.yPosBeacon3LineUpAfterTruss, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                pickUpSpot = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPickUpSpot, BlueBeforeTrussPosesOLDBOT.yPosPickUpSpot, headingPlaceAndPickUp);
                stackKnockerPos = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosStackKnockerPos, BlueBeforeTrussPosesOLDBOT.yPosStackKnockerPos, headingPlaceAndPickUp);
                beforePickUpAfterKnocked = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosBeforePickUpAfterKnocked, BlueBeforeTrussPosesOLDBOT.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

                lineUpForTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosLineUpForTruss, BlueBeforeTrussPosesOLDBOT.yPosLineUpForTruss, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, BlueBeforeTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
                lineUpPlacement = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosLineUpPlacement, BlueBeforeTrussPosesOLDBOT.yPosLineUpPlacement, headingPlaceAndPickUp);
                lineUpForFirstPlacementAfterTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosLineUpForFirstPlacementAfterTruss, BlueBeforeTrussPosesOLDBOT.yPosLineUpForFirstPlacementAfterTruss, BlueBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                putSlidesBackDownBeforePlace = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, BlueBeforeTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

                underTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosUnderTruss, BlueBeforeTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementPos = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPlacement, BlueBeforeTrussPosesOLDBOT.yPosPlacement, headingPlaceAndPickUp);
                putSlidesBackDownBeforePlace = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, BlueBeforeTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, BlueBeforeTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

                underTruss = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosUnderTruss, BlueBeforeTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementBeacon1 = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPlacementBeacon1, BlueBeforeTrussPosesOLDBOT.yPosPlacementBeacon1, headingPlaceAndPickUp);
                placementBeacon2 = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPlacementBeacon2, BlueBeforeTrussPosesOLDBOT.yPosPlacementBeacon2, headingPlaceAndPickUp);
                placementBeacon3 = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosPlacementBeacon3, BlueBeforeTrussPosesOLDBOT.yPosPlacementBeacon3, headingPlaceAndPickUp);
                slidesDownAfterPlace = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosSlidesDownAfterPlace, BlueBeforeTrussPosesOLDBOT.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
                underTrussGoingBack = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosUnderTrussGoingBack, BlueBeforeTrussPosesOLDBOT.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
                afterPlacePosForNoCrash = new Pose2d(BlueBeforeTrussPosesOLDBOT.xPosAfterPlacePosForNoCrash, BlueBeforeTrussPosesOLDBOT.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

                xPosPickUpPosAfterKnocked = BlueBeforeTrussPosesOLDBOT.xPosPickUpPosAfterKnocked;
                break;
            case "redAfterTruss":
                startingPosition = new Pose2d(RedAfterTrussPosesOLDBOT.xPosStartingPosition, RedAfterTrussPosesOLDBOT.yPosStartingPosition, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1Preload = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon1Preload, RedAfterTrussPosesOLDBOT.yPosBeacon1Preload, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon2Preload = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon2Preload, RedAfterTrussPosesOLDBOT.yPosBeacon2Preload, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3Preload = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon3Preload, RedAfterTrussPosesOLDBOT.yPosBeacon3Preload, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1KnockingLineUpBeforeTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon1KnockingLineUpBeforeTruss, RedAfterTrussPosesOLDBOT.yPosBeacon1KnockingLineUpBeforeTruss, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon1LineUpBeforeTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon1LineUpBeforeTruss, RedAfterTrussPosesOLDBOT.yPosBeacon1LineUpBeforeTruss, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3LineUpAfterTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeacon3LineUpAfterTruss, RedAfterTrussPosesOLDBOT.yPosBeacon3LineUpAfterTruss, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                lineUpPlacementBeacon2 = new Pose2d(RedAfterTrussPosesOLDBOT.xPosLineUpPlacementBeacon2, RedAfterTrussPosesOLDBOT.yPosLineUpPlacementBeacon2, headingPlaceAndPickUp);

                pickUpSpot = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPickUpSpot, RedAfterTrussPosesOLDBOT.yPosPickUpSpot, headingPlaceAndPickUp);
                stackKnockerPos = new Pose2d(RedAfterTrussPosesOLDBOT.xPosStackKnockerPos, RedAfterTrussPosesOLDBOT.yPosStackKnockerPos, headingPlaceAndPickUp);
                beforePickUpAfterKnocked = new Pose2d(RedAfterTrussPosesOLDBOT.xPosBeforePickUpAfterKnocked, RedAfterTrussPosesOLDBOT.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

                lineUpForTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosLineUpForTruss, RedAfterTrussPosesOLDBOT.yPosLineUpForTruss, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(RedAfterTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, RedAfterTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
                lineUpPlacement = new Pose2d(RedAfterTrussPosesOLDBOT.xPosLineUpPlacement, RedAfterTrussPosesOLDBOT.yPosLineUpPlacement, headingPlaceAndPickUp);
                lineUpForFirstPlacementAfterTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosLineUpForFirstPlacementAfterTruss, RedAfterTrussPosesOLDBOT.yPosLineUpForFirstPlacementAfterTruss, RedAfterTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                putSlidesBackDownBeforePlace = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, RedAfterTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

                underTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosUnderTruss, RedAfterTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementPos = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPlacement, RedAfterTrussPosesOLDBOT.yPosPlacement, headingPlaceAndPickUp);
                putSlidesBackDownBeforePlace = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, RedAfterTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(RedAfterTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, RedAfterTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

                underTruss = new Pose2d(RedAfterTrussPosesOLDBOT.xPosUnderTruss, RedAfterTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementBeacon1 = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPlacementBeacon1, RedAfterTrussPosesOLDBOT.yPosPlacementBeacon1, headingPlaceAndPickUp);
                placementBeacon2 = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPlacementBeacon2, RedAfterTrussPosesOLDBOT.yPosPlacementBeacon2, headingPlaceAndPickUp);
                placementBeacon3 = new Pose2d(RedAfterTrussPosesOLDBOT.xPosPlacementBeacon3, RedAfterTrussPosesOLDBOT.yPosPlacementBeacon3, headingPlaceAndPickUp);
                slidesDownAfterPlace = new Pose2d(RedAfterTrussPosesOLDBOT.xPosSlidesDownAfterPlace, RedAfterTrussPosesOLDBOT.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
                underTrussGoingBack = new Pose2d(RedAfterTrussPosesOLDBOT.xPosUnderTrussGoingBack, RedAfterTrussPosesOLDBOT.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
                afterPlacePosForNoCrash = new Pose2d(RedAfterTrussPosesOLDBOT.xPosAfterPlacePosForNoCrash, RedAfterTrussPosesOLDBOT.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

                xPosPickUpPosAfterKnocked = RedAfterTrussPosesOLDBOT.xPosPickUpPosAfterKnocked;
                break;
            case "redBeforeTruss":
                startingPosition = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosStartingPosition, RedBeforeTrussPosesOLDBOT.yPosStartingPosition, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1Preload = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon1Preload, RedBeforeTrussPosesOLDBOT.yPosBeacon1Preload, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon2Preload = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon2Preload, RedBeforeTrussPosesOLDBOT.yPosBeacon2Preload, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3Preload = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon3Preload, RedBeforeTrussPosesOLDBOT.yPosBeacon3Preload, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                beacon1KnockingLineUpBeforeTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon1KnockingLineUpBeforeTruss, RedBeforeTrussPosesOLDBOT.yPosBeacon1KnockingLineUpBeforeTruss, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon1LineUpBeforeTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon1LineUpBeforeTruss, RedBeforeTrussPosesOLDBOT.yPosBeacon1LineUpBeforeTruss, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                beacon3LineUpAfterTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeacon3LineUpAfterTruss, RedBeforeTrussPosesOLDBOT.yPosBeacon3LineUpAfterTruss, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);

                pickUpSpot = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPickUpSpot, RedBeforeTrussPosesOLDBOT.yPosPickUpSpot, headingPlaceAndPickUp);
                stackKnockerPos = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosStackKnockerPos, RedBeforeTrussPosesOLDBOT.yPosStackKnockerPos, headingPlaceAndPickUp);
                beforePickUpAfterKnocked = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosBeforePickUpAfterKnocked, RedBeforeTrussPosesOLDBOT.yPosBeforePickUpAfterKnocked, headingPlaceAndPickUp);

                lineUpForTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosLineUpForTruss, RedBeforeTrussPosesOLDBOT.yPosLineUpForTruss, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, RedBeforeTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);
                lineUpPlacement = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosLineUpPlacement, RedBeforeTrussPosesOLDBOT.yPosLineUpPlacement, headingPlaceAndPickUp);
                lineUpForFirstPlacementAfterTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosLineUpForFirstPlacementAfterTruss, RedBeforeTrussPosesOLDBOT.yPosLineUpForFirstPlacementAfterTruss, RedBeforeTrussPosesOLDBOT.headingStartingPositionAndBeacon);
                putSlidesBackDownBeforePlace = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, RedBeforeTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);

                underTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosUnderTruss, RedBeforeTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementPos = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPlacement, RedBeforeTrussPosesOLDBOT.yPosPlacement, headingPlaceAndPickUp);
                putSlidesBackDownBeforePlace = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPutSlidesBackDownBeforePlace, RedBeforeTrussPosesOLDBOT.yPosPutSlidesBackDownBeforePlace, headingPlaceAndPickUp);
                afterPickUpNoPixelCrash = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosAfterPickUpNoPixelCrash, RedBeforeTrussPosesOLDBOT.yPosAfterPickUpNoPixelCrash, headingPlaceAndPickUp);

                underTruss = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosUnderTruss, RedBeforeTrussPosesOLDBOT.yPosUnderTruss, headingPlaceAndPickUp);
                placementBeacon1 = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPlacementBeacon1, RedBeforeTrussPosesOLDBOT.yPosPlacementBeacon1, headingPlaceAndPickUp);
                placementBeacon2 = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPlacementBeacon2, RedBeforeTrussPosesOLDBOT.yPosPlacementBeacon2, headingPlaceAndPickUp);
                placementBeacon3 = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosPlacementBeacon3, RedBeforeTrussPosesOLDBOT.yPosPlacementBeacon3, headingPlaceAndPickUp);
                slidesDownAfterPlace = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosSlidesDownAfterPlace, RedBeforeTrussPosesOLDBOT.yPosSlidesDownAfterPlace, headingPlaceAndPickUp);
                underTrussGoingBack = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosUnderTrussGoingBack, RedBeforeTrussPosesOLDBOT.yPosUnderTrussGoingBack, headingPlaceAndPickUp);
                afterPlacePosForNoCrash = new Pose2d(RedBeforeTrussPosesOLDBOT.xPosAfterPlacePosForNoCrash, RedBeforeTrussPosesOLDBOT.yPosAfterPlacePosForNoCrash, headingPlaceAndPickUp);

                xPosPickUpPosAfterKnocked = RedBeforeTrussPosesOLDBOT.xPosPickUpPosAfterKnocked;
                break;
        }
    }

    public void openGrabber(){
        if(flipperDown){
            grabberServo.setPosition(grabberServoOpenPlacement);
        }
        else{
            grabberServo.setPosition(grabberOpen);
        }
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
    public void flipDownAndRotate(){
        setFlipperPos(flipperPosDown);
//        rotateUp();
        flipperDown = true;
    }
    public void flipUpAndRotate(){
        setFlipperPos(flipperPosAcross);
//        rotateDown();
        flipperDown = false;
    }
//    public void flipDown(){
//        setFlipperPos(flipperPosDown);
//        flipperDown = true;
//    }
//    public void flipUp(){
//        setFlipperPos(flipperPosAcross);
//        flipperDown = false;
//    }

    public void toggleFlipper() {
        if (flipperDown) {
            flipUpAndRotate();
        } else {
            flipDownAndRotate();
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
        conveyerMotor.setPower(unloadPowerForAuton);
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
        flipDownAndRotate();
        closeGrabber();
    }
    public void readyPickUpAuton(){
        resetSlides();
        openGrabber();
        flipUpAndRotate();
    }
    public void getReadyFirstPlaceAuton(){
        encodedSlipperySlides(slidePosFirstPlace, slidePowerUp);
        flipDownAndRotate();
        closeGrabber();
    }
    public void placeInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose, boolean isFirstTime){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPickUpNoPixelCrash, ""));
        posesToGoTo.add(new PosesAndActions(lineUpForTruss, ""));
        posesToGoTo.add(new PosesAndActions(underTrussGoingBack, "unloadPixel and closeGrabber"));
        posesToGoTo.add(new PosesAndActions(underTruss, "stopLoadingPixels and placeSlides"));
        posesToGoTo.add(new PosesAndActions(slidesDownAfterPlace, "flipDown"));
        posesToGoTo.add(new PosesAndActions(lineUpPlacement, ""));
        if(isFirstTime){
            posesToGoTo.add(new PosesAndActions(putSlidesBackDownBeforePlace, "resetSlides"));
        }
        posesToGoTo.add(new PosesAndActions(finalPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(MonkeyMapOLDBOT.sleepTimePlacePixels);
        if(isFirstTime){
            flipUpAndRotate();
            myOpMode.sleep(sleepTimeFlipForFirstPlaceAfterTruss);
            placeSlides();
            myOpMode.sleep(sleepTimePutSlidesUpNoBreakFlipper);
        }
        else{
            flipUpAndRotate();
            myOpMode.sleep(MonkeyMapOLDBOT.sleepTimeAfterFlip);
        }

    }
    public void goToPickUpInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPlacePosForNoCrash, ""));
        posesToGoTo.add(new PosesAndActions(underTruss, "resetSlides"));
        posesToGoTo.add(new PosesAndActions(lineUpForTruss, "loadPixels"));
        posesToGoTo.add(new PosesAndActions(finalPose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
    }
    public void autonVisionPickUp(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
        posesToGoTo.clear();
        double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+ beforePickUpAfterKnocked.getY() + offsetForPickUp;
        posesToGoTo.add(new PosesAndActions(new Pose2d(xPosPickUpPosAfterKnocked, yPosAfterSeeing, MonkeyMapOLDBOT.headingPlaceAndPickUp), ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        myOpMode.sleep(MonkeyMapOLDBOT.sleepTimePickUpPixel);
    }
    public void autonLoop(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, boolean isFirstTime){
        posesToGoTo.clear();
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
            return 1;
        }
        else if(OpenCVDetectTeamProp.centerX < 160){
            return 2;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            return  3;
        }
        return 0;
    }
    public int TeamPropDetectionReadingAfterTruss(){
        if(!OpenCVDetectTeamProp.isDetected){
            return 1;
        }
        else if(OpenCVDetectTeamProp.centerX < 160){
            return 2;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            return  3;
        }
        return 0;
    }
}
