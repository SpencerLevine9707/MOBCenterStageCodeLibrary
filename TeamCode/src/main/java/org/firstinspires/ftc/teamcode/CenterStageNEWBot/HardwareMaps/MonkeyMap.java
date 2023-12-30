package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight, conveyerMotor, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServoLeft, grabberServoRight, rotatorServo, flipperServoLeft, flipperServoRight, airplaneServo, correctorServo;

    public VoltageSensor batteryVoltageSensor;
    public DistanceSensor lineUpSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0.1, grabberServoScalerUp = 0.5, offsetForGrabberScalar = -0.02;
    public static double grabberClosed = 0.94, grabberOpen = 0.2;
    public static double flipperScalarDown = 0, flipperScalarUp = 1, flipperScalarOffset = 0, flipperPosDown = 0.15, flipperPosDown2Pixels = 0.175, flipperPosDown3Pixels = 0.2, flipperPosDown4Pixels = 0.225, flipperPosDown5Pixels = 0.25, flipperPosUp = 0.8, flipperPosUpFirstPlace = 0.9;
    public static double rotatorServoUpPos = 0.22, rotatorFlushWithSlides = 0.5, rotatorPickUpAndPlace = 0.45, rotator5Pixels = 0.35, rotator4Pixels = 0.375, rotator3Pixels = 0.4, rotator2Pixels = 0.425;
    public static double airplaneServoLoadedPos = 0.29, airplaneServoGoPos = 0.16;
    public static double correctorServoMidPos = 0.5;
    public static double correctorServoSpeed = 0.003, flipperServoSpeed = 0.003, rotatorServoSpeed = 0.003;
    public ArrayList<String> rotatorAndFlipperAutonPosesListAfter = new ArrayList<>(Arrays.asList("flipDown and rotateDown 5Pixels", "flipDown and rotateDown 3Pixels"));
    public ArrayList<String> rotatorAndFlipperAutonPosesListBefore = new ArrayList<>(Arrays.asList("flipDown and rotateDown 4Pixels", "flipDown and rotateDown 2Pixels"));

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPowerForAuton = 0.55, stopLoadPower = 0, unloadPower = 1;
    public static int resetSlidesPos = 0, slidesFullyExtendedPos = -500;
    public static int slidesBeacon1PreloadAfter = -300, slidesBeacon2PreloadAfter = -250, slidesBeacon3PreloadAfter = -300;
    public static int slidesBeacon1PreloadBefore = -500, slidesBeacon2PreloadBefore = -450, slidesBeacon3PreloadBefore = -400;
    public static int slidesFarBeaconAfterPos, slidesMidBeaconAfterPos, slidesCloseBeaconAfterPos, slidesWallBeaconBeforePos, slidesMidBeaconBeforePos, slidesTrussBeaconBeforePos;

    public static double holdPowerForSlides = 0, slidePowerEncoder = 1;


    //Poses
    public Pose2d startingPosition;
    public Pose2d purplePixelPlacementAfterFarAndCloseBeacons, purplePixelPlacementAfterMidBeacon, firstPlacementBeacon1After, firstPlacementBeacon2After, firstPlacementBeacon3After, startExtendFirstPlacementAfter;
    public Pose2d firstPlacementBeacon1BeforeClose, firstPlacementBeacon2BeforeClose, firstPlacementBeacon3BeforeClose, firstPlacementBeacon1BeforeFar, firstPlacementBeacon2BeforeFar, firstPlacementBeacon3BeforeFar;
    public Pose2d lineUpForPickUpFar, startArmExtendPickUpFar, pickUpPixelFar,  placePixelFar, flipAfterPlaceFar, startArmExtendPlaceFar;
    public Pose2d lineUpForPickUpClose, startArmExtendPickUpClose, pickUpPixelClose, startArmExtendPlaceClose, placePixelClose, flipAfterPlaceClose, goStraightThroughTrussClose;
    public Pose2d goAcrossForBeforeTrussPurplePixelFar, goAcrossForBeforeTrussPurplePixelCloseMidBeacon, goAcrossForBeforeTrussPurplePixelCloseWallBeacon, goAcrossForBeforeTrussPurplePixelCloseTrussBeacon;
    public static double headingPlaceAndPickUp = Math.toRadians(180);

    public static int sleepTimePlacePurplePixel = 300, sleepTimePickUpPixel = 50, sleepTimePlacePixel = 400;
    public boolean grabberIsOpen = true, rightGrabberOpen = true, leftGrabberOpen = true, flipperDown = true, airplaneLoaded = true;
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public MonkeyMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        grabberServoLeft = myOpMode.hardwareMap.get(Servo.class, "grabberServoLeft");
        grabberServoRight = myOpMode.hardwareMap.get(Servo.class, "grabberServoRight");
        rotatorServo = myOpMode.hardwareMap.get(Servo.class, "rotatorServo");
        flipperServoLeft = myOpMode.hardwareMap.get(Servo.class, "flipperServoLeft");
        flipperServoRight = myOpMode.hardwareMap.get(Servo.class, "flipperServoRight");
        airplaneServo = myOpMode.hardwareMap.get(Servo.class, "airplaneServo");
        correctorServo = myOpMode.hardwareMap.get(Servo.class, "correctorServo");

        flipperServoRight.setDirection(Servo.Direction.REVERSE);
        flipperServoLeft.scaleRange(flipperScalarDown, flipperScalarUp);
        flipperServoRight.scaleRange(flipperScalarDown + flipperScalarOffset, flipperScalarUp + flipperScalarOffset);
//        rotatorServo.scaleRange(rotatorScalarDown, rotatorScalarUp);

        grabberServoRight.setDirection(Servo.Direction.REVERSE);

        grabberServoLeft.scaleRange(grabberServoScalerDown, grabberServoScalerUp);
        grabberServoRight.scaleRange(grabberServoScalerDown + offsetForGrabberScalar, grabberServoScalerUp + offsetForGrabberScalar);



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
    public void initForAuton(String autonType){
        initPoses(autonType);
        flipUp();
        openGrabber();
        setRotatorUp();
        setCorrectorMid();
    }

    public void initPoses(String autonType){
        switch (autonType) {
            case "blueAfterTruss":
                startingPosition = new Pose2d(BlueAfterTrussPoses.xPosStartingPos, BlueAfterTrussPoses.yPosStartingPos, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacons = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfter, BlueAfterTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                purplePixelPlacementAfterMidBeacon = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfter, BlueAfterTrussPoses.yPosPurplePixelPlacementAfter, BlueAfterTrussPoses.headingTiltedBeaconsAfter);

                firstPlacementBeacon1After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfter, BlueAfterTrussPoses.headingBeacon1PlacementAfter);
                firstPlacementBeacon2After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
                firstPlacementBeacon3After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfter, BlueAfterTrussPoses.headingBeacon3PlacementAfter);

                firstPlacementBeacon1BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosStartExtendFirstPlacementAfter, BlueAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlaceAndPickUp);
                lineUpForPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpFar, BlueAfterTrussPoses.yPosLineUpForPickUpFar, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPickUpFar, BlueAfterTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelFar, BlueAfterTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosFlipAfterPlaceFar, BlueAfterTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceFar, BlueAfterTrussPoses.yPosStartArmExtendPlaceFar, BlueAfterTrussPoses.headingPlaceClose);
                lineUpForPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpClose, BlueAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPickUpClose, BlueAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelClose, BlueAfterTrussPoses.yPosPickUpPixelClose, BlueAfterTrussPoses.headingPlaceFar);
                startArmExtendPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceClose, BlueAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosFlipAfterPlaceClose, BlueAfterTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueAfterTrussPoses.xPosGoStraightThroughTrussClose, BlueAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.headingTrussBeaconBefore);
                break;

            case "blueBeforeTruss":
                startingPosition = new Pose2d(BlueBeforeTrussPoses.xPosStartingPos, BlueBeforeTrussPoses.yPosStartingPos, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacons = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfter, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                purplePixelPlacementAfterMidBeacon = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfter, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfter, BlueBeforeTrussPoses.headingTiltedBeaconsAfter);

                firstPlacementBeacon1After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfter, BlueBeforeTrussPoses.headingBeacon1PlacementAfter);
                firstPlacementBeacon2After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
                firstPlacementBeacon3After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfter, BlueBeforeTrussPoses.headingBeacon3PlacementAfter);

                firstPlacementBeacon1BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(BlueBeforeTrussPoses.xPosStartExtendFirstPlacementAfter, BlueBeforeTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlaceAndPickUp);
                lineUpForPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpFar, BlueBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpFar, BlueBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelFar, BlueBeforeTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceFar, BlueBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceFar, BlueBeforeTrussPoses.yPosStartArmExtendPlaceFar, BlueBeforeTrussPoses.headingPlaceClose);
                lineUpForPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpClose, BlueBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpClose, BlueBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelClose, BlueBeforeTrussPoses.yPosPickUpPixelClose, BlueBeforeTrussPoses.headingPlaceFar);
                startArmExtendPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceClose, BlueBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceClose, BlueBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueBeforeTrussPoses.xPosGoStraightThroughTrussClose, BlueBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingTrussBeaconBefore);
                break;

            case "redAfterTruss":
                startingPosition = new Pose2d(RedAfterTrussPoses.xPosStartingPos, RedAfterTrussPoses.yPosStartingPos, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacons = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfter, RedAfterTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                purplePixelPlacementAfterMidBeacon = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfter, RedAfterTrussPoses.yPosPurplePixelPlacementAfter, RedAfterTrussPoses.headingTiltedBeaconsAfter);

                firstPlacementBeacon1After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfter, RedAfterTrussPoses.headingBeacon1PlacementAfter);
                firstPlacementBeacon2After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
                firstPlacementBeacon3After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfter, RedAfterTrussPoses.headingBeacon3PlacementAfter);

                firstPlacementBeacon1BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosStartExtendFirstPlacementAfter, RedAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlaceAndPickUp);
                lineUpForPickUpFar = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpFar, RedAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpFar, RedAfterTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelFar, RedAfterTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceFar, RedAfterTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceFar, RedAfterTrussPoses.yPosStartArmExtendPlaceFar, RedAfterTrussPoses.headingPlaceClose);
                lineUpForPickUpClose = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpClose, RedAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpClose, RedAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelClose, RedAfterTrussPoses.yPosPickUpPixelClose, RedAfterTrussPoses.headingPlaceFar);
                startArmExtendPlaceClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceClose, RedAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceClose, RedAfterTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(RedAfterTrussPoses.xPosGoStraightThroughTrussClose, RedAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.headingTrussBeaconBefore);
                break;

            case "redBeforeTruss":
                startingPosition = new Pose2d(RedBeforeTrussPoses.xPosStartingPos, RedBeforeTrussPoses.yPosStartingPos, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacons = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfter, RedBeforeTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                purplePixelPlacementAfterMidBeacon = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfter, RedBeforeTrussPoses.yPosPurplePixelPlacementAfter, RedBeforeTrussPoses.headingTiltedBeaconsAfter);

                firstPlacementBeacon1After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfter, RedBeforeTrussPoses.headingBeacon1PlacementAfter);
                firstPlacementBeacon2After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
                firstPlacementBeacon3After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfter, RedBeforeTrussPoses.headingBeacon3PlacementAfter);

                firstPlacementBeacon1BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(RedBeforeTrussPoses.xPosStartExtendFirstPlacementAfter, RedBeforeTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlaceAndPickUp);
                lineUpForPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpFar, RedBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpFar, RedBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelFar, RedBeforeTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceFar, RedBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceFar, RedBeforeTrussPoses.yPosStartArmExtendPlaceFar, RedBeforeTrussPoses.headingPlaceClose);
                lineUpForPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpClose, RedBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpClose, RedBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelClose, RedBeforeTrussPoses.yPosPickUpPixelClose, RedBeforeTrussPoses.headingPlaceFar);
                startArmExtendPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceClose, RedBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceClose, RedBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(RedBeforeTrussPoses.xPosGoStraightThroughTrussClose, RedBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingTrussBeaconBefore);
                break;
        }
    }

    public void openRightGrabber(){
        grabberServoRight.setPosition(grabberOpen);
        rightGrabberOpen = true;
    }
    public void openLeftGrabber(){
        grabberServoLeft.setPosition(grabberOpen);
        leftGrabberOpen = true;
    }
    public void openGrabber(){
        openLeftGrabber();
        openRightGrabber();
        grabberIsOpen = true;
    }

    public void closeRightGrabber(){
        grabberServoRight.setPosition(grabberClosed);
        rightGrabberOpen = false;
    }
    public void closeLeftGrabber(){
        grabberServoLeft.setPosition(grabberClosed);
        leftGrabberOpen = false;
    }
    public void closeGrabber(){
        closeLeftGrabber();
        closeRightGrabber();
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

    public void toggleLeftGrabber(){
        if(leftGrabberOpen){
            closeLeftGrabber();
        }
        else{
            openLeftGrabber();
        }
    }

    public void toggleRightGrabber(){
        if(rightGrabberOpen){
            closeRightGrabber();
        }
        else{
            openRightGrabber();
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
    public void flipAndRotateDown5Pixels(){
        setFlipperPos(flipperPosDown5Pixels);
        rotatorServo.setPosition(rotator5Pixels);
    }
    public void flipAndRotateDown4Pixels(){
        setFlipperPos(flipperPosDown4Pixels);
        rotatorServo.setPosition(rotator4Pixels);
    }
    public void flipAndRotateDown3Pixels(){
        setFlipperPos(flipperPosDown3Pixels);
        rotatorServo.setPosition(rotator3Pixels);
    }
    public void flipAndRotateDown2Pixels(){
        setFlipperPos(flipperPosDown2Pixels);
        rotatorServo.setPosition(rotator2Pixels);
    }
    public void flipUp(){
        setFlipperPos(flipperPosUp);
        flipperDown = false;
    }
    public void flipUpFirstPlace(){
        setFlipperPos(flipperPosUpFirstPlace);
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
    public void setRotatorUp(){
        rotatorServo.setPosition(rotatorServoUpPos);
    }
    public void setRotatorFlush(){rotatorServo.setPosition(rotatorFlushWithSlides);}
    public void rotatorPickUpAndPlace(){rotatorServo.setPosition(rotatorPickUpAndPlace);}
    public void setCorrectorMid(){
        correctorServo.setPosition(correctorServoMidPos);
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
        encodedSlipperySlides(resetSlidesPos, slidePowerEncoder);
    }
    public void fullyExtendSlides(){
        encodedSlipperySlides(slidesFullyExtendedPos, slidePowerEncoder);
    }
    public void extendSlidesBeacon1PreloadAfter(){
        encodedSlipperySlides(slidesBeacon1PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesBeacon2PreloadAfter(){
        encodedSlipperySlides(slidesBeacon2PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesBeacon3PreloadAfter(){
        encodedSlipperySlides(slidesBeacon3PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesBeacon1PreloadBefore(){
        encodedSlipperySlides(slidesBeacon1PreloadBefore, slidePowerEncoder);
    }
    public void extendSlidesBeacon2PreloadBefore(){
        encodedSlipperySlides(slidesBeacon2PreloadBefore, slidePowerEncoder);
    }
    public void extendSlidesBeacon3PreloadBefore(){
        encodedSlipperySlides(slidesBeacon3PreloadBefore, slidePowerEncoder);
    }
    public void extendSlidesMidBeaconAfter(){
        encodedSlipperySlides(slidesMidBeaconAfterPos, slidePowerEncoder);
    }
    public void extendSlidesFarBeaconAfter(){
        encodedSlipperySlides(slidesFarBeaconAfterPos, slidePowerEncoder);
    }
    public void extendSlidesCloseBeaconAfter(){
        encodedSlipperySlides(slidesCloseBeaconAfterPos, slidePowerEncoder);
    }
    public void extendSlidesMidBeaconBefore(){
        encodedSlipperySlides(slidesMidBeaconBeforePos, slidePowerEncoder);
    }
    public void extendSlidesWallBeaconBefore(){
        encodedSlipperySlides(slidesWallBeaconBeforePos, slidePowerEncoder);
    }
    public void extendSlidesTrussBeaconBefore(){
        encodedSlipperySlides(slidesTrussBeaconBeforePos, slidePowerEncoder);
    }
    public void placeInAutonFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, "flipUp"));
        posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, "fullyExtendSlides"));
        posesToGoTo.add(new PosesAndActions(placePixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(sleepTimePlacePixel);
        resetSlides();
    }
    public void pickUpInAutonFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isAfter){
        posesToGoTo.clear();
        if(isAfter){
            posesToGoTo.add(new PosesAndActions(flipAfterPlaceFar, rotatorAndFlipperAutonPosesListAfter.get(pixelType)));
        }
        else{
            posesToGoTo.add(new PosesAndActions(flipAfterPlaceFar, rotatorAndFlipperAutonPosesListBefore.get(pixelType)));
        }
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, ""));
        posesToGoTo.add(new PosesAndActions(startArmExtendPickUpFar, "fullyExtendSlides and openGrabber"));
        posesToGoTo.add(new PosesAndActions(pickUpPixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        closeGrabber();
        myOpMode.sleep(sleepTimePickUpPixel);
        resetSlides();
    }
    public void placeInAutonClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpClose, "flipUp"));
        posesToGoTo.add(new PosesAndActions(startArmExtendPlaceClose, "fullyExtendSlides"));
        posesToGoTo.add(new PosesAndActions(placePixelClose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(sleepTimePlacePixel);
    }
    public void pickUpInAutonClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isAfter){
        posesToGoTo.clear();
        if(isAfter){
            posesToGoTo.add(new PosesAndActions(flipAfterPlaceClose, rotatorAndFlipperAutonPosesListAfter.get(pixelType)));
        }
        else{
            posesToGoTo.add(new PosesAndActions(flipAfterPlaceClose, rotatorAndFlipperAutonPosesListBefore.get(pixelType)));
        }
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpClose, ""));
        posesToGoTo.add(new PosesAndActions(startArmExtendPickUpClose, "fullyExtendSlides"));
        posesToGoTo.add(new PosesAndActions(goStraightThroughTrussClose, "openGrabber"));
        posesToGoTo.add(new PosesAndActions(pickUpPixelClose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        closeGrabber();
        myOpMode.sleep(sleepTimePickUpPixel);
        resetSlides();
    }
    public void autonLoopFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isAfter){
        posesToGoTo.clear();
        pickUpInAutonFar(follower, posesToGoTo, pixelType, isAfter);
        placeInAutonFar(follower, posesToGoTo);
    }
    public void autonLoopClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isAfter){
        posesToGoTo.clear();
        pickUpInAutonClose(follower, posesToGoTo, pixelType, isAfter);
        placeInAutonClose(follower, posesToGoTo);
    }
    public int TeamPropDetectionReading(){
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
    public int wrapPixelTypeInt(int pixelType){
        while(pixelType > 1){
            pixelType-=2;
        }
        return pixelType;
    }
}
