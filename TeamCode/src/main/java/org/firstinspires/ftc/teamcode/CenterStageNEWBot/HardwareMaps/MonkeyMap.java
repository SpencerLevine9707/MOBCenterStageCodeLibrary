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
    public DcMotor frontLeft, frontRight, backLeft, backRight, pullUpMotorLeft, pullUpMotorRight, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServoLeft, grabberServoRight, rotatorServo, flipperServoLeft, flipperServoRight, airplaneServo, correctorServo;

    public VoltageSensor batteryVoltageSensor;
    public DistanceSensor lineUpSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0.1, grabberServoScalerUp = 0.5, offsetForGrabberScalar = 0.03;
    public static double grabberClosed = 0.94, grabberOpen = 0;
    public static double flipperScalarDown = 0, flipperScalarUp = 1, flipperScalarOffset = 0, flipperPosDown = 0.15, flipperPosDown2Pixels = 0.83, flipperPosDown3Pixels = 0.82, flipperPosDown4Pixels = 0.81, flipperPosDown5Pixels = 0.7825, flipperPosDown6Pixels = 0.7725, flipperPosUp = 0.82, flipperPosUpPurplePixels = 0.83, flipperPosUpFirstPlace = 0, flipperPosDownForAuton = 0.15;
    public static double rotatorServoUpPos = 0.22, rotatorFlushWithSlides = 0.5, rotatorPickUpAndPlace = 0.45, rotator6Pixels = 0.517, rotator5Pixels = 0.517, rotator4Pixels = 0.51, rotator3Pixels = 0.485, rotator2Pixels = 0.5, rotatorServoFirstPlace = 0.6, rotatorServoPlaceInAuton = 0.58;
    public static double airplaneServoLoadedPos = 0.29, airplaneServoGoPos = 0.16;
    public static double correctorServoMidPos = 0.5, correctorServoPlaceFarPos = 0.35, correctorServoPlaceClosePos = 0.65, correctorServoBeacon2AfterPos = 0.07, correctorServoBeacon1PreloadPlace = 0.45, correctorServoBeacon3PreloadPlace = 0.55, correctorServoPickUpClose = 0.58, correctorServoPickUpMidFar = 0.38, correctorServoPickUpMidClose = 0.78;
    public static double correctorServoSpeed = 0.03, flipperServoSpeed = 0.03, rotatorServoSpeed = 0.03;
    public ArrayList<String> rotatorAndFlipperAutonPosesListOddPixels = new ArrayList<>(Arrays.asList("flipDown and rotateDown 5Pixels", "flipDown and rotateDown 3Pixels"));
    public ArrayList<String> rotatorAndFlipperAutonPosesListEvenPixels = new ArrayList<>(Arrays.asList("flipDown and rotateDown 4Pixels", "flipDown and rotateDown 2Pixels"));

    //Motor powers and pos
    public static int resetSlidesPos = 0, slidesFullyExtendedPos = -314;
    public static int slidesBeacon1PreloadAfter = 0, slidesBeacon2PreloadAfter = -310, slidesBeacon3PreloadAfter = -310;
    public static int slidesBeacon1PreloadBefore = -314, slidesBeacon2PreloadBefore = -250, slidesBeacon3PreloadBefore = -200;
    public static int slidesFirstPlacePosBeacon2 = 0, slidesFirstPlacePosBeacons13 = 0;

    public static double holdPowerForSlides = 0, slidePowerEncoder = 1;


    //Poses
    public Pose2d startingPosition;
    public Pose2d purplePixelPlacementAfterFarAndCloseBeacon1, purplePixelPlacementAfterFarAndCloseBeacon23, purplePixelPlacementAfterMidBeacon, firstPlacementBeacon1After, firstPlacementBeacon2After, firstPlacementBeacon3After, startExtendFirstPlacementAfter;
    public Pose2d firstPlacementBeacon1BeforeClose, firstPlacementBeacon2BeforeClose, firstPlacementBeacon3BeforeClose, firstPlacementBeacon1BeforeFar, firstPlacementBeacon2BeforeFar, firstPlacementBeacon3BeforeFar;
    public Pose2d lineUpForPickUpFar, startArmExtendPickUpFar, pickUpPixelFar,  placePixelFar, flipAfterPlaceFar, startArmExtendPlaceFar;
    public Pose2d lineUpForPickUpClose, startArmExtendPickUpClose, pickUpPixelClose, startArmExtendPlaceClose, placePixelClose, flipAfterPlaceClose, goStraightThroughTrussClose;
    public Pose2d goAcrossForBeforeTrussPurplePixelFar, goAcrossForBeforeTrussPurplePixelCloseMidBeacon, goAcrossForBeforeTrussPurplePixelCloseWallBeacon, goAcrossForBeforeTrussPurplePixelCloseTrussBeacon;
    public Pose2d midStackPickUpFar, midStackPickUpClose;
    public static double headingPlaceAndPickUp = Math.toRadians(180);

    public static int sleepTimePlacePurplePixel = 300, sleepTimePickUpPixel = 400, sleepTimePlacePixel = 400, sleepTimeFirstPlace = 100, sleepTimeBetweenFirstAndSecondPlace = 100, sleepTimeExtendSlides = 400, sleepTimeWaitForFlipFirstPlace = 200, sleepTimeWaitForFlip = 200, sleepTimeCorrectServo = 200;
    public boolean grabberIsOpen = true, rightGrabberOpen = true, leftGrabberOpen = true, flipperDown = true, airplaneLoaded = true;
    public static int timesToRunAuton = 4;
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
        pullUpMotorLeft = myOpMode.hardwareMap.get(DcMotor.class, "pullUpMotorLeft");
        pullUpMotorRight = myOpMode.hardwareMap.get(DcMotor.class, "pullUpMotorLeft");
        armMotorLeft = myOpMode.hardwareMap.get(DcMotor.class, "armMotorLeft");
        armMotorRight = myOpMode.hardwareMap.get(DcMotor.class, "armMotorRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUpMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUpMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    public void resetSlidePoses(){
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void initForAuton(String autonType){
        initPoses(autonType);
        flipUp();
        closeGrabber();
        setRotatorUp();
        setCorrectorMid();
        resetSlidePoses();
    }

    public void initPoses(String autonType){
        switch (autonType) {
            case "blueAfterTruss":
                startingPosition = new Pose2d(BlueAfterTrussPoses.xPosStartingPos, BlueAfterTrussPoses.yPosStartingPos, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPlaceAndPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPlaceAndPickUp);

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
                startArmExtendPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceFar, BlueAfterTrussPoses.yPosStartArmExtendPlaceFar, headingPlaceAndPickUp);
                lineUpForPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpClose, BlueAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPickUpClose, BlueAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelClose, BlueAfterTrussPoses.yPosPickUpPixelClose, BlueAfterTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceClose, BlueAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosFlipAfterPlaceClose, BlueAfterTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueAfterTrussPoses.xPosGoStraightThroughTrussClose, BlueAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, headingPlaceAndPickUp);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosMidStackPickUpFar, BlueAfterTrussPoses.yPosMidStackPickUpFar, BlueAfterTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosMidStackPickUpClose, BlueAfterTrussPoses.yPosMidStackPickUpClose, BlueAfterTrussPoses.headingMidStackPickUpClose);
                break;

            case "blueBeforeTruss":
                startingPosition = new Pose2d(BlueBeforeTrussPoses.xPosStartingPos, BlueBeforeTrussPoses.yPosStartingPos, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPlaceAndPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPlaceAndPickUp);

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
                lineUpForPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpFar, BlueBeforeTrussPoses.yPosLineUpForPickUpFar, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpFar, BlueBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelFar, BlueBeforeTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceFar, BlueBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceFar, BlueBeforeTrussPoses.yPosStartArmExtendPlaceFar, headingPlaceAndPickUp);
                lineUpForPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpClose, BlueBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpClose, BlueBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelClose, BlueBeforeTrussPoses.yPosPickUpPixelClose, BlueBeforeTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceClose, BlueBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceClose, BlueBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueBeforeTrussPoses.xPosGoStraightThroughTrussClose, BlueBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, headingPlaceAndPickUp);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpFar, RedAfterTrussPoses.yPosMidStackPickUpFar, RedAfterTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpClose, RedAfterTrussPoses.yPosMidStackPickUpClose, RedAfterTrussPoses.headingMidStackPickUpClose);
                break;

            case "redAfterTruss":
                startingPosition = new Pose2d(RedAfterTrussPoses.xPosStartingPos, RedAfterTrussPoses.yPosStartingPos, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPlaceAndPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPlaceAndPickUp);

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
                lineUpForPickUpFar = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpFar, RedAfterTrussPoses.yPosLineUpForPickUpFar, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpFar, RedAfterTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelFar, RedAfterTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceFar, RedAfterTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceFar, RedAfterTrussPoses.yPosStartArmExtendPlaceFar,headingPlaceAndPickUp);
                lineUpForPickUpClose = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpClose, RedAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpClose, RedAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelClose, RedAfterTrussPoses.yPosPickUpPixelClose, RedAfterTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceClose, RedAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceClose, RedAfterTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(RedAfterTrussPoses.xPosGoStraightThroughTrussClose, RedAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, headingPlaceAndPickUp);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpFar, RedAfterTrussPoses.yPosMidStackPickUpFar, RedAfterTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpClose, RedAfterTrussPoses.yPosMidStackPickUpClose, RedAfterTrussPoses.headingMidStackPickUpClose);
                break;

            case "redBeforeTruss":
                startingPosition = new Pose2d(RedBeforeTrussPoses.xPosStartingPos, RedBeforeTrussPoses.yPosStartingPos, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPlaceAndPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPlaceAndPickUp);

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
                lineUpForPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpFar, RedBeforeTrussPoses.yPosLineUpForPickUpFar, headingPlaceAndPickUp);
                startArmExtendPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpFar, RedBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPlaceAndPickUp);
                pickUpPixelFar = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelFar, RedBeforeTrussPoses.yPosPickUpPixelFar, headingPlaceAndPickUp);
                placePixelFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceFar, RedBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPlaceAndPickUp);
                startArmExtendPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceFar, RedBeforeTrussPoses.yPosStartArmExtendPlaceFar, headingPlaceAndPickUp);
                lineUpForPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpClose, RedBeforeTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
                startArmExtendPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpClose, RedBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPlaceAndPickUp);
                pickUpPixelClose = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelClose, RedBeforeTrussPoses.yPosPickUpPixelClose, RedBeforeTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceClose, RedBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPlaceAndPickUp);
                placePixelClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceClose, RedBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPlaceAndPickUp);
                goStraightThroughTrussClose = new Pose2d(RedBeforeTrussPoses.xPosGoStraightThroughTrussClose, RedBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPlaceAndPickUp);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosMidStackPickUpFar, RedBeforeTrussPoses.yPosMidStackPickUpFar, RedBeforeTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosMidStackPickUpClose, RedBeforeTrussPoses.yPosMidStackPickUpClose, RedBeforeTrussPoses.headingMidStackPickUpClose);
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

    public void resetArm(){
        setFlipperPos(flipperPosDown6Pixels);
//        closeGrabber();
        setRotatorFlush();
        setCorrectorMid();
        resetSlides();
    }
    public void setFlipperPos(double pos){
        flipperServoLeft.setPosition(pos);
        flipperServoRight.setPosition(pos);
    }
    public void flipDown(){
        setFlipperPos(flipperPosDown);
        flipperDown = true;
    }
    public void flipDownForAuton(){
        setFlipperPos(flipperPosDownForAuton);
        flipperDown = true;
    }
    public void flipAndRotateDownAndExtend6Pixels(){
        setFlipperPos(flipperPosDown6Pixels);
        rotatorServo.setPosition(rotator6Pixels);
        fullyExtendSlides();
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
//    public void unloadPixel(){
//        conveyerMotor.setPower(unloadPowerForAuton);
//    }
//    public void stopLoadingPixels(){
//        conveyerMotor.setPower(stopLoadPower);
//    }
//    public void loadPixels(){
//        conveyerMotor.setPower(conveyerPower);
//    }
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
        encodedSlipperySlides(slidesBeacon2PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesFarBeaconAfter(){
        encodedSlipperySlides(slidesBeacon3PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesCloseBeaconAfter(){
        encodedSlipperySlides(slidesBeacon1PreloadAfter, slidePowerEncoder);
    }
    public void extendSlidesMidBeaconBefore(){
        encodedSlipperySlides(slidesBeacon2PreloadBefore, slidePowerEncoder);
    }
    public void extendSlidesWallBeaconBefore(){
        encodedSlipperySlides(slidesBeacon3PreloadBefore, slidePowerEncoder);
    }
    public void extendSlidesTrussBeaconBefore(){
        encodedSlipperySlides(slidesBeacon1PreloadBefore, slidePowerEncoder);
    }
    public void placeInAutonFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, boolean isBlue){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, "flipDownForAuton"));
//        posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace"));
        if(isBlue){
            posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace"));
        }
        else{
            posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, "fullyExtendSlides and setCorrectorPlaceClose and rotateForPlace"));
        }
        posesToGoTo.add(new PosesAndActions(placePixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(sleepTimeFirstPlace);
        closeGrabber();
        myOpMode.sleep(sleepTimeBetweenFirstAndSecondPlace);
        openGrabber();
        myOpMode.sleep(sleepTimePlacePixel);
        resetArm();
    }
    public void pickUpInAutonFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isOddPixels, boolean goToMid){
        posesToGoTo.clear();
        if(isOddPixels){
            posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, rotatorAndFlipperAutonPosesListOddPixels.get(pixelType)));
        }
        else{
            posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, rotatorAndFlipperAutonPosesListEvenPixels.get(pixelType)));
        }
//        posesToGoTo.add(new PosesAndActions(lineUpForPickUpFar, ""));
        posesToGoTo.add(new PosesAndActions(startArmExtendPickUpFar, "fullyExtendSlides and openGrabber"));
        if(goToMid){
            posesToGoTo.add(new PosesAndActions(pickUpPixelFar, "setCorrectorPickUpCloseMid"));
        }
        else{
            posesToGoTo.add(new PosesAndActions(pickUpPixelFar, ""));
        }
        if(goToMid){
            posesToGoTo.add(new PosesAndActions(midStackPickUpFar, ""));
        }
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        closeGrabber();
        myOpMode.sleep(sleepTimePickUpPixel);
        resetArm();
    }
    public void placeInAutonClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, boolean isBlue){
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(goStraightThroughTrussClose, ""));
        posesToGoTo.add(new PosesAndActions(lineUpForPickUpClose, "flipDownForAuton"));
        if(isBlue){
            posesToGoTo.add(new PosesAndActions(startArmExtendPlaceClose, "fullyExtendSlides and setCorrectorPlaceClose and rotateForPlace"));
        }
        else{
            posesToGoTo.add(new PosesAndActions(startArmExtendPlaceClose, "fullyExtendSlides and setCorrectorPlaceFar and rotateForPlace"));
        }

        posesToGoTo.add(new PosesAndActions(placePixelClose, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        openGrabber();
        myOpMode.sleep(sleepTimeFirstPlace);
        closeGrabber();
        myOpMode.sleep(sleepTimeBetweenFirstAndSecondPlace);
        openGrabber();
        myOpMode.sleep(sleepTimePlacePixel);
        resetArm();
    }
    public void pickUpInAutonClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isOddPixels, boolean goToMid){
        posesToGoTo.clear();
        if(isOddPixels){
            posesToGoTo.add(new PosesAndActions(lineUpForPickUpClose, rotatorAndFlipperAutonPosesListOddPixels.get(pixelType)));
        }
        else{
            posesToGoTo.add(new PosesAndActions(lineUpForPickUpClose, rotatorAndFlipperAutonPosesListEvenPixels.get(pixelType)));
        }
//        posesToGoTo.add(new PosesAndActions(goStraightThroughTrussClose, "fullyExtendSlides and openGrabber and setCorrectorPickUpClose"));
        posesToGoTo.add(new PosesAndActions(goStraightThroughTrussClose, "fullyExtendSlides and openGrabber"));
        if(goToMid){
            posesToGoTo.add(new PosesAndActions(pickUpPixelClose, "setCorrectorPickUpCloseMid"));
        }
        else{
            posesToGoTo.add(new PosesAndActions(pickUpPixelClose, ""));
        }
        if(goToMid){
            posesToGoTo.add(new PosesAndActions(midStackPickUpClose, ""));
        }
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        correctorServo.setPosition(correctorServoPickUpClose);
        myOpMode.sleep(sleepTimeCorrectServo);
        closeGrabber();
        myOpMode.sleep(sleepTimePickUpPixel);
        resetArm();
//        resetSlides();
//        flipDownForAuton();
//        correctorServo.setPosition(MonkeyMap.correctorServoPlaceClosePos);
//        rotatorServo.setPosition(MonkeyMap.rotatorServoPlaceInAuton);
    }
    public void autonLoopFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isOddPixels, boolean goToMid, boolean isBlue){
        posesToGoTo.clear();
        pickUpInAutonFar(follower, posesToGoTo, pixelType, isOddPixels, goToMid);
        placeInAutonFar(follower, posesToGoTo, isBlue);
    }
    public void autonLoopClose(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isOddPixels, boolean goToMid, boolean isBlue){
        posesToGoTo.clear();
        pickUpInAutonClose(follower, posesToGoTo, pixelType, isOddPixels, goToMid);
        placeInAutonClose(follower, posesToGoTo, isBlue);
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
