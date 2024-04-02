package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;

    //Define all hardware
    public DcMotorEx frontLeft, frontRight, backLeft, backRight, pullUpMotorLeft, pullUpMotorRight, armMotorLeft, flipperMotor;
    public static double currentAlertArmMotor = 6.25;

    public Servo stackSetter, grabberServoLeft, grabberServoRight, rotatorServo, airplaneServo, correctorServo, pullUpServoLeft, pullUpServoRight;
    public VoltageSensor batteryVoltageSensor;
//    public DistanceSensor lineUpSensor;
    public ColorRangeSensor detectionLeftGrabber, detectionRightGrabber;
//    public ModernRoboticsI2cRangeSensor relocalizer;
    public static double distForPickUpDetect = 0.35;

    //Servo Positions
    public static double grabberServoScalerDown = 0.1, grabberServoScalerUp = 0.7, offsetForGrabberScalar = 0.025;
    public static double grabberClosed = 0.72, grabberOpen = 0, grabberOpenTele = 0.3;
    public static double grabberOpenFlipperUp = 0.4;
    public static double stackSetterUpPos = 0.55, stackSetterPickUpTeleopPos = 0.51, stackSetter5PixelsPos = 0.35;

    //5Pixels 180 at 2cm, 172 at 3 cm
    public static int flipperPosDown = 1, flipperPosDown2Pixels = 1, flipperPosDown3Pixels = 160, flipperPosDown4Pixels = 1, flipperPosDown5Pixels = 170, flipperPosDown6Pixels = 300, flipperPosUp = 100, flipperPosUpPurplePixels = 100, flipperPosUpFirstPlace = 285, flipperPosUpFirstPlaceAuton = 1700, flipperPosUpPlaceInAuton = 400, flipperPosFirstPlaceOtherAlliance = 350, flipperPosDownForAuton = 100, flipperMotorPickUpTeleop = 35;
    public static double rotatorServoUpPos = 0.26, rotatorFlushWithSlides = 0.54, rotatorPickUpAndPlace = 0.45, rotator6Pixels = 0.52, rotator5Pixels = 0.52, rotator4Pixels = 0.52, rotator3Pixels = 0.52, rotator2Pixels = 0.52, rotatorServoFirstPlace = 0.6, rotatorServoPlaceInAuton = 0.58;
    public static double airplaneServoLoadedPos = 0.29, airplaneServoGoPos = 0.05, airplaneAngle = 0.495;
    public static double correctorServoMidPos = 0.47, correctorServoPlaceFarPos = 0.38, correctorServoPlaceClosePos = 0.62, correctorServoBeacon2AfterPos = 0.04, correctorServoBeacon2BeforePos = 0.9, correctorServoBeacon1PreloadPlace = 0.45, correctorServoBeacon3PreloadPlace = 0.55, correctorServoPickUpClose = 0.58, correctorServoPickUpMidFar = 0.38, correctorServoPickUpMidClose = 0.78;
    public static double correctorServoSpeed = 0.03, rotatorServoSpeed = 0.03;
    public static int flipperMotorSpeed = 250, slideSpeed = -300;
    public static int flipperMotorTolerance = 1000, armMotorTolerance = 100;

    //OG: 10, 0.05, 0, 0
    public static PIDFCoefficients PIDFFlipperMotor = new PIDFCoefficients(10, 0.05, 0, 0);
    public ArrayList<String> rotatorAndFlipperAutonPosesListOddPixels = new ArrayList<>(Arrays.asList("flipDown and rotateDown 5Pixels", "flipDown and rotateDown 3Pixels"));
    public ArrayList<String> rotatorAndFlipperAutonPosesListEvenPixels = new ArrayList<>(Arrays.asList("flipDown and rotateDown 4Pixels", "flipDown and rotateDown 2Pixels"));

    //Auto adjust place and stuff
//    public static double maxRotatorPosUp = 0.644, maxRotatorPosDown = 0.37;
    public static double maxRotatorPosUp = 0.62, maxRotatorPosDown = 0.4;
    public static double minRotatorPosPickUp = 0.52, maxRotatorPosPickUp = 0.54;
    public static int lowestFlipperPosDown = 200, highestFlipperPos = 1400;
    public static double correctorServoMinPosition = 0.04, correctorServoMaxPosition = 0.9;
    public double rotatorRange = maxRotatorPosUp - maxRotatorPosDown;
    public double rotatorRangeSlidesDown = maxRotatorPosPickUp - minRotatorPosPickUp;
    public double correctorRange = MonkeyMap.correctorServoMaxPosition - MonkeyMap.correctorServoMinPosition;

    //Pull Up Servo Poses
    public static double pullUpServoDownPos = 0.69, pullUpServoUpPos = 0.29;

    //Motor powers and pos
    public static int resetSlidesPos = 0, slidesFullyExtendedPos = -660, slidesParkPos = -50, extendTeleopPos = 0, slidesPlaceInAuton = -400;
    public static double inchesPerSlideEncoderTic = 23.5/660;
    public static int slidesBeacon1PreloadAfter = 0, slidesBeacon2PreloadAfter = -250, slidesBeacon3PreloadAfter = -430;
    public static int slidesBeacon1PreloadBefore = -310, slidesBeacon2PreloadBefore = -350, slidesBeacon3PreloadBefore = -350, slidesBeacon2PreloadBeforeBlue = -310;
    public static int slidesFirstPlacePos = -160, slidesFirstPlacePosOtherAlliance = -200;

    public static double holdPowerForSlides = 0, slidePowerEncoder = 1, flipperPower = 1, flipperPowerAuton = 1;
    public int slidesPosWhenPickUp = 0;


    //Poses
    public Pose2d startingPosition, afterPlacePixelFar;
    public Pose2d purplePixelPlacementAfterFarAndCloseBeacon1, purplePixelPlacementAfterFarAndCloseBeacon23, lineUpPurplePixelAfterTrussBeacon23, lineUpPurplePixelAfterTrussBeacon1, firstPlacementBeacon1After, firstPlacementBeacon2After, firstPlacementBeacon3After, startExtendFirstPlacementAfter, turnForFirstPlacementAfter;
    public Pose2d firstPlacementBeacon1BeforeClose, firstPlacementBeacon2BeforeClose, firstPlacementBeacon3BeforeClose, firstPlacementBeacon1BeforeFar, firstPlacementBeacon2BeforeFar, firstPlacementBeacon3BeforeFar;
    public Pose2d lineUpForPickUpFar, lineUpForPlaceFar, startArmExtendPickUpFar, pickUpPixelFar, turnAfterPickUpPixelFar,  placePixelFar, flipAfterPlaceFar, startArmExtendPlaceFar, turnAfterPlacePixel, lineUpTurnAfterPlacePixel;
    public Pose2d lineUpForPickUpClose, startArmExtendPickUpClose, pickUpPixelClose, startArmExtendPlaceClose, placePixelClose, flipAfterPlaceClose, goStraightThroughTrussClose;
    public Pose2d goAcrossForBeforeTrussPurplePixelFar, goAcrossForBeforeTrussPurplePixelCloseMidBeacon, goAcrossForBeforeTrussPurplePixelCloseWallBeacon, goAcrossForBeforeTrussPurplePixelCloseTrussBeacon;
    public Pose2d midStackPickUpFar, midStackPickUpClose, goAroundPurplePixelBeacon2;
    public Pose2d parkTriangle, lineUpParkTriangle, parkSquare, lineUpParkSquare;
    public static double headingPickUp = Math.toRadians(180);
    public static double headingPlace = Math.toRadians(0);
    public static double maxVelPlacePixel = 5;
    public static double distToGoForwardPickUpVision = -5;

    public static int sleepTimePlacePurplePixel = 100, sleepTimePickUpPixel = 500, sleepTimePlacePixel = 200, sleepTimeYellowPixel = 100, sleepTimeFirstPlace = 200, sleepTimeBetweenFirstAndSecondPlace = 400, sleepTimeExtendSlides = 500, sleepTimeWaitForFlipFirstPlace = 0, sleepTimeCorrectServo = 200, sleepTimeWaitToPlaceFirstPlacement = 500, sleepTimeWaitToResetAuton = 500;
    public boolean grabberIsOpen = true, rightGrabberOpen = true, leftGrabberOpen = true, flipperDown = true, airplaneLoaded = true, pullUpDown = true;
    public static int timesToRunAuton = 4;
    //Goofy noises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;
//    public BNO055IMU gyro;
    public static int testZone = 1;
    public static double lineDist = 0.05, offsetForPickUp = -3.5;

    public MonkeyMap (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        rotatorRange = maxRotatorPosUp - maxRotatorPosDown;
        correctorRange = MonkeyMap.correctorServoMaxPosition - MonkeyMap.correctorServoMinPosition;

        grabberServoLeft = myOpMode.hardwareMap.get(Servo.class, "grabberServoLeft");
        grabberServoRight = myOpMode.hardwareMap.get(Servo.class, "grabberServoRight");
        rotatorServo = myOpMode.hardwareMap.get(Servo.class, "rotatorServo");
        airplaneServo = myOpMode.hardwareMap.get(Servo.class, "airplaneServo");
        correctorServo = myOpMode.hardwareMap.get(Servo.class, "correctorServo");
        pullUpServoLeft = myOpMode.hardwareMap.get(Servo.class, "pullUpServoLeft");
        pullUpServoRight = myOpMode.hardwareMap.get(Servo.class, "pullUpServoRight");
        stackSetter = myOpMode.hardwareMap.get(Servo.class, "stackSetter");

        grabberServoRight.setDirection(Servo.Direction.REVERSE);
        pullUpServoRight.setDirection(Servo.Direction.REVERSE);

        grabberServoLeft.scaleRange(grabberServoScalerDown, grabberServoScalerUp);
        grabberServoRight.scaleRange(grabberServoScalerDown + offsetForGrabberScalar, grabberServoScalerUp + offsetForGrabberScalar);



        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class, ("backRight"));  //port 0
        pullUpMotorLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "pullUpMotorLeft");
        pullUpMotorRight = myOpMode.hardwareMap.get(DcMotorEx.class, "pullUpMotorRight");
        armMotorLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        flipperMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "flipperMotor");

        armMotorLeft.setCurrentAlert(currentAlertArmMotor, CurrentUnit.AMPS);

//        flipperMotor.setTargetPositionTolerance(flipperMotorTolerance);
//        armMotorLeft.setTargetPositionTolerance(armMotorTolerance);

//        flipperMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDFFlipperMotor);

//        gyro = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flipperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUpMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pullUpMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        relocalizer = myOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "relocalizer");
        detectionLeftGrabber = myOpMode.hardwareMap.get(ColorRangeSensor.class, "detectionLeftGrabber");
        detectionRightGrabber = myOpMode.hardwareMap.get(ColorRangeSensor.class, "detectionRightGrabber");

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
    }
    public void resetFlipperPos(){
        flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void initForAuton(String autonType){
        initPoses(autonType);
        closeGrabber();
        setRotatorUp();
        setCorrectorMid();
        pullUpDown();
//        resetSlidePoses();
//        resetFlipperPos();
        flipUp();
        resetSlides();
    }

    public void initPoses(String autonType){
        switch (autonType) {
            case "blueAfterTruss":
                startingPosition = new Pose2d(BlueAfterTrussPoses.xPosStartingPos, BlueAfterTrussPoses.yPosStartingPos, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                afterPlacePixelFar = new Pose2d(BlueAfterTrussPoses.xPosAfterPlaceFar, BlueAfterTrussPoses.yPosAfterPlaceFar, headingPlace);

                lineUpPurplePixelAfterTrussBeacon1 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                lineUpPurplePixelAfterTrussBeacon23 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPickUp);

                firstPlacementBeacon1After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfterBeacon1, headingPlace);
                firstPlacementBeacon2After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfterBeacon2, headingPlace);
                firstPlacementBeacon3After = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfterBeacon3, headingPlace);

                firstPlacementBeacon1BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosStartExtendFirstPlacementAfter, BlueAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPickUp);
                turnForFirstPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosStartExtendFirstPlacementAfter, BlueAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlace);
                lineUpForPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpFar, BlueAfterTrussPoses.yPosLineUpForPickUpFar, headingPickUp);
                lineUpForPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPlaceFar, BlueAfterTrussPoses.yPosLineUpForPlaceFar, headingPickUp);
                startArmExtendPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPickUpFar, BlueAfterTrussPoses.yPosStartArmExtendPickUpFar, headingPickUp);
                pickUpPixelFar = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelFar, BlueAfterTrussPoses.yPosPickUpPixelFar, headingPickUp);
                turnAfterPickUpPixelFar = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelFar, BlueAfterTrussPoses.yPosPickUpPixelFar, headingPlace);

                placePixelFar = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelFar, BlueAfterTrussPoses.yPosPlacePixelFar, BlueAfterTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosFlipAfterPlaceFar, BlueAfterTrussPoses.yPosFlipAfterPlaceFar, headingPickUp);
                startArmExtendPlaceFar = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceFar, BlueAfterTrussPoses.yPosStartArmExtendPlaceFar, headingPlace);
                turnAfterPlacePixel = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceFar, BlueAfterTrussPoses.yPosStartArmExtendPlaceFar, headingPickUp);
                lineUpForPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpClose, BlueAfterTrussPoses.yPosLineUpForPickUpClose, headingPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPickUpClose, BlueAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPickUp);
                pickUpPixelClose = new Pose2d(BlueAfterTrussPoses.xPosPickUpPixelClose, BlueAfterTrussPoses.yPosPickUpPixelClose, BlueAfterTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosStartArmExtendPlaceClose, BlueAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPickUp);
                placePixelClose = new Pose2d(BlueAfterTrussPoses.xPosPlacePixelClose, BlueAfterTrussPoses.yPosPlacePixelClose, BlueAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueAfterTrussPoses.xPosFlipAfterPlaceClose, BlueAfterTrussPoses.yPosFlipAfterPlaceClose, headingPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueAfterTrussPoses.xPosGoStraightThroughTrussClose, BlueAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPickUp);
                goAroundPurplePixelBeacon2 = new Pose2d(BlueAfterTrussPoses.xPosGoAroundPurplePixelBeacon2, BlueAfterTrussPoses.yPosGoAroundPurplePixelBeacon2, BlueAfterTrussPoses.headingStartingPositionAndBeacon);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, headingPickUp);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosMidStackPickUpFar, BlueAfterTrussPoses.yPosMidStackPickUpFar, BlueAfterTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(BlueAfterTrussPoses.xPosMidStackPickUpClose, BlueAfterTrussPoses.yPosMidStackPickUpClose, BlueAfterTrussPoses.headingMidStackPickUpClose);

                parkTriangle = new Pose2d(BlueAfterTrussPoses.xPosParkTriangle, BlueAfterTrussPoses.yPosParkTriangle, headingPlace);
                lineUpParkTriangle = new Pose2d(BlueAfterTrussPoses.xPosLineUpParkTriangle, BlueAfterTrussPoses.yPosLineUpParkTriangle, headingPlace);
                parkSquare = new Pose2d(BlueAfterTrussPoses.xPosParkSquare, BlueAfterTrussPoses.yPosParkSquare, headingPlace);
                lineUpParkSquare = new Pose2d(BlueAfterTrussPoses.xPosLineUpParkSquare, BlueAfterTrussPoses.yPosLineUpParkSquare, headingPlace);
                break;

            case "blueBeforeTruss":
                startingPosition = new Pose2d(BlueBeforeTrussPoses.xPosStartingPos, BlueBeforeTrussPoses.yPosStartingPos, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);

                lineUpPurplePixelAfterTrussBeacon1 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                lineUpPurplePixelAfterTrussBeacon23 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPickUp);

                firstPlacementBeacon1After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfterBeacon1, headingPickUp);
                firstPlacementBeacon2After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfterBeacon2, headingPlace);
                firstPlacementBeacon3After = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfterBeacon3, headingPickUp);

                firstPlacementBeacon1BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(BlueBeforeTrussPoses.xPosStartExtendFirstPlacementAfter, BlueBeforeTrussPoses.yPosStartExtendFirstPlacementAfter, headingPickUp);
                turnForFirstPlacementAfter = new Pose2d(BlueBeforeTrussPoses.xPosStartExtendFirstPlacementAfter, BlueBeforeTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlace);
                lineUpForPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpFar, BlueBeforeTrussPoses.yPosLineUpForPickUpFar, headingPickUp);
                lineUpForPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPlaceFar, BlueBeforeTrussPoses.yPosLineUpForPlaceFar, headingPickUp);
                startArmExtendPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpFar, BlueBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPickUp);
                pickUpPixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelFar, BlueBeforeTrussPoses.yPosPickUpPixelFar, headingPickUp);
                turnAfterPickUpPixelFar = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPlaceFar, BlueBeforeTrussPoses.yPosLineUpForPlaceFar, headingPlace);

                placePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelFar, BlueBeforeTrussPoses.yPosPlacePixelFar, BlueBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceFar, BlueBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPickUp);
                startArmExtendPlaceFar = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceFar, BlueBeforeTrussPoses.yPosStartArmExtendPlaceFar, headingPickUp);
                lineUpForPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosLineUpForPickUpClose, BlueBeforeTrussPoses.yPosLineUpForPickUpClose, headingPickUp);
                startArmExtendPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPickUpClose, BlueBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPickUp);
                pickUpPixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPickUpPixelClose, BlueBeforeTrussPoses.yPosPickUpPixelClose, BlueBeforeTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosStartArmExtendPlaceClose, BlueBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPickUp);
                placePixelClose = new Pose2d(BlueBeforeTrussPoses.xPosPlacePixelClose, BlueBeforeTrussPoses.yPosPlacePixelClose, BlueBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(BlueBeforeTrussPoses.xPosFlipAfterPlaceClose, BlueBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPickUp);
                goStraightThroughTrussClose = new Pose2d(BlueBeforeTrussPoses.xPosGoStraightThroughTrussClose, BlueBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPickUp);
                goAroundPurplePixelBeacon2 = new Pose2d(BlueBeforeTrussPoses.xPosGoAroundPurplePixelBeacon2, BlueBeforeTrussPoses.yPosGoAroundPurplePixelBeacon2, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(BlueBeforeTrussPoses.xPosMidStackPickUpFar, BlueBeforeTrussPoses.yPosMidStackPickUpFar, BlueBeforeTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(BlueBeforeTrussPoses.xPosMidStackPickUpClose, BlueBeforeTrussPoses.yPosMidStackPickUpClose, BlueBeforeTrussPoses.headingMidStackPickUpClose);

                parkTriangle = new Pose2d(BlueBeforeTrussPoses.xPosParkTriangle, BlueBeforeTrussPoses.yPosParkTriangle, headingPlace);
                lineUpParkTriangle = new Pose2d(BlueBeforeTrussPoses.xPosLineUpParkTriangle, BlueBeforeTrussPoses.yPosLineUpParkTriangle, headingPlace);
                parkSquare = new Pose2d(BlueBeforeTrussPoses.xPosParkSquare, BlueBeforeTrussPoses.yPosParkSquare, headingPlace);
                lineUpParkSquare = new Pose2d(BlueBeforeTrussPoses.xPosLineUpParkSquare, BlueBeforeTrussPoses.yPosLineUpParkSquare, headingPlace);
                break;

            case "redAfterTruss":
                startingPosition = new Pose2d(RedAfterTrussPoses.xPosStartingPos, RedAfterTrussPoses.yPosStartingPos, RedAfterTrussPoses.headingStartingPositionAndBeacon);

                lineUpPurplePixelAfterTrussBeacon1 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                lineUpPurplePixelAfterTrussBeacon23 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedAfterTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPickUp);

                firstPlacementBeacon1After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfterBeacon1, headingPlace);
                firstPlacementBeacon2After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfterBeacon2, headingPlace);
                firstPlacementBeacon3After = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfterBeacon3, headingPlace);

                firstPlacementBeacon1BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosStartExtendFirstPlacementAfter, RedAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPickUp);
                turnForFirstPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosStartExtendFirstPlacementAfter, RedAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlace);
                lineUpForPickUpFar = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpFar, RedAfterTrussPoses.yPosLineUpForPickUpFar, headingPickUp);
                lineUpForPlaceFar = new Pose2d(RedAfterTrussPoses.xPosLineUpForPlaceFar, RedAfterTrussPoses.yPosLineUpForPlaceFar, headingPickUp);
                startArmExtendPickUpFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpFar, RedAfterTrussPoses.yPosStartArmExtendPickUpFar, headingPickUp);
                pickUpPixelFar = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelFar, RedAfterTrussPoses.yPosPickUpPixelFar, headingPickUp);
                turnAfterPickUpPixelFar = new Pose2d(RedAfterTrussPoses.xPosLineUpForPlaceFar, RedAfterTrussPoses.yPosLineUpForPlaceFar, headingPlace);

                placePixelFar = new Pose2d(RedAfterTrussPoses.xPosPlacePixelFar, RedAfterTrussPoses.yPosPlacePixelFar, RedAfterTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceFar, RedAfterTrussPoses.yPosFlipAfterPlaceFar, headingPickUp);
                startArmExtendPlaceFar = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceFar, RedAfterTrussPoses.yPosStartArmExtendPlaceFar, headingPickUp);
                lineUpForPickUpClose = new Pose2d(RedAfterTrussPoses.xPosLineUpForPickUpClose, RedAfterTrussPoses.yPosLineUpForPickUpClose, headingPickUp);
                startArmExtendPickUpClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPickUpClose, RedAfterTrussPoses.yPosStartArmExtendPickUpClose, headingPickUp);
                pickUpPixelClose = new Pose2d(RedAfterTrussPoses.xPosPickUpPixelClose, RedAfterTrussPoses.yPosPickUpPixelClose, RedAfterTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(RedAfterTrussPoses.xPosStartArmExtendPlaceClose, RedAfterTrussPoses.yPosStartArmExtendPlaceClose, headingPickUp);
                placePixelClose = new Pose2d(RedAfterTrussPoses.xPosPlacePixelClose, RedAfterTrussPoses.yPosPlacePixelClose, RedAfterTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedAfterTrussPoses.xPosFlipAfterPlaceClose, RedAfterTrussPoses.yPosFlipAfterPlaceClose, headingPickUp);
                goStraightThroughTrussClose = new Pose2d(RedAfterTrussPoses.xPosGoStraightThroughTrussClose, RedAfterTrussPoses.yPosGoStraightThroughTrussClose, headingPickUp);
                goAroundPurplePixelBeacon2 = new Pose2d(RedAfterTrussPoses.xPosGoAroundPurplePixelBeacon2, RedAfterTrussPoses.yPosGoAroundPurplePixelBeacon2, RedAfterTrussPoses.headingStartingPositionAndBeacon);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, headingPickUp);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpFar, RedAfterTrussPoses.yPosMidStackPickUpFar, RedAfterTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(RedAfterTrussPoses.xPosMidStackPickUpClose, RedAfterTrussPoses.yPosMidStackPickUpClose, RedAfterTrussPoses.headingMidStackPickUpClose);

                parkTriangle = new Pose2d(RedAfterTrussPoses.xPosParkTriangle, RedAfterTrussPoses.yPosParkTriangle, headingPlace);
                lineUpParkTriangle = new Pose2d(RedAfterTrussPoses.xPosLineUpParkTriangle, RedAfterTrussPoses.yPosLineUpParkTriangle, headingPlace);
                parkSquare = new Pose2d(RedAfterTrussPoses.xPosParkSquare, RedAfterTrussPoses.yPosParkSquare, headingPlace);
                lineUpParkSquare = new Pose2d(RedAfterTrussPoses.xPosLineUpParkSquare, RedAfterTrussPoses.yPosLineUpParkSquare, headingPlace);
                break;

            case "redBeforeTruss":
                startingPosition = new Pose2d(RedBeforeTrussPoses.xPosStartingPos, RedBeforeTrussPoses.yPosStartingPos, RedBeforeTrussPoses.headingStartingPositionAndBeacon);

                lineUpPurplePixelAfterTrussBeacon1 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                lineUpPurplePixelAfterTrussBeacon23 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfterFarAndCloseBeacon1 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon1, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon1, headingPickUp);
                purplePixelPlacementAfterFarAndCloseBeacon23 = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfterBeacon23, RedBeforeTrussPoses.yPosPurplePixelPlacementAfterBeacon23, headingPickUp);

                firstPlacementBeacon1After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfterBeacon1, headingPlace);
                firstPlacementBeacon2After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfterBeacon2, headingPlace);
                firstPlacementBeacon3After = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfterBeacon3, headingPlace);

                firstPlacementBeacon1BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon1PlacementBeforeClose);
                firstPlacementBeacon2BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon2PlacementBeforeClose);
                firstPlacementBeacon3BeforeClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingBeacon3PlacementBeforeClose);

                firstPlacementBeacon1BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon1PlacementBeforeFar);
                firstPlacementBeacon2BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon2PlacementBeforeFar);
                firstPlacementBeacon3BeforeFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingBeacon3PlacementBeforeFar);

                startExtendFirstPlacementAfter = new Pose2d(RedBeforeTrussPoses.xPosStartExtendFirstPlacementAfter, RedBeforeTrussPoses.yPosStartExtendFirstPlacementAfter, headingPickUp);
                turnForFirstPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosStartExtendFirstPlacementAfter, RedAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlace);
                lineUpForPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpFar, RedBeforeTrussPoses.yPosLineUpForPickUpFar, headingPickUp);
                lineUpForPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPlaceFar, RedBeforeTrussPoses.yPosLineUpForPlaceFar, headingPickUp);
                startArmExtendPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpFar, RedBeforeTrussPoses.yPosStartArmExtendPickUpFar, headingPickUp);
                pickUpPixelFar = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelFar, RedBeforeTrussPoses.yPosPickUpPixelFar, headingPickUp);
                turnAfterPickUpPixelFar = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPlaceFar, RedBeforeTrussPoses.yPosLineUpForPlaceFar, headingPlace);

                placePixelFar = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelFar, RedBeforeTrussPoses.yPosPlacePixelFar, RedBeforeTrussPoses.headingPlaceFar);
                flipAfterPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceFar, RedBeforeTrussPoses.yPosFlipAfterPlaceFar, headingPickUp);
                startArmExtendPlaceFar = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceFar, RedBeforeTrussPoses.yPosStartArmExtendPlaceFar, headingPickUp);
                lineUpForPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosLineUpForPickUpClose, RedBeforeTrussPoses.yPosLineUpForPickUpClose, headingPickUp);
                startArmExtendPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPickUpClose, RedBeforeTrussPoses.yPosStartArmExtendPickUpClose, headingPickUp);
                pickUpPixelClose = new Pose2d(RedBeforeTrussPoses.xPosPickUpPixelClose, RedBeforeTrussPoses.yPosPickUpPixelClose, RedBeforeTrussPoses.headingPickUpClose);
                startArmExtendPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosStartArmExtendPlaceClose, RedBeforeTrussPoses.yPosStartArmExtendPlaceClose, headingPickUp);
                placePixelClose = new Pose2d(RedBeforeTrussPoses.xPosPlacePixelClose, RedBeforeTrussPoses.yPosPlacePixelClose, RedBeforeTrussPoses.headingPlaceClose);
                flipAfterPlaceClose = new Pose2d(RedBeforeTrussPoses.xPosFlipAfterPlaceClose, RedBeforeTrussPoses.yPosFlipAfterPlaceClose, headingPickUp);
                goStraightThroughTrussClose = new Pose2d(RedBeforeTrussPoses.xPosGoStraightThroughTrussClose, RedBeforeTrussPoses.yPosGoStraightThroughTrussClose, headingPickUp);
                goAroundPurplePixelBeacon2 = new Pose2d(RedBeforeTrussPoses.xPosGoAroundPurplePixelBeacon2, RedBeforeTrussPoses.yPosGoAroundPurplePixelBeacon2, RedBeforeTrussPoses.headingStartingPositionAndBeacon);

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelCloseMidBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.headingMidBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseWallBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.headingWallBeaconBefore);
                goAcrossForBeforeTrussPurplePixelCloseTrussBeacon = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.headingTrussBeaconBefore);

                midStackPickUpFar = new Pose2d(RedBeforeTrussPoses.xPosMidStackPickUpFar, RedBeforeTrussPoses.yPosMidStackPickUpFar, RedBeforeTrussPoses.headingMidStackPickUpFar);
                midStackPickUpClose = new Pose2d(RedBeforeTrussPoses.xPosMidStackPickUpClose, RedBeforeTrussPoses.yPosMidStackPickUpClose, RedBeforeTrussPoses.headingMidStackPickUpClose);

                parkTriangle = new Pose2d(RedBeforeTrussPoses.xPosParkTriangle, RedBeforeTrussPoses.yPosParkTriangle, headingPlace);
                lineUpParkTriangle = new Pose2d(RedBeforeTrussPoses.xPosLineUpParkTriangle, RedBeforeTrussPoses.yPosLineUpParkTriangle, headingPlace);
                parkSquare = new Pose2d(RedBeforeTrussPoses.xPosParkSquare, RedBeforeTrussPoses.yPosParkSquare, headingPlace);
                lineUpParkSquare = new Pose2d(RedBeforeTrussPoses.xPosLineUpParkSquare, RedBeforeTrussPoses.yPosLineUpParkSquare, headingPlace);
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
    public void openRightGrabberTele(){
        grabberServoRight.setPosition(grabberOpenTele);
        rightGrabberOpen = true;
    }
    public void openLeftGrabberTele(){
        grabberServoLeft.setPosition(grabberOpenTele);
        leftGrabberOpen = true;
    }
    public void openGrabberTele(){
        openLeftGrabberTele();
        openRightGrabberTele();
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
            openGrabberTele();
        }
    }
    public void toggleGrabberFlipperUp(){
        if(grabberIsOpen){
            closeGrabber();
        }
        else{
            openGrabberFlipperUp();
        }
    }
    public void toggleLeftGrabberFlipperUp(){
        if(leftGrabberOpen){
            closeLeftGrabber();
        }
        else{
            openLeftGrabberFlipperUp();
        }
    }
    public void toggleRightGrabberFlipperUp(){
        if(rightGrabberOpen){
            closeRightGrabber();
        }
        else{
            openRightGrabberFlipperUp();
        }
    }
    public void openRightGrabberFlipperUp(){
        grabberServoRight.setPosition(grabberOpenFlipperUp);
        rightGrabberOpen = true;
    }
    public void openLeftGrabberFlipperUp(){
        grabberServoLeft.setPosition(grabberOpenFlipperUp);
        leftGrabberOpen = true;
    }
    public void openGrabberFlipperUp(){
        openLeftGrabberFlipperUp();
        openRightGrabberFlipperUp();
        grabberIsOpen = true;
    }

    public void toggleLeftGrabber(){
        if(leftGrabberOpen){
            closeLeftGrabber();
        }
        else{
            openLeftGrabberTele();
        }
    }

    public void toggleRightGrabber(){
        if(rightGrabberOpen){
            closeRightGrabber();
        }
        else{
            openRightGrabberTele();
        }
    }

    public void resetArmAuton(){
//        flipUp();
//        closeGrabber();
        setRotatorFlush();
        setCorrectorMid();
        resetSlides();
    }
    public void resetArm(){
        resetSlides();
        flipUp();
        closeGrabber();
        setRotatorUp();
        setCorrectorMid();
    }
    public void setArmFirstPlace(){
        encodedSlipperySlides(slidesFirstPlacePos, slidePowerEncoder);
        flipUpFirstPlace();
        setAutoRotator(flipperMotor.getTargetPosition());
    }
    public void setFlipperPos(int pos, double power) {
        flipperMotor.setTargetPosition(pos);
        flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipperMotor.setPower(power);
    }
    public void flipDown(){
        setFlipperPos(flipperPosDown, flipperPower);
        flipperDown = true;
    }
    public void flipDownForAuton(){
        setFlipperPos(flipperPosDownForAuton, flipperPower);
        flipperDown = true;
    }
    public void flipDownPurplePixel(){
        setFlipperPos(flipperPosUpPurplePixels, flipperPower);
        setRotatorFlush();
    }
    public void flipAndRotateDownAndExtend6Pixels(){
        setFlipperPos(flipperPosDown6Pixels, flipperPower);
        rotatorServo.setPosition(rotator6Pixels);
        fullyExtendSlides();
    }
    public void flipAndRotateDown6Pixels(){
        setFlipperPos(flipperPosDown6Pixels, flipperPower);
        rotatorServo.setPosition(rotator6Pixels);
//        fullyExtendSlides();
    }
    public void flipAndRotateDown5Pixels(){
        setFlipperPos(flipperPosDown5Pixels, flipperPower);
        stackSetter.setPosition(stackSetter5PixelsPos);
//        rotatorServo.setPosition(rotator5Pixels);
    }
    public void flipAndRotateDown4Pixels(){
        setFlipperPos(flipperPosDown4Pixels, flipperPower);
//        rotatorServo.setPosition(rotator4Pixels);
    }
    public void flipAndRotateDown3Pixels(){
        setFlipperPos(flipperPosDown3Pixels, flipperPower);
//        rotatorServo.setPosition(rotator3Pixels);
    }
    public void flipAndRotateDown2Pixels(){
        setFlipperPos(flipperPosDown2Pixels, flipperPower);
//        rotatorServo.setPosition(rotator2Pixels);
    }
    public void flipUp(){
        setFlipperPos(flipperPosUp, flipperPower);
        flipperDown = false;
    }
    public void flipUpFirstPlace(){
        setFlipperPos(flipperPosUpFirstPlace, flipperPower);
    }
    public void flipUpFirstPlaceAuton(){
        setFlipperPos(flipperPosUpFirstPlaceAuton, flipperPowerAuton);
    }
    public void extendOutPickUp(int slidesPos){
        encodedSlipperySlides(extendTeleopPos, slidePowerEncoder);
        setAutoRotatorGrabber(slidesPos);
        openGrabberTele();
        setFlipperPos(flipperMotorPickUpTeleop, flipperPower);
    }
    public void pumpFakeMacro(int slidesPos){
        setAutoRotatorGrabber(slidesPos);
        setFlipperPos(flipperMotorPickUpTeleop, flipperPower);
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
    public void pullUpDown(){
        pullUpServoLeft.setPosition(pullUpServoDownPos);
        pullUpServoRight.setPosition(pullUpServoDownPos);
        pullUpDown = true;
    }
    public void pullUpUp(){
        pullUpServoLeft.setPosition(pullUpServoUpPos);
        pullUpServoRight.setPosition(pullUpServoUpPos);
        pullUpDown = false;
    }
    public void togglePullUp() {
        if (pullUpDown) {
            pullUpUp();
        } else {
            pullUpDown();
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
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setSlidePowers(power);
    }
    public void setSlidePowers(double power){
        armMotorLeft.setPower(power);
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
        stackSetter.setPosition(stackSetterUpPos);
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(turnAfterPlacePixel, ""));
        posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, "flipUpPlaceInAuton and extendSlidesPlaceAuton"));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(placePixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        openGrabber();
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(afterPlacePixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true, maxVelPlacePixel);
        resetArm();
    }
    public void pickUpInAutonFar(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, int pixelType, boolean isOddPixels, boolean goToMid){
        if(pixelType == 1){
            flipAndRotateDown5Pixels();
        }
        else if(pixelType == 2){
            flipAndRotateDown3Pixels();
        }


        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(startArmExtendPlaceFar, ""));
        posesToGoTo.add(new PosesAndActions(turnAfterPlacePixel, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);

        openGrabber();
//        fullyExtendSlides();
        setCorrectorMid();
        setRotatorFlush();

        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(lineUpForPlaceFar, "fullyExtendSlides and openGrabber and setCorrectorMid and setRotatorFlush"));
        posesToGoTo.add(new PosesAndActions(lineUpForPlaceFar, ""));
        posesToGoTo.add(new PosesAndActions(pickUpPixelFar, ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        double yPosAfterSeeing = (lineDist * OpenCVGreatestColorTest.xDist) + pickUpPixelFar.getY() + offsetForPickUp;
//        fullyExtendSlides();
        posesToGoTo.clear();
        posesToGoTo.add(new PosesAndActions(new Pose2d(pickUpPixelFar.getX() + distToGoForwardPickUpVision, yPosAfterSeeing, headingPickUp), ""));
        follower.reinit(posesToGoTo);
        follower.goToPoints(true);
        while(!armMotorLeft.isOverCurrent()){
            encodedSlipperySlides(armMotorLeft.getCurrentPosition() + slideSpeed, slidePowerEncoder);
        }
        slidesPosWhenPickUp = armMotorLeft.getCurrentPosition();
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
        resetArmAuton();
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
        resetArmAuton();
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
            return 3;
//            return testZone;
        }
        else if(OpenCVDetectTeamProp.centerX < 160){
            return 2;
//            return testZone;
        }
        else if(OpenCVDetectTeamProp.centerX > 160){
            return 1;
//            return testZone;
        }
        return testZone;
    }
    public int wrapPixelTypeInt(int pixelType){
        while(pixelType > 1){
            pixelType-=2;
        }
        return pixelType;
    }
    public double angleWrap(double angle){
        angle += 90;
        while(angle < -180){
            angle += 180;
        }
        while(angle > 180){
            angle -= 180;
        }
        return angle;
    }
    public void setAutoCorrector(double heading){
        double correctorScalar = angleWrap(heading)/180;
        double newCorrectorPos = correctorServoMinPosition + (correctorScalar*correctorRange);
        correctorServo.setPosition(newCorrectorPos);
    }
    public void setAutoCorrectorOtherWay(double heading){
        double correctorScalar = angleWrap(heading)/180;
        double newCorrectorPos = correctorServoMaxPosition - (correctorScalar*correctorRange);
        correctorServo.setPosition(newCorrectorPos);
    }
    public void setAutoRotator(double flipperPos){
        double flipperScalar = Math.abs((flipperPos-lowestFlipperPosDown)/(lowestFlipperPosDown-highestFlipperPos));
        double newRotatorPos = MonkeyMap.maxRotatorPosDown + (flipperScalar*rotatorRange);
        rotatorServo.setPosition(newRotatorPos);
    }
    public void setAutoRotatorGrabber(double slidesPos){
        double flipperScalar = Math.abs((slidesPos - resetSlidesPos)/(slidesFullyExtendedPos));
        double newRotatorPos = MonkeyMap.minRotatorPosPickUp + (flipperScalar*rotatorRangeSlidesDown);
        rotatorServo.setPosition(newRotatorPos);
    }
}
