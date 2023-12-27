package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

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
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;

@Config
public class MonkeyMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    public LinearOpMode myOpMode;
    public OpMode myOperatorMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight, conveyerMotor, armMotorLeft, armMotorRight;

    public Servo spencerLikesKids, grabberServoLeft, grabberServoRight, rotatorServo, flipperServoLeft, flipperServoRight, airplaneServo, correctorServo;

    public VoltageSensor batteryVoltageSensor;
    public DistanceSensor lineUpSensor;

    //Servo Positions
    public static double grabberServoScalerDown = 0.1, grabberServoScalerUp = 0.5, offsetForGrabberScalar = -0.02;
    public static double rotatorScalarDown = 0, rotatorScalarUp = 0;
    public static double grabberClosed = 0.94, grabberOpen = 0.2, grabberServoOpenPlacement = 0.2;
    public static double wheelServoPow = 1, servoStopPow = 0.5;
    public static double flipperScalarDown = 0, flipperScalarUp = 1, flipperScalarOffset = 0, flipperPosDown = 0.17, flipperPosAcross = 0.89, rotatorFlushWithSldies = 0.5, rotatorPickUpAndPlace = 0.45;
    public static double airplaneServoLoadedPos = 0.29, airplaneServoGoPos = 0.16;
    public static double stackKnockerKnockedPos = 0, stackKnockerResetPos = 0.34;
    public static double correctorServoMidPos = 0.5;
    public static double rotatorServoUpPos = 0.22;

    //Motor powers and pos
    public static double conveyerPower = -1, unloadPowerForAuton = 0.55, stopLoadPower = 0, unloadPower = 1;
    public static int resetSlidesPos = 0, placementSlidesPos = -150, slidePosFirstPlace = -150;

    public static double holdPowerForSlides = 0, slidePowerDown = 0.3, slidePowerUp = 0.6;

    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;
    public static double correctorServoSpeed = 0.003, flipperServoSpeed = 0.003, rotatorServoSpeed = 0.003;


    //Bfue Poses
    public Pose2d startingPosition, purplePixelPlacementAfter, firstPlacementAfter, startExtendFirstPlacementAfter, lineUpForPickUpFar, startArmExtendPickUpFar, pickUpPixelFar,  placePixelFar, flipAfterPlaceFar, startArmExtendPlaceFar, lineUpForPickUpClose, startArmExtendPickUpClose, pickUpPixelClose, startArmExtendPlaceClose, placePixelClose, flipAfterPlaceClose, goAcrossForBeforeTrussPurplePixelFar, goAcrossForBeforeTrussPurplePixelClose = new Pose2d(-36, -56, 90);

    //All Poses
    public static double headingPlaceAndPickUp = Math.toRadians(0);
    public double xPosPickUpPosAfterKnocked;

    public static int sleepTimePlacePreloadBeacon = 700, sleepTimePickUpPixel = 0, sleepTimePlacePixels = 400, sleepTimeKnockStack = 300, sleepTimeAfterFlip = 500, sleepTimeFlipForFirstPlaceAfterTruss = 200, sleepTimePutSlidesUpNoBreakFlipper = 300;
    public boolean grabberIsOpen = true, rightGrabberOpen = true, leftGrabberOpen = true, flipperDown = true, airplaneLoaded = true, rotatorDown = false, isKnocked = false;
    public static double timesToRunAuton = 1;
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
        grabberServoLeft = myOpMode.hardwareMap.get(Servo.class, "grabberServoLeft");
        grabberServoRight = myOpMode.hardwareMap.get(Servo.class, "grabberServoRight");
        rotatorServo = myOpMode.hardwareMap.get(Servo.class, "rotatorServo");
        flipperServoLeft = myOpMode.hardwareMap.get(Servo.class, "flipperServoLeft");
        flipperServoRight = myOpMode.hardwareMap.get(Servo.class, "flipperServoRight");
        airplaneServo = myOpMode.hardwareMap.get(Servo.class, "airplaneServo");
        correctorServo = myOpMode.hardwareMap.get(Servo.class, "correctorServo");

        flipperServoLeft.setDirection(Servo.Direction.REVERSE);
//        rotatorServo.setDirection(Servo.Direction.REVERSE);
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
        closeGrabber();
        setRotatorUp();
        setCorrectorMid();
    }

    public void initPoses(String autonType){
        switch (autonType) {
            case "blueAfterTruss":
                startingPosition = new Pose2d(BlueAfterTrussPoses.xPosStartingPos, BlueAfterTrussPoses.yPosStartingPos, BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosPurplePixelPlacementAfter, BlueAfterTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                firstPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosFirstPlacementAfter, BlueAfterTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
                startExtendFirstPlacementAfter = new Pose2d(BlueAfterTrussPoses.xPosStartExtendFirstPlacementAfter, BlueAfterTrussPoses.yPosStartExtendFirstPlacementAfter, headingPlaceAndPickUp);
                lineUpForPickUpFar = new Pose2d(BlueAfterTrussPoses.xPosLineUpForPickUpFar, BlueAfterTrussPoses.yPosLineUpForPickUpClose, headingPlaceAndPickUp);
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

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, -BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelClose = new Pose2d(BlueAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, -BlueAfterTrussPoses.headingStartingPositionAndBeacon);
                break;

            case "blueBeforeTruss":
                startingPosition = new Pose2d(BlueBeforeTrussPoses.xPosStartingPos, BlueBeforeTrussPoses.yPosStartingPos, BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfter = new Pose2d(BlueBeforeTrussPoses.xPosPurplePixelPlacementAfter, BlueBeforeTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                firstPlacementAfter = new Pose2d(BlueBeforeTrussPoses.xPosFirstPlacementAfter, BlueBeforeTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
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

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, -BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelClose = new Pose2d(BlueBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, BlueBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, -BlueBeforeTrussPoses.headingStartingPositionAndBeacon);
                break;

            case "redAfterTruss":
                startingPosition = new Pose2d(RedAfterTrussPoses.xPosStartingPos, RedAfterTrussPoses.yPosStartingPos, RedAfterTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosPurplePixelPlacementAfter, RedAfterTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                firstPlacementAfter = new Pose2d(RedAfterTrussPoses.xPosFirstPlacementAfter, RedAfterTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
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

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, -RedAfterTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelClose = new Pose2d(RedAfterTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedAfterTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, -RedAfterTrussPoses.headingStartingPositionAndBeacon);
                break;

            case "redBeforeTruss":
                startingPosition = new Pose2d(RedBeforeTrussPoses.xPosStartingPos, RedBeforeTrussPoses.yPosStartingPos, RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                purplePixelPlacementAfter = new Pose2d(RedBeforeTrussPoses.xPosPurplePixelPlacementAfter, RedBeforeTrussPoses.yPosPurplePixelPlacementAfter, headingPlaceAndPickUp);
                firstPlacementAfter = new Pose2d(RedBeforeTrussPoses.xPosFirstPlacementAfter, RedBeforeTrussPoses.yPosFirstPlacementAfter, headingPlaceAndPickUp);
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

                goAcrossForBeforeTrussPurplePixelFar = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelFar, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelFar, -RedBeforeTrussPoses.headingStartingPositionAndBeacon);
                goAcrossForBeforeTrussPurplePixelClose = new Pose2d(RedBeforeTrussPoses.xPosGoAcrossForBeforeTrussPurplePixelClose, RedBeforeTrussPoses.yPosGoAcrossForBeforeTrussPurplePixelClose, -RedBeforeTrussPoses.headingStartingPositionAndBeacon);
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

//    public void toggleIntakeWheels(){
//        if(!wheelOn){
//            wheelServoLeft.setPosition(wheelServoPow);
//            wheelServoRight.setPosition(wheelServoPow);
//            wheelOn = true;
//        }
//        else{
//            wheelServoLeft.setPosition(servoStopPow);
//            wheelServoRight.setPosition(servoStopPow);
//            wheelOn = false;
//        }
//    }

//    public void toggleConveyer(){
//        if(!conveyerOn){
//            conveyerMotor.setPower(conveyerPower);
//            conveyerOn = true;
//        }
//        else{
//            conveyerMotor.setPower(0);
//            conveyerOn = false;
//        }
//    }

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
    public void setCorrectorMid(){
        correctorServo.setPosition(correctorServoMidPos);
    }
//    public void knockStack(){
//        stackKnocker.setPosition(stackKnockerKnockedPos);
//        isKnocked = true;
//    }
//    public void resetKnocker(){
//        stackKnocker.setPosition(stackKnockerResetPos);
//        isKnocked = false;
//    }
//    public void toggleKnocker() {
//        if (isKnocked) {
//            resetKnocker();
//        } else {
//            knockStack();
//        }
//    }
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
//    public void rotateDown(){
//        rotatorServo.setPosition(rotatorPickUp);
//        rotatorDown = true;
//    }
//    public void rotateUp(){
//        rotatorServo.setPosition(rotatorPlace);
//        rotatorDown = false;
//    }
//    public void toggleRotator() {
//        if (rotatorDown) {
//            rotateUp();
//        } else {
//            rotateDown();
//        }
//    }
    public void setRotatorFlush(){rotatorServo.setPosition(rotatorFlushWithSldies);}
    public void rotatorPickUpAndPlace(){rotatorServo.setPosition(rotatorPickUpAndPlace);}
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
//    public void placeInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose, boolean isFirstTime){
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(afterPickUpNoPixelCrash, ""));
//        posesToGoTo.add(new PosesAndActions(lineUpForTruss, ""));
//        posesToGoTo.add(new PosesAndActions(underTrussGoingBack, "unloadPixel and closeGrabber"));
//        posesToGoTo.add(new PosesAndActions(underTruss, "stopLoadingPixels and placeSlides"));
//        posesToGoTo.add(new PosesAndActions(slidesDownAfterPlace, "flipDown"));
//        posesToGoTo.add(new PosesAndActions(lineUpPlacement, ""));
//        if(isFirstTime){
//            posesToGoTo.add(new PosesAndActions(putSlidesBackDownBeforePlace, "resetSlides"));
//        }
//        posesToGoTo.add(new PosesAndActions(finalPose, ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//        openGrabber();
//        myOpMode.sleep(MonkeyMap.sleepTimePlacePixels);
//        if(isFirstTime){
//            flipUp();
//            myOpMode.sleep(sleepTimeFlipForFirstPlaceAfterTruss);
//            placeSlides();
//            myOpMode.sleep(sleepTimePutSlidesUpNoBreakFlipper);
//        }
//        else{
//            flipUp();
//            myOpMode.sleep(MonkeyMap.sleepTimeAfterFlip);
//        }
//
//    }
//    public void goToPickUpInAuton(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, Pose2d finalPose){
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(afterPlacePosForNoCrash, ""));
//        posesToGoTo.add(new PosesAndActions(underTruss, "resetSlides"));
//        posesToGoTo.add(new PosesAndActions(lineUpForTruss, "loadPixels"));
//        posesToGoTo.add(new PosesAndActions(finalPose, ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//    }
//    public void autonVisionPickUp(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo){
//        posesToGoTo.clear();
//        double yPosAfterSeeing = ((lineDist * Math.sin(Math.toRadians(OpenCVGreatestColorTest.thetaX))))+ beforePickUpAfterKnocked.getY() + offsetForPickUp;
//        posesToGoTo.add(new PosesAndActions(new Pose2d(xPosPickUpPosAfterKnocked, yPosAfterSeeing, MonkeyMap.headingPlaceAndPickUp), ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//        myOpMode.sleep(MonkeyMap.sleepTimePickUpPixel);
//    }
//    public void autonLoop(PointFollower follower, ArrayList<PosesAndActions> posesToGoTo, boolean isFirstTime){
//        posesToGoTo.clear();
//        Pose2d finalPose = new Pose2d();
//        finalPose = beforePickUpAfterKnocked;
//        goToPickUpInAuton(follower, posesToGoTo, finalPose);
//        autonVisionPickUp(follower, posesToGoTo);
//
//        posesToGoTo.clear();
//        posesToGoTo.add(new PosesAndActions(beforePickUpAfterKnocked, ""));
//        follower.reinit(posesToGoTo);
//        follower.goToPoints(true);
//
//        autonVisionPickUp(follower, posesToGoTo);
//
//        finalPose = placementPos;
//
//        placeInAuton(follower, posesToGoTo, finalPose, isFirstTime);
//    }

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
