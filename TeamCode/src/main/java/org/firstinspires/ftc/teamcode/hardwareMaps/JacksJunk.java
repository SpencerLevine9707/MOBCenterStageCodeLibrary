package org.firstinspires.ftc.teamcode.hardwareMaps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

@Config
public class JacksJunk {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode
    private LinearOpMode myOpMode;

    //Define all hardware
    public DcMotor frontLeft, frontRight, backLeft, backRight, armMotor1, armMotor2;

    public Servo spencerLikesKids, grabberServo1, rotatorServo, armServo1, armServo2;

    public VoltageSensor batteryVoltageSensor;

    public DistanceSensor jacksTip, jacksAsshole;
    
    //Arm servo positions

    public static double armServoOffset = -0.01;
    public static double armServoPickUpPos = 0, armServoUp45Degrees = 0.25, armServoUpPos = 0.5, armServoReadyPlace= 0.8, armServoPlacePos = 1, armServoScaleDown = 0.24, armServoScaleUp = 0.78;
    
    //Speed for arm motors to keep slides up
    public static double holdSpeed = 0.125;

    //teleop
    //arm speed up low and high speed
    public static double lowArmSpeed = -0.5;
    public static double highArmSpeed = -1;
    //arm speed down low and high speed
    public static double lowArmSpeedDown = -0.06;
    public static double highArmSpeedDown = -0.4;
    //wait time on rotate for flip
    public static double waitTimeRotate = 0.5;
    //wait time on grabber so as to not jerk off the pole
    public static double waitTimeGrabber = 0.08;
    //if it'll do the macros for the arms or not in teleop
    public static boolean doesMacroArms = false;


    //Positions for grabber open and closed
    public static double grabberClosed1 = 0.84, grabberOpen1 = 0.7, grabberPark = 0.925;

    //Rotator positions for rotatorServo
    public static double rotatorPosition = 0.055, rotatorDist1 = 0.93;

    // Sleep times for auton
    // TODO: 3/31/23 change sleep times AS needed to optimize acuarcy and speed
    public static int sleepTimeArmDown = 150, sleepTimeGrabberOpen = 75, sleepTimeGrabberClose = 75, getOutTheWaySleepTime = 150, sleepTimeGrabberCloseStart = 0, sleepTimeGrabberCloseGetOutTheWay = 200;

    //Positons for spencerLikesKids, or the servo that lifs W-Odo wheels
    // TODO: 3/31/23 change spencerLikesKid positions AS needed
    public static double spencerLikesKidsPosUp = 0.4, spencerLikesKidsPosDown = 0.9;

    //Positions for Cone heights in auton
    // TODO: AS needed, cone heights for auton
    public static int ch2 = 180, ch3 = 140, ch4 = 100, ch5 = 60, ch6 = 30;
    public static int[] coneHeights;

    //Positions for pole heights in auton
    //TODO: AS needed, change pole heights to match new Slides
    //h3 = 765?
    public static int slideReset = 10, h1 = 300, h2 = 465, h3 = 765;

    //Poses for auton
    public Pose2d startingPosition, lineUpPlacePreload, placePosHigh, placePosHighPreload, placePosMid, placePosPreloadMidtonomous, placePosFarHigh, CONES, lineUpPickUpConePreloadMidtonomous, lineUpConesFarHighPlace, park1, park2, park3, park1Far, park2Far, park3Far;

    //Tangents for splines in auto
    public static double conesTan = Math.toRadians(0), placeTanHighJunct = Math.toRadians(210), placeTanMidAndFarHighJuct = Math.toRadians(155), /*tanForLineUpPlace = Math.toRadians(262.69),*/ /*tanForPreloadPlace = Math.toRadians(264.05)*/ tanForPreloadPlace = 4.3, tanForConesPlacFarHigh = Math.toRadians(0.38);

    //Values for distance sensors to adjust auton
    //TODO: AS needed change values for distance sensors
    public static double distanceForGoodPickUp = 4.2, distanceForBadPickUp = 3.7, distanceForGoodPlace = 3, distanceForBadPlace = 2.5, distanceForGoodPlace2 = 2.25, distanceForBadPlace2 = 1.75, distanceForIdealPlace = (distanceForGoodPlace + distanceForBadPlace) / 2, distanceForIdealPlace2 = (distanceForGoodPlace2 + distanceForBadPlace2) / 2, distanceForIdealPickUp = (distanceForGoodPickUp + distanceForBadPickUp) / 2;

    public static double turningNeededPerBox = Math.toRadians(3);

    //Distance until bot lifts arm on the first move
    //TODO: Adjust if needed
    public static double distanceBeforeColor = 18.4, distanceBeforeCockSlide = 25, distanceBeforeSlidesAndShit = 7, distanceBeforeOpenGrabber = 10.5, distanceBeforeCloseGrabber = 5.5;

    //Powers for encoder arms up/down
    //TODO: adjust so the arms don't fuck themselves
    public static double armPowerUp = 0.8;
    public static double armPowerDown = 0.5;

    //X and Y positions for auton poses
    //TODO: Adjust AS needed
    //Start
    //used to be 33.5, 62.05
    //35 almost there
    //39 a bit too far to the right
    public static double xPosStartingPosition = 36.25, yPosStartingPosition = -63;

    //CONES
    //60.5, 11
    //61.5, 11
    //10.5
    //63, 10.75 for right??
    //12.5?
    //62 left

    public static double xPosCONES = 62, yPosCONES = 14;

    //High junction preload

    public static double xPosPlacePosHighPreload = 33.5, yPosPlacePosHighPreload = 5;

    //High junction
    //34.25, 6.4
    //33.75, 5.9 is 1
    //34.25, 5.8
    public static double xPosPlacePosHigh = 33.5, yPosPlacePosHigh = 5;

    //Mid junction
    //36-37 for mid for some reason - prob bc dead wheel comes off ground

    //17.81 needs up; try 15
    //33.75, 16.5 for 1
    //33, 17
    public static double xPosPlaceMid = 33.5, yPosPlaceMid = 17;

    public static double xPosPlaceMidPreload = 32.5, yPosPlaceMidPreload = 24;

    public static double xPosLineUpPickUpConePreloadMidtonomous = 43, yPosLineUpPickUpConePreloadMidtonomous = 13.5;

    //Far High junction
    //9.3, 19.5
    public static double xPosPlaceFarHigh = 12, yPosPlaceFarHigh = 19.5;

    //Parks
    //y pos used to be 11.75
    public static double yPosAllParks = 14, yPosParkBack = 34, xPosPark1 = 62, xPosPark2 = 36, xPosPark3 = 12;

    //Line Up Place
    public static double xPosLineUpPlacePreload = 37.15, yPosLineUpPlacePreload = 19.8;

    //Line Up Cones Far High
    public static double xPosLineUpConesFarHighPlace = 31.58, yPosLineUpConesFarHighPlace = 14;

    //Headings for poses
    //TODO: Adjust IF needed
    //Cones and park 0.2?
    public static double startingPositionAndLineUpPlaceHeading = Math.toRadians(90), CONESAndParkHeading = Math.toRadians(0), placePosHighHeading = Math.toRadians(30), placePosMidAndFarHighHeading = Math.toRadians(-30), lineUpConesFarHighPlaceHeading = CONESAndParkHeading;

    //Times for park early for CONES and placePos
    //TODO: Adjust these times AS needed
    public static double parkEarlyPlace = 25, parkEarlyCones = 25;

    //Values for distance sensor for when it reads more than them it doesn't adjust
    //TODO: Change dists AS needed
    public static double noConesDistance = 7, noPlaceDistance = 7;

    public static double velocityForSpeedUp = 60, accelerationForSpeedUp = 50;

    //Define trajectories
    public Trajectory placeConePreload, backToStart1, backToStart3, backToStart, placeConePreloadMidtonomous, pickUpConePreloadMidtonomous;

    //Define GoofyNoises
    public int matchStart, wIntro, endgameStart, yabbaDabbaDo, driversPickUp, funnyFunny, teleStart;

    public JacksJunk (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // init all hardware
    public void init(boolean isTeleOp){

        //Init armMotors for slides
        armMotor1 = myOpMode.hardwareMap.get(DcMotor.class, ("armMotor1"));
        armMotor2 = myOpMode.hardwareMap.get(DcMotor.class, ("armMotor2"));

        //Init servos
        armServo1 = myOpMode.hardwareMap.get(Servo.class, ("armServo1"));
        armServo2 = myOpMode.hardwareMap.get(Servo.class, ("armServo2"));
        rotatorServo = myOpMode.hardwareMap.get(Servo.class, "rotatorServo");
        grabberServo1 = myOpMode.hardwareMap.get(Servo.class, "grabberServo1");
        spencerLikesKids = myOpMode.hardwareMap.get(Servo.class, "spencerLikesKids");
        
        //Init distance sensors
        jacksTip = myOpMode.hardwareMap.get(DistanceSensor.class, "jacksTip");
        jacksAsshole = myOpMode.hardwareMap.get(DistanceSensor.class, "jacksAsshole");

        //Init voltage sensor
        batteryVoltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        //Directions of armMotors for slides
        armMotor1.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.FORWARD);

        //Reset encoders on armMotors
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reverse armServo1
        armServo1.setDirection(Servo.Direction.REVERSE);

        //Scale Ranges
        //0.23, 0.78
        //0.22, 0.77
        armServo1.scaleRange(armServoScaleDown, armServoScaleUp);
        armServo2.scaleRange(armServoScaleDown + armServoOffset, armServoScaleUp + armServoOffset);

        if(isTeleOp){
            //Only init wheels for teleop because sample mecanum drive does it dif for auton
            frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
            frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
            backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
            backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

            //sets arm motors for slides to run without encoder for teleop
            armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //sets motor directions for teleop and braking beahavior; not needed in auton bc sample mecanum drive does this differently
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Init webcam
//        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //Init telementry and webcam for dashboard use
        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);

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
    public void definePosesAndTrajectoriesAuton(boolean isRightSide, SampleMecanumDrive drive){
        coneHeights = new int[]{ch2, ch3, ch4, ch5, ch6};
        //Checks if right side
        if(isRightSide) {
//            xPosCONES = ;
            //Redefines tans
//            double tempTan = placeTanHighJunct;
//            placeTanHighJunct = placeTanMidAndFarHighJuct;
//            placeTanMidAndFarHighJuct = tempTan;
//            tanForPreloadPlace = -Math.abs(tanForPreloadPlace);
            placeTanHighJunct = -Math.abs(placeTanHighJunct);
            placeTanMidAndFarHighJuct = -Math.abs(placeTanMidAndFarHighJuct);
            tanForPreloadPlace = -Math.abs(tanForPreloadPlace);
            tanForConesPlacFarHigh = -Math.abs(tanForConesPlacFarHigh);

            //makes all y poses negative
            yPosAllParks = -Math.abs(yPosAllParks);
            yPosParkBack = -Math.abs(yPosParkBack);
            //-9
            yPosCONES = -Math.abs(yPosCONES);
            yPosLineUpPlacePreload = -Math.abs(yPosLineUpPlacePreload);
            yPosPlaceMid = -Math.abs(yPosPlaceMid);
            yPosPlaceMidPreload = -Math.abs(yPosPlaceMidPreload);
            yPosPlaceFarHigh = -Math.abs(yPosPlaceFarHigh);
            yPosPlacePosHigh = -Math.abs(yPosPlacePosHigh);
            //-63 for right
            yPosStartingPosition = -Math.abs(yPosStartingPosition);
            yPosLineUpPlacePreload = -Math.abs(yPosLineUpPlacePreload);
            yPosLineUpConesFarHighPlace = -Math.abs(yPosLineUpConesFarHighPlace);
            yPosPlacePosHighPreload = -Math.abs(yPosPlacePosHighPreload);
            yPosLineUpPickUpConePreloadMidtonomous = -Math.abs(yPosLineUpPickUpConePreloadMidtonomous);

            //36.25
//            xPosStartingPosition = 35;

            //makes necessary headings negative
            placePosHighHeading = -Math.abs(placePosHighHeading);
            placePosMidAndFarHighHeading = Math.abs(placePosMidAndFarHighHeading);
            startingPositionAndLineUpPlaceHeading = -Math.abs(startingPositionAndLineUpPlaceHeading);
            lineUpConesFarHighPlaceHeading = -Math.abs(lineUpConesFarHighPlaceHeading);
            CONESAndParkHeading = Math.abs(CONESAndParkHeading);
        }
        else{
//            xPosCONES = Math.abs(xPosCONES);
//            placeTanHighJunct = Math.toRadians(210);
//            placeTanMidAndFarHighJuct = Math.toRadians(155);
//            tanForPreloadPlace = Math.abs(Math.toRadians(264.05));
            placeTanHighJunct = Math.abs(placeTanHighJunct);
            placeTanMidAndFarHighJuct = Math.abs(placeTanMidAndFarHighJuct);
            tanForPreloadPlace = Math.abs(tanForPreloadPlace);
            tanForConesPlacFarHigh = Math.abs(tanForConesPlacFarHigh);


            //makes all y poses negative
            yPosAllParks = Math.abs(yPosAllParks);
            yPosParkBack = Math.abs(yPosParkBack);
            //6
            yPosCONES = Math.abs(yPosCONES);
            yPosLineUpPlacePreload = Math.abs(yPosLineUpPlacePreload);
            yPosPlaceMid = Math.abs(yPosPlaceMid);
            yPosPlaceMidPreload = Math.abs(yPosPlaceMidPreload);
            yPosPlaceFarHigh = Math.abs(yPosPlaceFarHigh);
            yPosPlacePosHigh = Math.abs(yPosPlacePosHigh);
            yPosStartingPosition = Math.abs(yPosStartingPosition);
            yPosLineUpPlacePreload = Math.abs(yPosLineUpPlacePreload);
            yPosLineUpConesFarHighPlace = Math.abs(yPosLineUpConesFarHighPlace);
            yPosPlacePosHighPreload = Math.abs(yPosPlacePosHighPreload);
            yPosLineUpPickUpConePreloadMidtonomous = Math.abs(yPosLineUpPickUpConePreloadMidtonomous);

            //35
//            xPosStartingPosition = 37;

            //makes necessary headings negative
            placePosHighHeading = Math.abs(placePosHighHeading);
            placePosMidAndFarHighHeading = -Math.abs(placePosMidAndFarHighHeading);
            startingPositionAndLineUpPlaceHeading = Math.abs(startingPositionAndLineUpPlaceHeading);
            lineUpConesFarHighPlaceHeading = Math.abs(lineUpConesFarHighPlaceHeading);
            CONESAndParkHeading = -Math.abs(CONESAndParkHeading);
        }

        //Start defining pose2d's
        //Start pose
        startingPosition = new Pose2d(xPosStartingPosition, yPosStartingPosition, startingPositionAndLineUpPlaceHeading);

        //Place poses
        placePosMid = new Pose2d(xPosPlaceMid, yPosPlaceMid, placePosMidAndFarHighHeading);
        placePosHigh = new Pose2d(xPosPlacePosHigh, yPosPlacePosHigh, placePosHighHeading);
        placePosFarHigh = new Pose2d(xPosPlaceFarHigh, yPosPlaceFarHigh, placePosMidAndFarHighHeading);
        placePosHighPreload = new Pose2d(xPosPlacePosHighPreload, yPosPlacePosHighPreload, placePosHighHeading);
        placePosPreloadMidtonomous = new Pose2d(xPosPlaceMidPreload, yPosPlaceMidPreload, CONESAndParkHeading);
        lineUpPickUpConePreloadMidtonomous = new Pose2d(xPosLineUpPickUpConePreloadMidtonomous, yPosLineUpPickUpConePreloadMidtonomous, CONESAndParkHeading);

                //CONES pose
        CONES = new Pose2d(xPosCONES, yPosCONES, CONESAndParkHeading);

        //Line up CONES far high place pose
        lineUpConesFarHighPlace = new Pose2d(xPosLineUpConesFarHighPlace, yPosLineUpConesFarHighPlace, lineUpConesFarHighPlaceHeading);

        //Park poses
        park1 = new Pose2d(xPosPark1, yPosAllParks, CONESAndParkHeading);
        park2 = new Pose2d(xPosPark2, yPosAllParks, CONESAndParkHeading);
        park3 = new Pose2d(xPosPark3, yPosAllParks, CONESAndParkHeading);

        park1Far = new Pose2d(xPosPark1, yPosParkBack, CONESAndParkHeading);
        park2Far = new Pose2d(xPosPark2, yPosParkBack, CONESAndParkHeading);
        park3Far = new Pose2d(xPosPark3, yPosParkBack, CONESAndParkHeading);

        //Line up place pose
        lineUpPlacePreload = new Pose2d(xPosLineUpPlacePreload, yPosLineUpPlacePreload, startingPositionAndLineUpPlaceHeading);


        //Set estimated pose to starting position
        drive.setPoseEstimate(startingPosition);

        placeConePreload = drive.trajectoryBuilder(startingPosition, true)
                .addDisplacementMarker(distanceBeforeColor, ()-> jacksEncodedCock(h3, armPowerUp))
                .addDisplacementMarker(distanceBeforeCockSlide, ()-> jacksEncodedShaft(armServoReadyPlace))

//                .splineToSplineHeading(lineUpPlacePreload, Math.toRadians(262.69))
//                .splineToSplineHeading(placePosHigh, placeTanHighJunct/*Math.toRadians(241.16)*/)
//                .build();
                .splineToSplineHeading(placePosHighPreload, tanForPreloadPlace, SampleMecanumDrive.getVelocityConstraint(velocityForSpeedUp, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(accelerationForSpeedUp))
                .build();

        placeConePreloadMidtonomous = drive.trajectoryBuilder(startingPosition, true)
                .addDisplacementMarker(distanceBeforeColor, ()-> jacksEncodedCock(h2, armPowerUp))
                .addDisplacementMarker(distanceBeforeCockSlide, ()-> jacksEncodedShaft(armServoReadyPlace))
                .splineToSplineHeading(placePosPreloadMidtonomous, Math.toRadians(94.71))
                .build();

        pickUpConePreloadMidtonomous = drive.trajectoryBuilder(placePosPreloadMidtonomous)
                .splineToSplineHeading(lineUpPickUpConePreloadMidtonomous, Math.toRadians(19.44))
                .splineToSplineHeading(CONES, conesTan)
                .build();

        backToStart1 = drive.trajectoryBuilder(park1, true)
                .addDisplacementMarker(() -> jacksEncodedCock(slideReset, armPowerDown))
                .lineToLinearHeading(park2)
                .build();

        backToStart3 = drive.trajectoryBuilder(park3)
                .addDisplacementMarker(() -> jacksEncodedCock(slideReset, armPowerDown))
                .lineToLinearHeading(park2)
                .build();

        backToStart = drive.trajectoryBuilder(park2)
                .lineToLinearHeading(startingPosition)
                .build();

    }
    //Method to set the slides to a certain position
    public void jacksEncodedCock(int pos, double power) {
        armMotor1.setTargetPosition(pos);
        armMotor2.setTargetPosition(pos);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(power);
        armMotor2.setPower(power);
    }

    public void jacksEncodedShaft(double pos){
        armServo1.setPosition(pos);
        armServo2.setPosition(pos);
    }
    public Command jacksEncodedCommand(double pos){
        armServo1.setPosition(pos);
        armServo2.setPosition(pos);
        return null;
    }
    public Command jacksEncodedCommandCock(int pos, double power){
        armMotor1.setTargetPosition(pos);
        armMotor2.setTargetPosition(pos);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(power);
        armMotor2.setPower(power);
        return null;
    }
    public void openGrabber(){
        grabberServo1.setPosition(grabberOpen1);
    }
    public void closeGrabber(){
        grabberServo1.setPosition(grabberClosed1);
    }
    public void parkGabber(){grabberServo1.setPosition(grabberPark);}
    public void rotateFirst(){
        rotatorServo.setPosition(rotatorDist1);
    }
    public void rotateNext(){
        rotatorServo.setPosition(rotatorPosition);
    }
    public void placeConeAuto(){
        jacksEncodedShaft(armServoPlacePos);

        myOpMode.sleep(sleepTimeArmDown);

        openGrabber();

        myOpMode.sleep(sleepTimeGrabberOpen);
    }
    public void pickUpConeAuto(){
        closeGrabber();

        myOpMode.sleep(sleepTimeGrabberClose);

        jacksEncodedShaft(armServoReadyPlace);

        myOpMode.sleep(sleepTimeArmDown);
    }
    public void pickUpConeAutoGetOutTheWay(){
        jacksEncodedShaft(armServoPickUpPos);

        myOpMode.sleep(getOutTheWaySleepTime);

        closeGrabber();

        myOpMode.sleep(sleepTimeGrabberCloseGetOutTheWay);

        jacksEncodedShaft(armServoReadyPlace);

        myOpMode.sleep(sleepTimeArmDown);
    }
    public Trajectory reAdjustPlace(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build();
    }
//    public Trajectory reAdjustPlaceGetOutTheWay(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
//        return drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(endPose)
//                .build();
//    }
    public Trajectory adjustOtherPlaces(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, double endTan, int armHeight) {
        return drive.trajectoryBuilder(startPose, true)
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(armHeight, armPowerUp);
                })
                .addTemporalMarker(0.8, this::rotateNext)
                .splineToLinearHeading(endPose, endTan)
                .build();
    }
    public boolean isTooFarPlace(double dist) {
        return dist > distanceForGoodPlace && dist < noPlaceDistance;
    }
    public boolean isTooClosePlace(double dist) {
        return dist < distanceForBadPlace;
    }
    public boolean isTooFarOtherPlaces(double dist) {
        return dist > distanceForGoodPlace2 && dist < noPlaceDistance;
    }
    public boolean isTooCloseOtherPlaces(double dist) {
        return dist < distanceForBadPlace2;
    }

    public Trajectory adjustPickUp(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, int ch) {
        return drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(ch, armPowerDown);
                    rotateFirst();
                })
                .addDisplacementMarker(distanceBeforeOpenGrabber, this::openGrabber)
//                .addTemporalMarker(0.4, () -> {
//
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(ch, armPowerDown);
//                    rotateFirst();
//                })
//                .addTemporalMarker(0.8, this::openGrabber)
                .splineToLinearHeading(endPose, conesTan)
                .build();
    }
    public Trajectory adjustPickUpAfterPlaceFarHigh(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, int ch) {
        return drive.trajectoryBuilder(startPose)
//                .addDisplacementMarker(() -> {
//
//                })
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(ch, armPowerDown);
                    rotateFirst();
                })
                .addDisplacementMarker(distanceBeforeOpenGrabber, this::openGrabber)
//                .addTemporalMarker(0.4, () -> {
//                    closeGrabber();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(ch, armPowerDown);
//                    rotateFirst();
//                })
//                .addTemporalMarker(0.8, this::openGrabber)

                .splineToSplineHeading(lineUpConesFarHighPlace, tanForConesPlacFarHigh)
                .splineToLinearHeading(endPose, conesTan)
                .build();
    }
    public Trajectory adjustPickUpGetOutTheWay(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, int ch) {
        return drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(ch, armPowerDown);
                    rotateFirst();
                })
                .addDisplacementMarker(distanceBeforeOpenGrabber, this::openGrabber)
//                .addTemporalMarker(0.4, () -> {
//                    closeGrabber();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(ch, armPowerDown);
//                    rotateFirst();
//                })
//                .addTemporalMarker(0.8, this::openGrabber)

                .splineToSplineHeading(endPose, conesTan)
                .build();
    }
    public Trajectory adjustPickUpGetOutTheWayAfterPlaceFarHigh(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, int ch) {
        return drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(ch, armPowerDown);
                    rotateFirst();
                })
                .addDisplacementMarker(distanceBeforeOpenGrabber, this::openGrabber)
//                .addTemporalMarker(0.4, () -> {
//                    closeGrabber();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(ch, armPowerDown);
//                    rotateFirst();
//                })
//                .addTemporalMarker(0.8, this::openGrabber)

                .splineToSplineHeading(lineUpConesFarHighPlace, tanForConesPlacFarHigh)
                .splineToSplineHeading(endPose, conesTan)
                .build();
    }
    public Trajectory reAdjustPickUp(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build();
    }
    public Trajectory adjustPark1(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, boolean isReversed) {
        return drive.trajectoryBuilder(startPose, isReversed)
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(slideReset, armPowerDown);
                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(slideReset, armPowerDown);
////                    rotateFirst();
//                })
//                .addTemporalMarker(0.8, this::closeGrabber)
                .splineToLinearHeading(endPose, conesTan)
                .build();
    }
    public Trajectory adjustPark2(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, boolean isReversed) {
        return drive.trajectoryBuilder(startPose, isReversed)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(900, 1);
                })
                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
                    jacksEncodedShaft(armServoPickUpPos);
                    jacksEncodedCock(slideReset, armPowerDown);
                })
//                .addDisplacementMarker(() -> {
//                    jacksEncodedShaft(armServoReadyPlace);
//                })
//                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
//                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
//                    jacksEncodedCock(slideReset, armPowerDown);
//                    jacksEncodedShaft(armServoPickUpPos);
//                })
//                .addTemporalMarker(0.4, () -> {
//                    closeGrabber();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(slideReset, armPowerDown);
////                    rotateFirst();
//                })
                .lineToLinearHeading(
                        endPose,
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
    }
    public Trajectory adjustPark3(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
//                .addDisplacementMarker(distanceBeforeCloseGrabber, this::closeGrabber)
//                .addDisplacementMarker(distanceBeforeSlidesAndShit, ()-> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(slideReset, armPowerDown);
//                })
//                .addTemporalMarker(0.4, () -> {
//                    closeGrabber();
//                })
//                .addTemporalMarker(0.5, () -> {
//                    jacksEncodedShaft(armServoPickUpPos);
//                    jacksEncodedCock(slideReset, armPowerDown);
////                    rotateFirst();
//                })
                .lineToLinearHeading(endPose)
                .build();
    }
    public void getParkAndEndPlace(SampleMecanumDrive drive, int zone, Pose2d startPose, Pose2d park1, Pose2d park2, Pose2d park3) {
        if (zone == 0) {
            drive.followTrajectory(adjustPark1(drive, startPose, park1, false));
        } else {
            drive.followTrajectory(adjustPark2(drive, startPose, park2, false));
            if (zone == 2) {
                drive.followTrajectory(adjustPark3(drive, park2, park3));
            }
        }
        myOpMode.requestOpModeStop();
    }
    public void getParkAndEndPickUp(SampleMecanumDrive drive, int zone, Pose2d startPose, Pose2d park1, Pose2d park2, Pose2d park3) {
        if (zone == 1){
            drive.followTrajectory(adjustPark2(drive, startPose, park2, false));
        } else if (zone == 2){
            drive.followTrajectory(adjustPark3(drive, startPose, park3));
        }
        myOpMode.requestOpModeStop();
    }

    public void getParkAndEndPlaceFar(SampleMecanumDrive drive, int zone, Pose2d startPose, Pose2d park1, Pose2d park2, Pose2d park3) {
        drive.followTrajectory(adjustPark2(drive, startPose, park2, false));
        if (zone == 0) {
            drive.followTrajectory(adjustPark1(drive, startPose, park1, false));
        }
        else if (zone == 2) {
            drive.followTrajectory(adjustPark3(drive, park2, park3));
        }
        myOpMode.requestOpModeStop();
    }
    
    //rotate next method for teleop
    public void theOneWhoSpins() {
        if (rotatorServo.getPosition() >= rotatorPosition - 0.1 && rotatorServo.getPosition() <= rotatorPosition + 0.1) {
            rotateFirst();
        }
        else {
            rotateNext();
        }
    }

    public void theOneWhoGrabs(){
        if (grabberServo1.getPosition() >= grabberClosed1 - 0.1 && grabberServo1.getPosition() <= grabberClosed1 + 0.1) {
            openGrabber();
        }
        else{
            closeGrabber();
        }
    }
    public Pose2d getNewPlaceHigh(double dispX, double dispY, boolean isLeftSide) {
        if(isLeftSide){
            return new Pose2d(placePosHigh.getX() - dispX, placePosHigh.getY() - dispY, placePosHigh.getHeading());
        }
        else{
            return new Pose2d(placePosHigh.getX() - dispX, placePosHigh.getY() + dispY, placePosHigh.getHeading());
        }
    }
    public Pose2d getNewHeadingPlaceHigh(double headingToPole, boolean isLeftSide) {
        if(isLeftSide){
            return new Pose2d(placePosHigh.getX(), placePosHigh.getY(), headingToPole);
        }
        else{
            return new Pose2d(placePosHigh.getX(), placePosHigh.getY(), headingToPole);
        }
    }

    public Pose2d getNewPlaceMid(double dispX, double dispY, boolean isLeftSide) {
        if(isLeftSide){
            return new Pose2d(placePosMid.getX() - dispX, placePosMid.getY() + dispY, placePosMid.getHeading());
        }
        else{
            return new Pose2d(placePosMid.getX() - dispX, placePosMid.getY() - dispY, placePosMid.getHeading());
        }

    }
    public Pose2d getNewPlaceFarHigh(double dispX, double dispY, boolean isLeftSide) {
        if(isLeftSide){
            return new Pose2d(placePosFarHigh.getX() - dispX, placePosFarHigh.getY() + dispY, placePosFarHigh.getHeading());
        }
        else{
            return new Pose2d(placePosFarHigh.getX() - dispX, placePosFarHigh.getY() - dispY, placePosFarHigh.getHeading());
        }
    }
    public boolean giveAnalConsent(double dist){
        return (isTooClosePlace(dist) || isTooFarPlace(dist)) && myOpMode.opModeIsActive();
    }
    public boolean giveAnalConsentAgain(double dist){
        return (isTooCloseOtherPlaces(dist) || isTooFarOtherPlaces(dist)) && myOpMode.opModeIsActive();
    }
    public boolean givePenalConsent(double dist){
        return (dist > distanceForGoodPickUp || dist < distanceForBadPickUp) && myOpMode.opModeIsActive();
    }
    public double getDispPoles(double dist, boolean multiplyByRadical3, boolean firstTime){
        if(firstTime){
            if(multiplyByRadical3){
                return ((distanceForIdealPlace - dist) / 2) * Math.sqrt(3.0);
            }
            return ((distanceForIdealPlace - dist) / 2);
        }
        else{
            if(multiplyByRadical3){
                return ((distanceForIdealPlace2 - dist) / 2) * Math.sqrt(3.0);
            }
            return ((distanceForIdealPlace2 - dist) / 2);
        }
    }
    public double getDispCones(double dist){
        return -(distanceForIdealPickUp - dist);
    }

    public Pose2d getNewCONES(double dispX){
        return new Pose2d(CONES.getX() + dispX, CONES.getY(), CONES.getHeading());
    }
}
