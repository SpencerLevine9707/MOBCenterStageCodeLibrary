package org.firstinspires.ftc.teamcode.conceptCode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerStuff.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardwareMaps.openCVTestLeft;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


//@Config
@Autonomous(group="drive")
@Disabled
public class straightTest2 extends LinearOpMode {


    public static boolean doesPoleAlignmentAdjust = false;
    public static boolean doesConeAdjust = true;
    public static boolean doesPoleDistanceAdjust = false;
    public static boolean doesEndEarly = false;
    public static double parkEarlyPlace = 25;
    public static double parkEarlyCones = 25;

    public static double alignmentAdjustDist = 3;


    public static double anchorPointX = 170;
    public static double anchorPointY = 90;

    public static int boxWidth = 37;

    public static int boxHeight = 60;

    private ElapsedTime runtime = new ElapsedTime();
    //Define Gyro

    // private BNO055IMU imu;


    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static openCVTestLeft.OpenCV_Pipeline pipeline;


    public DcMotor frontLeft, frontRight, backLeft, backRight, armMotor1, armMotor2;
    public Servo spencerLikesKids, grabberServo1, grabberServo2, rotatorServo, armServo1, armServo2;

    public VoltageSensor batteryVoltageSensor;

    public DistanceSensor jacksTip, jacksAsshole;

    public static double servoOffset = 0;

    public static double armServoPlacePos1 = 0.31 + servoOffset;

    public static double armServoPlacePos2 = 0.97;

    public static double armUpPos = 0.64 + servoOffset;

    public static double armBeforePlace = 0.8 + servoOffset;

    public static double holdSpeed = -0.15;

    public static double grabberClosed1 = 0.45;//change
    public static double grabberOpen1 = 0.6;


    public static double rotatorPosition = 0.22;
    public static double rotatorDist1 = 0.88;

//    int color = 0;

    public static int armMotPos = 0;
    public static int hover = 140;
    public static int h1 = 900;
    public static int h2 = 1150;
    public static int h3 = 2000;

    public static double xDisplacePushSignal = 0;
    public static double yDisplacePushSignal = 0;

    public static double xDisplaceCones = 1.86;

    public static double yDisplaceCones = -0.8;

    public static double xDisplaceStart = 0;

    public static double yDisplaceStart = 0;

    public static double xDisplacePlace = 0.2;
    public static double yDisplacePlace = -1;

    public static int sleepTimeArmDown = 250;

    public static int sleepTimeGrabberOpen = 125;

    public static int sleepTimeGrabberClose = 125;

    public static int sleepTimeGrabberCloseStart = 50;

    public static int accelerationForSlowdown = 40;//45
    public static int velocityForSlowdown = 30;//25

    public static double spencerLikesKidsPosUp = 0.4;
    public static double spencerLikesKidsPosDown = 0.9;

    public static int ch2 = 470;
    public static int ch3 = 350;
    public static int ch4 = 220;
    public static int ch5 = 130;
    public static int ch6 = 0;

    public static double conesTan = Math.toRadians(0);

    public static double placeTan2 = Math.toRadians(210);

    public static double distanceBeforeColor = 18.4;

    //    public static double distanceForGoodPickUp = 2.7;
//
//    public static double distanceForBadPickUp = 2.2;
//
//    public static double distanceForGoodPlace = 3.75;
//
//    public static double distanceForBadPlace = 2.75;
//
//    public static double distanceForGoodPlace2 = 3.45;
//
//    public static double distanceForBadPlace2 = 2.45;\
    public static double distanceForGoodPickUp = 2.4;//2.7

    public static double distanceForBadPickUp = 1.9;//2.2

    public static double distanceForGoodPlace = 3;

    public static double distanceForBadPlace = 2.5;

    public static double distanceForGoodPlace2 = 2.25;//3.45

    public static double distanceForBadPlace2 = 1.75;//2.45

    public static double distanceForIdealPlace = (distanceForGoodPlace + distanceForBadPlace) / 2;
    public static double distanceForIdealPlace2 = (distanceForGoodPlace2 + distanceForBadPlace2) / 2;
    public static double distanceForIdealPickUp = (distanceForGoodPickUp + distanceForBadPickUp) / 2;

    public static double noConesDistance = 10;
    public static double noPlaceDistance = 10;

    public static Pose2d placePosOG;

    public static Pose2d CONESOG;

    public static double errorCorrectionX = 10;

    public static double errorCorrectionY = 10;







    @Override
    public void runOpMode() {

        double alignmentDispX = (alignmentAdjustDist / 2) * Math.sqrt(3.0);
        double alignmentDispY = -alignmentAdjustDist / 2;
        int matchStart = hardwareMap.appContext.getResources().getIdentifier("match_start", "raw", hardwareMap.appContext.getPackageName());

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        int zone = 0;

        boolean isRunning = true;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(36, -5.97, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(36, 9.31, Math.toRadians(90.60)))
                .lineToLinearHeading(new Pose2d(36, 26.17, Math.toRadians(91.57)))
                .lineToLinearHeading(new Pose2d(36, 41.44, Math.toRadians(93.73)))
                .lineToLinearHeading(new Pose2d(36, 58.30, Math.toRadians(90.78)))
                .build();

        armMotor1 = hardwareMap.dcMotor.get("armMotor1");

        armMotor2 = hardwareMap.dcMotor.get("armMotor2");

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        grabberServo1 = hardwareMap.get(Servo.class, "grabberServo1");
        spencerLikesKids = hardwareMap.get(Servo.class, "spencerLikesKids");

        jacksTip = hardwareMap.get(DistanceSensor.class, "jacksTip");
        jacksAsshole = hardwareMap.get(DistanceSensor.class, "jacksAsshole");

        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor1.setDirection(DcMotor.Direction.FORWARD);

        armMotor2.setDirection(DcMotor.Direction.REVERSE);

        armServo1.setDirection(Servo.Direction.REVERSE);

        Pose2d startingPosition = new Pose2d(36, 63.6, Math.toRadians(90));

        Pose2d CONES = new Pose2d(59.7  + xDisplaceCones, 12.25 + yDisplaceCones, Math.toRadians(0));

        Pose2d park1 = new Pose2d(62, 11.75, Math.toRadians(0));
        Pose2d park2 = new Pose2d(38 , 11.75, Math.toRadians(0));
        Pose2d park3 = new Pose2d(14, 11.75, Math.toRadians(0));

        Pose2d colorPosition = new Pose2d(36.5, 46, Math.toRadians(90));

        Pose2d pushSignalConeAcross = new Pose2d(36 + xDisplacePushSignal, 3.75+ yDisplacePushSignal, Math.toRadians(90));//11.75 works

        Pose2d lineUpCone1 = new Pose2d(37.15, 19.80, Math.toRadians(90.00));

        Pose2d placePos = new Pose2d(34 + xDisplacePlace, 6.9 + yDisplacePlace, Math.toRadians(30));

        CONESOG = CONES;
        placePosOG = placePos;

        drive.setPoseEstimate(startingPosition);


//        Trajectory pushConeAcross = drive.trajectoryBuilder(startingPosition)
//                .addDisplacementMarker(distanceBeforeColor, ()-> {
//                    jacksEncodedCock(h3);
//                })
//                .lineToLinearHeading(pushSignalConeAcross)
//
//                .build();

        Trajectory lineUpForCone1 = drive.trajectoryBuilder(startingPosition)
                .addDisplacementMarker(distanceBeforeColor, ()-> {
                    jacksEncodedCock(h3);
                })
                .lineToLinearHeading(lineUpCone1)
                .build();

        Trajectory pickUpCone;


        Trajectory placeCone1 = drive.trajectoryBuilder(startingPosition, true)
                .addDisplacementMarker(distanceBeforeColor, ()-> {
                    jacksEncodedCock(h3);
                })
                .splineToSplineHeading(lineUpCone1, Math.toRadians(262.69))
                .splineToSplineHeading(placePos, Math.toRadians(241.16))
                .build();


        Trajectory placeOtherCones;

        Trajectory backToStart1 = drive.trajectoryBuilder(park1, true)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                })
                .lineToLinearHeading(park2)
                .build();
        Trajectory backToStart3 = drive.trajectoryBuilder(park3)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                })
                .lineToLinearHeading(park2)
                .build();
        Trajectory backToStart = drive.trajectoryBuilder(park2)
                .lineToLinearHeading(startingPosition)
                .build();


        jacksEncodedShaft(armUpPos);

        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up pipeline
//        pipeline = new openCVTest.OpenCV_Pipeline();
//        webcam.setPipeline(pipeline);

        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        while (opModeInInit()) {
            //Telemetry readings for the HSV values in each region
            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
            telemetry.addData("Battery Voltage ", batteryVoltageSensor.getVoltage());

            if ((pipeline.HSV_Value_2[0] < 88)) {
                zone = 0;
                telemetry.addLine("Yellow");
            } else if (pipeline.HSV_Value_2[0] < 183) {
                zone = 1;
                telemetry.addLine("Cyan");
            } else {
                zone = 2;
                telemetry.addLine("Magenta");
            }

            telemetry.update();
        }

        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, matchStart);

        runtime.reset();

//        closeGrabber();
        sleep(sleepTimeGrabberCloseStart);
        rotateNext();

        //lines up for cone 1; no adjustments
//        drive.followTrajectory(lineUpForCone1);
        drive.setPoseEstimate(untitled0.start());
        drive.followTrajectorySequence(untitled0);

            telemetry.addLine("Runtime: " + runtime.seconds());
            telemetry.update();

        }


    public void jacksEncodedCock(int pos) {
        armMotor1.setTargetPosition(pos);
        armMotor2.setTargetPosition(pos);
        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor1.setPower(1);
        armMotor2.setPower(1);

    }
    public void jacksEncodedShaft(double pos){
        armServo1.setPosition(pos);
        armServo2.setPosition(pos);
    }
    public void openGrabber(){
        grabberServo1.setPosition(grabberOpen1);
    }
    public void closeGrabber(){
        grabberServo1.setPosition(grabberClosed1);
    }
    public void rotateFirst(){
        rotatorServo.setPosition(rotatorPosition);
    }
    public void rotateNext(){
        rotatorServo.setPosition(rotatorDist1);
    }
    public void placeConeAuto(){
        jacksEncodedShaft(armServoPlacePos2);

        sleep(sleepTimeArmDown);

        openGrabber();

        sleep(sleepTimeGrabberOpen);
    }
    public void pickUpConeAuto(){
        closeGrabber();

        sleep(sleepTimeGrabberClose);

        jacksEncodedShaft(armBeforePlace);

        sleep(sleepTimeArmDown);
    }
    public Trajectory reAdjustPlace(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(endPose)
                .build();
    }
    public Trajectory adjustOtherPlaces(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose, true)
                .addTemporalMarker(0.2, () -> {
                    jacksEncodedCock(h3);
                })
                .addTemporalMarker(0.7, this::rotateNext)
                .splineToLinearHeading(endPose, placeTan2)
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
    public boolean albertGoesToTheBathroomToTakeAFatShitAndFindsOutHesActuallyReallyReallyConstiptatedSoHeCantPlaceTheConesAndHasToKeepTheSamePosition(Pose2d Pos){
        return Math.abs(Pos.getX() - placePosOG.getX()) < errorCorrectionX && Math.abs(Pos.getY() - placePosOG.getY()) < errorCorrectionY;
    }
    public boolean albertGoesToTheStoreToGetMedicationsForHisConstipationOnlyToFindOutThatHeNowCannotChangeCONESPosition(Pose2d Pos){
        return Math.abs(Pos.getX() - CONESOG.getX()) < errorCorrectionX;
    }
    public Trajectory adjustPickUp(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, int ch) {
        return drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(ch);
                    rotateFirst();
                })
                .splineToLinearHeading(
                        endPose,
                        conesTan,
                        SampleMecanumDrive.getVelocityConstraint(velocityForSlowdown, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(accelerationForSlowdown))
                .build();
    }
    public Trajectory reAdjustPickUp(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(
                        endPose,
                        SampleMecanumDrive.getVelocityConstraint(velocityForSlowdown, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(accelerationForSlowdown))
                .build();
    }
    public Trajectory adjustPark1(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, boolean isReversed) {
        return drive.trajectoryBuilder(startPose, isReversed)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                    jacksEncodedShaft(armUpPos);
                })
                .splineToLinearHeading(endPose, conesTan)
                .build();
    }
    public Trajectory adjustPark2(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose, boolean isReversed) {
        return drive.trajectoryBuilder(startPose, isReversed)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                    jacksEncodedShaft(armUpPos);
                })
                .lineToLinearHeading(
                        endPose,
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
    }
    public Trajectory adjustPark3(SampleMecanumDrive drive, Pose2d startPose, Pose2d endPose) {
        return drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                    jacksEncodedShaft(armUpPos);
                })
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
        requestOpModeStop();
    }
    public void getParkAndEndPickUp(SampleMecanumDrive drive, int zone, Pose2d startPose, Pose2d park1, Pose2d park2, Pose2d park3) {
        if (zone == 1){
            drive.followTrajectory(adjustPark2(drive, startPose, park2, false));
        } else if (zone == 2){
            drive.followTrajectory(adjustPark3(drive, startPose, park3));
        }
        requestOpModeStop();
    }
}
