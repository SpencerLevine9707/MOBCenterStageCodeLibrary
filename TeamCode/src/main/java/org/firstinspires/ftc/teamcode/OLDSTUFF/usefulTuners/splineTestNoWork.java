








package org.firstinspires.ftc.teamcode.OLDSTUFF.usefulTuners;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Config
@Autonomous(group="drive")
@Disabled
public class splineTestNoWork extends LinearOpMode {

    public DcMotor frontLeft;

    public DcMotor frontRight;

    public DcMotor backLeft;

    public DcMotor backRight;

    public DcMotor armMotor1;

    public DcMotor armMotor2;

    public ColorSensor color_sensor;

    private ElapsedTime runtime = new ElapsedTime();



    public Servo grabberServo1;
    public Servo grabberServo2;
    public Servo rotatorServo;
    public Servo armServo1;
    public Servo armServo2;

    public static double servoOffset = 0;

    public static double armServoResetPos = 0.15 + servoOffset;

    public static double armServoPlacePos1 = 0.31 + servoOffset;

    public static double armServoPlacePos2 = 1;

    public static double armServoOverPos = 0.96 + servoOffset;

    public static double armUpPos = 0.67 + servoOffset;

    public static double armBeforePlace = 0.8 + servoOffset;

    public static double armServoPlacePos2Cone2 = 0.25 + servoOffset;

    public static double armServoPlacePos2Cone3 = 0.22 + servoOffset;

    public static double armServoPlacePos2Cone4 = 0.21 + servoOffset;

    public static double armServoPlacePos2Cone5 = 0.195 + servoOffset;

    public static double armServoPlacePos2Cone6 = 0.165 + servoOffset;

    public static double offsetXCones = 2.75;

    public static double offsetYCones = -1.5;

    public static double offsetXPlace = 2.75;

    public static double offsetYPlace = -0.9;

    public static double holdSpeed = -0.15;

    public static double grabberClosed1 = 0.6;//change
    public static double grabberOpen1 = 0.4;


    public static double rotatorPosition = 0.91;
    public static double rotatorDist1 = 0.25;

    public static int armMotPos = 0;
    public static int hover = 140;
    public static int h1 = 900;
    public static int h2 = 1900;
    public static int h3 = 2000;

    public static double signalDisplaceY = 0;

    public static double xDisplaceColor = 0;

    public static double yDisplaceColor = 0;

    public static double xDisplaceCones = 0;

    public static double yDisplaceCones = 0;

    public static double xDisplaceStart = 0;

    public static double yDisplaceStart = 0;

    public static double xDisplacePlace = 0;

    public static double yDisplacePlace = 0;

    public static int sleepTimeArmDown = 500;

    public static int sleepTimeGrabberOpen = 150;

    public static int sleepTimeGrabberClose = 250;

    public static int sleepTimeColorSensor = 25;

    public static double ch1 = 100;
    public static double ch2 = 80;
    public static double ch3 = 60;
    public static double ch4 = 40;
    public static double ch5 = 20;
    public static double ch6 = 0;




    public static double conesTan = 15;

    public static double placeTan = 0;

    public static double splineTan1 = 0;

    public static double splineTan2 = 150;



    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //color_sensor = hardwareMap.get(ColorSensor.class, "color");

        armMotor1 = hardwareMap.dcMotor.get("armMotor1");

        armMotor2 = hardwareMap.dcMotor.get("armMotor2");

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        grabberServo1 = hardwareMap.get(Servo.class, "grabberServo1");

        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor1.setDirection(DcMotor.Direction.FORWARD);

        armMotor2.setDirection(DcMotor.Direction.REVERSE);

        armServo1.setDirection(Servo.Direction.REVERSE);
        Pose2d startingPosition = new Pose2d(36, 63.6, Math.toRadians(180));

        Pose2d CONES = new Pose2d(61.2 + xDisplaceCones, 11.75+ yDisplaceCones, Math.toRadians(0));
        Pose2d CONES2 = new Pose2d(61.2+(offsetXCones*1) + xDisplaceCones, 11.75 + (offsetYCones*1)+ yDisplaceCones, Math.toRadians(0));
        Pose2d CONES3 = new Pose2d(61.2+(offsetXCones*2)+ xDisplaceCones, 11.75 + (offsetYCones*2)+ yDisplaceCones, Math.toRadians(0));
        Pose2d CONES4 = new Pose2d(61.2+(offsetXCones*3)+ xDisplaceCones, 11.75 + (offsetYCones*3)+ yDisplaceCones, Math.toRadians(0));
        Pose2d CONES5 = new Pose2d(61.2+(offsetXCones*4)+ xDisplaceCones, 11.75 + (offsetYCones*4)+ yDisplaceCones, Math.toRadians(0));

        Pose2d park1 = new Pose2d(62, 11.75, Math.toRadians(0));
        Pose2d park2 = new Pose2d(38, 11.75, Math.toRadians(0));
        Pose2d park3 = new Pose2d(14, 11.75, Math.toRadians(0));

        Pose2d colorPosition = new Pose2d(36.5, 46, Math.toRadians(90));

        Pose2d lineUpCones2and3and4 = new Pose2d(36, 11.75, Math.toRadians(45));

        Pose2d afterPlacingAllCones = new Pose2d(36, 11.75, Math.toRadians(45));

        Pose2d pushSignalConeAcross = new Pose2d(36, 1.5, Math.toRadians(90));

        Pose2d lineUpCone1 = new Pose2d(36, 12.5, Math.toRadians(90));

        Pose2d placePosCone1 = new Pose2d(34 + xDisplacePlace, 6.9, Math.toRadians(45));
        Pose2d placePosCone2 = new Pose2d(34+(offsetXPlace*1)+ xDisplacePlace, 6.9 +(offsetYPlace*1)+ yDisplacePlace, Math.toRadians(45));
        Pose2d placePosCone3 = new Pose2d(34+(offsetXPlace*2)+ xDisplacePlace, 6.9 +(offsetYPlace*2)+ yDisplacePlace, Math.toRadians(45));
        Pose2d placePosCone4 = new Pose2d(34+(offsetXPlace*3)+ xDisplacePlace, 6.9 +(offsetYPlace*3)+ yDisplacePlace, Math.toRadians(45));
        Pose2d placePosCone5 = new Pose2d(34+(offsetXPlace*4)+ xDisplacePlace, 6.9 +(offsetYPlace*4)+ yDisplacePlace, Math.toRadians(45));
        Pose2d placePosCone6 = new Pose2d(34+(offsetXPlace*5)+ xDisplacePlace, 6.9 +(offsetYPlace*5)+ yDisplacePlace, Math.toRadians(45));

        Pose2d afterPlacingAllCone = new Pose2d(40, 11.75, Math.toRadians(0));


        drive.setPoseEstimate(startingPosition);





        //Displacement marker code:
        /*.addDisplacementMarker(() -> {

        })*/


        //Cleaned up code!

        Trajectory moveToColor = drive.trajectoryBuilder(startingPosition)
                .addDisplacementMarker(() -> {
                    closeGrabber();
                    jacksEncodedShaft(armUpPos);
                })
                .lineToLinearHeading(colorPosition)
                .build();

        Trajectory pushConeAcross = drive.trajectoryBuilder(colorPosition)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(h3);
                })
                .lineToLinearHeading(pushSignalConeAcross)
                .build();

        Trajectory lineUpForCone1 = drive.trajectoryBuilder(pushSignalConeAcross)
                .lineToLinearHeading(lineUpCone1)
                .build();

        Trajectory pickUpCone2 = drive.trajectoryBuilder(placePosCone1)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                    rotateFirst();
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(0);
                })
                .splineToLinearHeading(CONES, Math.toRadians(conesTan))
                .build();

        Trajectory pickUpCone3 = drive.trajectoryBuilder(placePosCone2)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                    rotateFirst();
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(0);
                })
                .splineToLinearHeading(CONES2, Math.toRadians(conesTan))
                .build();

        Trajectory pickUpCone4 = drive.trajectoryBuilder(placePosCone3)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                    rotateFirst();
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(0);
                })
                .splineToLinearHeading(CONES3, Math.toRadians(conesTan))
                .build();

        Trajectory pickUpCone5 = drive.trajectoryBuilder(placePosCone4)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                    rotateFirst();
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(0);
                })
                .splineToLinearHeading(CONES4, Math.toRadians(conesTan))
                .build();

        Trajectory pickUpCone6 = drive.trajectoryBuilder(placePosCone5)
                .addDisplacementMarker(() -> {
                    jacksEncodedShaft(armServoPlacePos1);
                    rotateFirst();
                })
                .addTemporalMarker(0.5, () -> {
                    jacksEncodedCock(0);
                })
                .splineToLinearHeading(CONES5, Math.toRadians(conesTan))
                .build();



        Trajectory placeCone1 = drive.trajectoryBuilder(lineUpCone1)
                .lineToLinearHeading(placePosCone1)
                .build();

        Trajectory placeCone2 = drive.trajectoryBuilder(CONES)
                .addDisplacementMarker(() -> {
                    rotateNext();
                    jacksEncodedCock(h3);
                })
                .splineToLinearHeading(placePosCone2, Math.toRadians(placeTan))
                .build();

        Trajectory placeCone3 = drive.trajectoryBuilder(CONES2)
                .addDisplacementMarker(() -> {
                    rotateNext();
                    jacksEncodedCock(h3);
                })
                .splineToLinearHeading(placePosCone3, Math.toRadians(placeTan))
                .build();

        Trajectory placeCone4 = drive.trajectoryBuilder(CONES3)
                .addDisplacementMarker(() -> {
                    rotateNext();
                    jacksEncodedCock(h3);
                })
                .splineToLinearHeading(placePosCone4, Math.toRadians(placeTan))
                .build();

        Trajectory placeCone5 = drive.trajectoryBuilder(CONES4)
                .addDisplacementMarker(() -> {
                    rotateNext();
                    jacksEncodedCock(h3);
                })
                .splineToLinearHeading(placePosCone5, Math.toRadians(placeTan))
                .build();

        Trajectory placeCone6 = drive.trajectoryBuilder(CONES5)
                .addDisplacementMarker(() -> {
                    rotateNext();
                    jacksEncodedCock(h3);
                })
                .splineToLinearHeading(placePosCone6, Math.toRadians(placeTan))
                .build();

        Trajectory parkTo2 = drive.trajectoryBuilder(placePosCone6)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                    jacksEncodedShaft(armUpPos);
                })
                .lineToLinearHeading(
                        park2,
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(120), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        Trajectory parkTo1 = drive.trajectoryBuilder(park2)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                })
                .lineToLinearHeading(park1)
                .build();

        Trajectory parkTo3 = drive.trajectoryBuilder(park2)
                .addDisplacementMarker(() -> {
                    jacksEncodedCock(0);
                })
                .lineToLinearHeading(park3)
                .build();

        Trajectory spline1 = drive.trajectoryBuilder(startingPosition)
            .splineToLinearHeading(pushSignalConeAcross, Math.toRadians(conesTan))
                    .build();

        Trajectory spline2 = drive.trajectoryBuilder(spline1.end())
                .splineToLinearHeading(CONES, Math.toRadians(splineTan1))
                .build();

        Trajectory spline3 = drive.trajectoryBuilder(spline2.end())
                .splineToLinearHeading(park3, Math.toRadians(placeTan))
                .build();

        Trajectory spline4 = drive.trajectoryBuilder(spline3.end())
                .splineToLinearHeading(startingPosition, Math.toRadians(splineTan2))
                .build();







        waitForStart();

        //WORKS!!!>???????


        closeGrabber();

        sleep(sleepTimeGrabberClose);

        jacksEncodedShaft(armUpPos);

        rotateNext();







        while (opModeIsActive()){
            telemetry.update();

            drive.followTrajectory(spline1);

            drive.followTrajectory(spline2);

            drive.followTrajectory(spline3);

            drive.followTrajectory(spline4);
        }
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


}

