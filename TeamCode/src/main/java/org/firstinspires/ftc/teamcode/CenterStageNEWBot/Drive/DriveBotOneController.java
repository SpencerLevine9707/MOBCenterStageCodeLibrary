package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

@Config
@TeleOp
public class DriveBotOneController extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    SampleMecanumDrive driver;
    public ElapsedTime planeTime = new ElapsedTime(), timeForCloseLeft = new ElapsedTime(), timeForCloseRight = new ElapsedTime();
    public static double timeForPlaneWait = 0.75;
    public static double roomForErrorFlipper = 5;
    public static int maxFlipperPos = 1600;
    public static int minFlipperPos = -10;
    public static double slideSpeed = -300;
    public static int maxArmPos = 0, minArmPos = -660;
    public static boolean retractSlidesOnFlip = false;
    public boolean retractSlidesForFlipping = false;
    public static boolean slideResetOnInit = false;
    public static double slowSpeedDrive = 0.2;
    public static double divisorForSpinPower = 1.5;
    public static double timeForCloseWait = 0.75;
    public static int roomForErrorSlidePos = -8;
    public Orientation angles;



    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        if(slideResetOnInit){
            wBot.resetSlidePoses();
        }
        driver = new SampleMecanumDrive(this.hardwareMap);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        wBot.gyro.initialize(parameters);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);

        boolean a1Pressable = true;
        boolean b1Pressable = true;
        boolean x1Pressable = true;
        boolean y1Pressable = true;
        boolean dpu1Pressable = true;
        boolean dpl1Pressable = true;
        boolean dpr1Pressable = true;
        boolean dpd1Pressable = true;
        boolean lb1Pressable = true;
        boolean rb1Pressable = true;
        boolean isReserveDrive = false;
        boolean armCameDown = true;
        boolean autoCorrectorAdjust = false;

        double powerForMotors = 0;

        planeTime.reset();
        wBot.setRotatorUp();
        wBot.setCorrectorMid();
        wBot.closeGrabber();
        wBot.encodedSlipperySlides(maxArmPos, MonkeyMap.slidePowerEncoder);
        wBot.loadPlane();

        timeForCloseLeft.reset();
        timeForCloseRight.reset();

//        wBot.resetFlipperPos();
        wBot.flipUp();
        wBot.pullUpDown();

        waitForStart();

        while(opModeIsActive()){
            double heading = driver.getPoseEstimate().getHeading();

            //flipperPos stuff
            double flipperPos = wBot.flipperMotor.getCurrentPosition();
            boolean isFlipperDown = flipperPos < MonkeyMap.lowestFlipperPosDown;
            double armPos = wBot.armMotorLeft.getTargetPosition();

//            Gamepad 1 controls
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;
            double ry1 = gamepad1.right_stick_y;
            boolean y1 = gamepad1.y;

            telemetry.addLine("lx1: " + lx1 + " ly1: " + ly1 + " rx1 " + rx1);

            powerForMotors = Math.abs(lx1) + Math.abs(ly1) + Math.abs(rx1/divisorForSpinPower);
            drive.moveInTeleop(lx1, ly1, rx1, powerForMotors);


            if(y1 && planeTime.seconds() > timeForPlaneWait){
                wBot.toggleAirplane();
                planeTime.reset();
            }

            else{
                wBot.pullUpMotorRight.setPower(0);
                wBot.pullUpMotorLeft.setPower(0);
            }

//            Gamepad 2 controls
            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
//            boolean y1 = gamepad2.y;
            boolean lb1 = gamepad1.left_bumper;
            boolean rb1 = gamepad1.right_bumper;
            double rt1 = gamepad1.right_trigger;
            double lt1 = gamepad1.left_trigger;
            boolean dpu1 = gamepad1.dpad_up;
            boolean dpd1 = gamepad1.dpad_down;
            boolean dpl1 = gamepad1.dpad_left;
            boolean dpr1 = gamepad1.dpad_right;


            if(a1 && a1Pressable){
                wBot.toggleGrabber();
                timeForCloseLeft.reset();
                timeForCloseRight.reset();
            }
            if(b1 && b1Pressable && timeForCloseRight.seconds() > timeForCloseWait){
                wBot.toggleLeftGrabber();
                timeForCloseLeft.reset();
            }
            if(x1 && x1Pressable && timeForCloseLeft.seconds() > timeForCloseWait){
                wBot.toggleRightGrabber();
                timeForCloseRight.reset();
            }
            if(isFlipperDown){
                if(wBot.detectionLeftGrabber.getDistance(DistanceUnit.INCH) < MonkeyMap.distForPickUpDetect && timeForCloseLeft.seconds() > timeForCloseWait){
                    wBot.closeLeftGrabber();
                }
                if(wBot.detectionRightGrabber.getDistance(DistanceUnit.INCH) < MonkeyMap.distForPickUpDetect && timeForCloseRight.seconds() > timeForCloseWait){
                    wBot.closeRightGrabber();
                }
                autoCorrectorAdjust = false;
                if(armCameDown){
                    wBot.resetArm();
                    armCameDown = false;
                    retractSlidesForFlipping = true;
                }
                if(armPos < roomForErrorSlidePos){
                    wBot.setRotatorFlush();
                }
            }
            else{
                wBot.setAutoRotator(flipperPos);
                armCameDown = true;
                retractSlidesForFlipping = false;
                if(autoCorrectorAdjust){
                    wBot.setAutoCorrector(heading);
                }
                if (dpl1 && dpl1Pressable){
                    autoCorrectorAdjust = !autoCorrectorAdjust;
                    if(!autoCorrectorAdjust){
                        wBot.setCorrectorMid();
                    }
                }
            }


            telemetry.addLine("Grabber Servo Pos (left) " + wBot.grabberServoLeft.getPosition() + " Grabber Servo Pos (Right) " + wBot.grabberServoRight.getPosition());

            if(y1 && y1Pressable){
                wBot.setArmFirstPlace();
//                if(retractSlidesOnFlip){
//                    wBot.resetSlides();
//                }
            }
            if(Math.abs(ry1) > 0 && wBot.armMotorLeft.getTargetPosition() <= maxArmPos && wBot.armMotorLeft.getTargetPosition() >= minArmPos) {
                wBot.encodedSlipperySlides((int) (wBot.armMotorLeft.getCurrentPosition() - (slideSpeed * ry1)), MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() >= maxArmPos){
                wBot.encodedSlipperySlides(maxArmPos, MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() <= minArmPos){
                wBot.encodedSlipperySlides(minArmPos, MonkeyMap.slidePowerEncoder);
            }

            if(lb1 && lb1Pressable){
                wBot.resetArm();
            }
            if(rb1 && rb1Pressable){
                wBot.setRotatorFlush();
            }


            telemetry.addLine("armMotorLeftPow: " + wBot.armMotorLeft.getPower());
//            if(Math.abs(ry2) > 0){
//                wBot.rotatorServo.setPosition(wBot.rotatorServo.getPosition() + MonkeyMap.rotatorServoSpeed * ry2);
//            }
            if(rt1 > 0 && flipperPos - roomForErrorFlipper <= maxFlipperPos){
                wBot.setFlipperPos((int) (flipperPos - (MonkeyMap.flipperMotorSpeed * rt1)), MonkeyMap.flipperPower);
                if(retractSlidesOnFlip || retractSlidesForFlipping){
                    wBot.resetSlides();
                }
            }
            else if(lt1 > 0 && flipperPos + roomForErrorFlipper >= minFlipperPos){
                wBot.setFlipperPos((int) (flipperPos + (MonkeyMap.flipperMotorSpeed * lt1)), MonkeyMap.flipperPower);
                if(retractSlidesOnFlip || retractSlidesForFlipping){
                    wBot.resetSlides();
                }
            }

            if(flipperPos <= minFlipperPos){
                wBot.setFlipperPos(minFlipperPos, MonkeyMap.flipperPower);
            }
            if(flipperPos >= maxFlipperPos){
                wBot.setFlipperPos(maxFlipperPos, MonkeyMap.flipperPower);
            }


            if(dpu1 && dpu1Pressable){
                wBot.setCorrectorMid();
            }
//            if(dpd1 && dpd1Pressable){
//                driver.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
//            }

            if(wBot.leftGrabberOpen && wBot.rightGrabberOpen){
                wBot.grabberIsOpen = true;
            }
            if(!wBot.leftGrabberOpen && !wBot.rightGrabberOpen){
                wBot.grabberIsOpen = false;
            }

            a1Pressable = !a1;
            dpu1Pressable = !dpu1;
            rb1Pressable = !rb1;
            lb1Pressable = !lb1;
            x1Pressable = !x1;
            b1Pressable = !b1;
            a1Pressable = !a1;
            y1Pressable = !y1;
            dpu1Pressable = !dpu1;
            dpd1Pressable = !dpd1;
            dpl1Pressable = !dpl1;
            dpr1Pressable = !dpr1;

//            telemetry.addData("Heading", angles.firstAngle);
//            telemetry.addData("Roll", angles.secondAngle);
//            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.addLine("slidesPos (left): " + wBot.armMotorLeft.getCurrentPosition());
            telemetry.addLine("Rotator Pos: " + wBot.rotatorServo.getPosition());
//            telemetry.addLine("Conveyor motor pow: " + wBot.conveyerMotor.getPower());
            telemetry.addLine("Flipper Motor Pos: " + flipperPos);
            telemetry.addLine("Flipper Motor Targ Pos: " + wBot.flipperMotor.getTargetPosition());
            telemetry.addLine("ly2 is: " + ly1);
            telemetry.addLine("rx2 is: " + rx1);
            telemetry.addLine("ry2 is: " + ry1);
            telemetry.addLine("Corrector Servo pos is " + wBot.correctorServo.getPosition());
            telemetry.addData("Pull Up Servo Pos ", wBot.pullUpServoLeft.getPosition());


            telemetry.update();
        }
    }

}

