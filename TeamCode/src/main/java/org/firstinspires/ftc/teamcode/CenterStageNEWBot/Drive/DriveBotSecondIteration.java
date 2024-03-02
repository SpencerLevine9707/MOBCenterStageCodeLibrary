package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

@Config
@TeleOp
public class DriveBotSecondIteration extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    SampleMecanumDrive driver;
    public ElapsedTime planeTime = new ElapsedTime(), timeForCloseLeft = new ElapsedTime(), timeForCloseRight = new ElapsedTime();
    public static double timeForPlaneWait = 0.75;
    public static double roomForErrorFlipper = 5;
    public static int maxFlipperPos = 1600;
    public static int minFlipperPos = 0;
    public static double slideSpeed = -300;
    public static int maxArmPos = 0, minArmPos = -660;
    public static boolean retractSlidesOnFlip = false;
    public boolean retractSlidesForFlipping = false;
    public static boolean slideResetOnInit = false;
    public static double slowSpeedDrive = 0.2;
    public static double divisorForSpinPower = 1;
    public static double timeForCloseWait = 0.85;
    public static int roomForErrorSlidePos = -8;
    public static int rCloseGrabber = 255, gCloseGrabber = 0, bCloseGrabber = 0;
    public static int rOpenGrabber = 0, gOpenGrabber = 255, bOpenGrabber = 0;
    public static int durationToKeepColor = 100000;
    public static double maxMultiplier = 0.75;

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

        boolean a2Pressable = true;
        boolean b2Pressable = true;
        boolean x2Pressable = true;
        boolean y2Pressable = true;
        boolean dpu2Pressable = true;
        boolean dpl2Pressable = true;
        boolean dpr2Pressable = true;
        boolean dpd2Pressable = true;
        boolean lb2Pressable = true;
        boolean rb2Pressable = true;
        boolean a1Pressable = true;
        boolean dpr1Pressable = true;
        boolean dpu1Pressable = true;
        boolean dpd1Pressable = true;
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
        driver.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while(opModeIsActive()){
            double heading = Math.toDegrees(driver.getPoseEstimate().getHeading());

            //flipperPos stuff
            double flipperPos = wBot.flipperMotor.getTargetPosition();
            double flipperPos2 = wBot.flipperMotor.getCurrentPosition();
            boolean isFlipperDown = flipperPos < MonkeyMap.lowestFlipperPosDown;
            int armPos = wBot.armMotorLeft.getTargetPosition();

//            Gamepad 1 controls
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;
            boolean y1 = gamepad1.y;
            boolean lsb1 = gamepad1.left_stick_button;
            boolean rsb1 = gamepad1.right_stick_button;
            boolean dpr1 = gamepad1.dpad_right;
            boolean rb1 = gamepad1.right_bumper, lb1 = gamepad1.left_bumper;
            double rt1 = gamepad1.right_trigger, lt1 = gamepad1.left_trigger;
            boolean a1 = gamepad1.a;
            boolean dpu1 = gamepad1.dpad_up;
            boolean dpd1 = gamepad1.dpad_down;

            telemetry.addLine("lx1: " + lx1 + " ly1: " + ly1 + " rx1 " + rx1);

            double powerFromTriggers = Math.abs(lx1) + Math.abs(ly1) + Math.abs(rx1/divisorForSpinPower);

//            drive.moveInTeleop(-lx1, -ly1, rx1, powerFromTriggers);
            if(rb1){
                powerForMotors = slowSpeedDrive;
            }
            else{
               powerForMotors = powerFromTriggers;
            }
//            if(a1 && a1Pressable){
//                isReserveDrive = !isReserveDrive;
//            }

            if(isReserveDrive){
                drive.moveInTeleop(-lx1, -ly1, rx1, powerForMotors);
            }
            else{
                drive.moveInTeleop(lx1, ly1, rx1, powerForMotors*maxMultiplier);
            }

            if(y1 && planeTime.seconds() > timeForPlaneWait){
                wBot.toggleAirplane();
                planeTime.reset();
            }

//            if(lsb1 && rsb1 && planeTime.seconds() > timeForPlaneWait){
//                wBot.shootPlane();
//                planeTime.reset();
//            }
            if(lt1 > 0){
                wBot.pullUpMotorRight.setPower(lt1);
                wBot.pullUpMotorLeft.setPower(-lt1);
            }
            else if(rt1 > 0){
                wBot.pullUpMotorRight.setPower(-rt1);
                wBot.pullUpMotorLeft.setPower(rt1);
            }
            else{
                wBot.pullUpMotorRight.setPower(0);
                wBot.pullUpMotorLeft.setPower(0);
            }
            if(dpu1 && dpu1Pressable){
                wBot.pullUpUp();
            }
            if(dpd1 && dpd1Pressable){
                wBot.pullUpDown();
            }
            if(dpr1 && dpr1Pressable){
                wBot.pullUpServoRight.setPosition(MonkeyMap.airplaneAngle);
                wBot.pullUpServoLeft.setPosition(MonkeyMap.airplaneAngle);
            }

//            Gamepad 2 controls
            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;
            boolean lb2 = gamepad2.left_bumper;
            boolean rb2 = gamepad2.right_bumper;
            double rt2 = gamepad2.right_trigger;
            double lt2 = gamepad2.left_trigger;
            boolean dpu2 = gamepad2.dpad_up;
            boolean dpd2 = gamepad2.dpad_down;
            boolean dpl2 = gamepad2.dpad_left;
            boolean dpr2 = gamepad2.dpad_right;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            double rx2 = gamepad2.right_stick_x;


            if(a2 && a2Pressable){
                wBot.toggleGrabber();
                if(isFlipperDown){
                    timeForCloseLeft.reset();
                    timeForCloseRight.reset();
                }
            }
            if(b2 && b2Pressable && timeForCloseLeft.seconds() > timeForCloseWait){
                wBot.toggleLeftGrabber();
                if(isFlipperDown){
                    timeForCloseLeft.reset();
                }
            }
            if(x2 && x2Pressable && timeForCloseRight.seconds() > timeForCloseWait){
                wBot.toggleRightGrabber();
                if(isFlipperDown){
                    timeForCloseRight.reset();
                }
            }
            if(isFlipperDown){
                if(wBot.detectionLeftGrabber.getDistance(DistanceUnit.INCH) < MonkeyMap.distForPickUpDetect && timeForCloseLeft.seconds() > timeForCloseWait){
                    if(wBot.leftGrabberOpen){
                        timeForCloseLeft.reset();
                    }
                    wBot.closeLeftGrabber();
                }
                if(wBot.detectionRightGrabber.getDistance(DistanceUnit.INCH) < MonkeyMap.distForPickUpDetect && timeForCloseRight.seconds() > timeForCloseWait){
                    if(wBot.rightGrabberOpen){
                        timeForCloseRight.reset();
                    }
                    wBot.closeRightGrabber();
                }
                autoCorrectorAdjust = false;
                if(armCameDown){
//                    wBot.resetArm();
//                    wBot.setRotatorUp();
                    armCameDown = false;
                    retractSlidesForFlipping = true;
                }
                if(armPos < roomForErrorSlidePos){
                    wBot.setAutoRotatorGrabber(armPos);
                }
            }
            else{
                wBot.setAutoRotator(flipperPos);
                armCameDown = true;
                retractSlidesForFlipping = false;
                if(autoCorrectorAdjust){
                    wBot.setAutoCorrector(heading);
                }
                if (dpl2 && dpl2Pressable){
                    autoCorrectorAdjust = !autoCorrectorAdjust;
                    if(!autoCorrectorAdjust){
                        wBot.setCorrectorMid();
                    }
                }
            }


            telemetry.addLine("Grabber Servo Pos (left) " + wBot.grabberServoLeft.getPosition() + " Grabber Servo Pos (Right) " + wBot.grabberServoRight.getPosition());

            if(y2 && y2Pressable){
                wBot.setArmFirstPlace();
//                if(retractSlidesOnFlip){
//                    wBot.resetSlides();
//                }
            }
            if(Math.abs(ly2) > 0 && wBot.armMotorLeft.getTargetPosition() <= maxArmPos && wBot.armMotorLeft.getTargetPosition() >= minArmPos) {
                wBot.encodedSlipperySlides((int) (wBot.armMotorLeft.getCurrentPosition() - (slideSpeed * ly2)), MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() >= maxArmPos){
                wBot.encodedSlipperySlides(maxArmPos, MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() <= minArmPos){
                wBot.encodedSlipperySlides(minArmPos, MonkeyMap.slidePowerEncoder);
            }

            if(lb2 && lb2Pressable){
                wBot.resetArm();
            }
            if(rb2 && rb2Pressable){
                wBot.extendOutPickUp(armPos);
            }


            telemetry.addLine("armMotorLeftPow: " + wBot.armMotorLeft.getPower());

            if(Math.abs(rx2) > 0) {
                wBot.correctorServo.setPosition(wBot.correctorServo.getPosition() + MonkeyMap.correctorServoSpeed * rx2);
            }
//            if(Math.abs(ry2) > 0){
//                wBot.rotatorServo.setPosition(wBot.rotatorServo.getPosition() + MonkeyMap.rotatorServoSpeed * ry2);
//            }
            if(rt2 > 0 && flipperPos2 - roomForErrorFlipper <= maxFlipperPos){
                wBot.setFlipperPos((int) (flipperPos2 - (MonkeyMap.flipperMotorSpeed * rt2)), MonkeyMap.flipperPower);
                if(retractSlidesOnFlip || retractSlidesForFlipping){
                    wBot.resetSlides();
                }
            }
            else if(lt2 > 0 && flipperPos2 + roomForErrorFlipper >= minFlipperPos){
                wBot.setFlipperPos((int) (flipperPos2 + (MonkeyMap.flipperMotorSpeed * lt2)), MonkeyMap.flipperPower);
                if(retractSlidesOnFlip || retractSlidesForFlipping){
                    wBot.resetSlides();
                }
            }

            if(flipperPos < minFlipperPos){
                wBot.setFlipperPos(minFlipperPos, MonkeyMap.flipperPower);
            }
            if(flipperPos > maxFlipperPos){
                wBot.setFlipperPos(maxFlipperPos, MonkeyMap.flipperPower);
            }


            if(dpu2 && dpu2Pressable){
                wBot.setCorrectorMid();
            }
            if(dpd2 && dpd2Pressable){
                driver.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
            }

            if(wBot.leftGrabberOpen && wBot.rightGrabberOpen){
                wBot.grabberIsOpen = true;
            }
            if(!wBot.leftGrabberOpen && !wBot.rightGrabberOpen){
                wBot.grabberIsOpen = false;
            }
            if(wBot.grabberIsOpen && wBot.leftGrabberOpen && wBot.rightGrabberOpen){
                gamepad2.setLedColor(rOpenGrabber, gOpenGrabber, bOpenGrabber, durationToKeepColor);
            }
            else {
                gamepad2.setLedColor(rCloseGrabber, gCloseGrabber, bCloseGrabber, durationToKeepColor);
            }

            if(dpr2 && dpr2Pressable){
                wBot.pumpFakeMacro(armPos);
            }

            a1Pressable = !a1;
            dpr1Pressable = !dpr1;
            dpu1Pressable = !dpu1;
            rb2Pressable = !rb2;
            lb2Pressable = !lb2;
            x2Pressable = !x2;
            b2Pressable = !b2;
            a2Pressable = !a2;
            y2Pressable = !y2;
            dpu2Pressable = !dpu2;
            dpd2Pressable = !dpd2;
            dpl2Pressable = !dpl2;
            dpr2Pressable = !dpr2;
            dpd1Pressable = !dpd1;

            driver.update();

            telemetry.addData("Heading", heading);
            telemetry.addData("grabberIsOpen ", wBot.grabberIsOpen);
            telemetry.addData("leftGrabberOpen ", wBot.leftGrabberOpen);
            telemetry.addData("rightGrabberOpen ", wBot.rightGrabberOpen);
//            telemetry.addData("Roll", angles.secondAngle);
//            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.addLine("slidesPos (left): " + wBot.armMotorLeft.getCurrentPosition());
            telemetry.addLine("Rotator Pos: " + wBot.rotatorServo.getPosition());
//            telemetry.addLine("Conveyor motor pow: " + wBot.conveyerMotor.getPower());
            telemetry.addLine("Flipper Motor Pos: " + flipperPos);
            telemetry.addLine("Flipper Motor Targ Pos: " + wBot.flipperMotor.getTargetPosition());
            telemetry.addLine("ly2 is: " + ly2);
            telemetry.addLine("rx2 is: " + rx2);
            telemetry.addLine("ry2 is: " + ry2);
            telemetry.addLine("Corrector Servo pos is " + wBot.correctorServo.getPosition());
            telemetry.addData("Pull Up Servo Pos ", wBot.pullUpServoLeft.getPosition());


            telemetry.update();
        }
    }

}

