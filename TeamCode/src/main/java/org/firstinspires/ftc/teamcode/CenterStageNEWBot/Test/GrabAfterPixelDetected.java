package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;

@Config
@TeleOp
public class GrabAfterPixelDetected extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    public ElapsedTime planeTime = new ElapsedTime();
    public ElapsedTime timeForClose = new ElapsedTime();
    public static double timeForPlaneWait = 0.75;
    public static double maxFlipperPos = 0;
    public static double lowestFlipperPos = 1;
    public static double slideSpeed = -150;
    public static int maxArmPos = 0, minArmPos = -314;
    public static boolean retractSlidesOnFlip = true;
    public static boolean slideResetOnInit = false;
    public static double slowSpeedDrive = 0.3;
    public static double maxRotatorPosUp = 0.644, maxRotatorPosDown = 0.43;
    public static double lowestFlipperPosDown = 0.765, highestFlipperPos = 0.02;
    public static double divisorForSpinPower = 1.5;
    public static double pullUpPowMotor = 1;
    public ColorRangeSensor color;
    public static double distForPickUp = 0.35;
    public static double timeForCloseWait = 0.75;



    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        if(slideResetOnInit){
            wBot.resetSlidePoses();
        }

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
        boolean isReserveDrive = false;

        double powerForMotors = 0;

        planeTime.reset();
        wBot.setRotatorUp();
        wBot.setCorrectorMid();
        wBot.flipUp();
        wBot.closeGrabber();
        wBot.encodedSlipperySlides(MonkeyMap.resetSlidesPos, MonkeyMap.slidePowerEncoder);
        wBot.loadPlane();
        double rotatorRange = maxRotatorPosUp - maxRotatorPosDown;

        color = hardwareMap.get(ColorRangeSensor.class, "colorSense");
        timeForClose.reset();

        waitForStart();

        while(opModeIsActive()){
//            Gamepad 1 controls
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;
            boolean y1 = gamepad1.y;
            double flipperPos = wBot.flipperServoLeft.getPosition();
            boolean lsb1 = gamepad1.left_stick_button;
            boolean rsb1 = gamepad1.right_stick_button;
            boolean rb1 = gamepad1.right_bumper, lb1 = gamepad1.left_bumper;
            double rt1 = gamepad1.right_trigger, lt1 = gamepad1.left_trigger;
            boolean a1 = gamepad1.a;

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
                drive.moveInTeleop(lx1, ly1, rx1, powerForMotors);
            }
            if(y1 && planeTime.seconds() > timeForPlaneWait){
                wBot.toggleAirplane();
                planeTime.reset();
            }


            if(lsb1 && rsb1 && planeTime.seconds() > timeForPlaneWait){
                wBot.shootPlane();
                planeTime.reset();
            }

            if(rt1 > 0){
                wBot.pullUpMotorRight.setPower(rt1);
                wBot.pullUpMotorLeft.setPower(-rt1);
            }
            else if(lt1 > 0){
                wBot.pullUpMotorRight.setPower(-lt1);
                wBot.pullUpMotorLeft.setPower(lt1);
            }
            else{
                wBot.pullUpMotorRight.setPower(0);
                wBot.pullUpMotorLeft.setPower(0);
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
                timeForClose.reset();
            }
            if(x2 && x2Pressable){
                wBot.toggleLeftGrabber();
            }
            if(b2 && b2Pressable){
                wBot.toggleRightGrabber();
            }
            if(color.getDistance(DistanceUnit.INCH) < distForPickUp && timeForClose.seconds() > timeForCloseWait){
                wBot.closeGrabber();
            }

            telemetry.addLine("Grabber Servo Pos (left) " + wBot.grabberServoLeft.getPosition() + " Grabber Servo Pos (Right) " + wBot.grabberServoRight.getPosition());

            if(y2 && y2Pressable){
                wBot.toggleFlipper();
                if(retractSlidesOnFlip){
                    wBot.resetSlides();
                }
            }
            telemetry.addLine("y2 is " + y2);

//            if(ly2 < 0){
//                wBot.setSlidePowers(-ly2/divisorForSlidePowersUp);
//            }
//            else if(ly2 > 0){
//                wBot.setSlidePowers(-ly2/divisorForSlidePowersDown);
//            }
//            else{
//                wBot.setSlidePowers(MonkeyMap.holdPowerForSlides);
//            }
            if(Math.abs(ly2) > 0 && wBot.armMotorLeft.getTargetPosition() <= maxArmPos && wBot.armMotorLeft.getTargetPosition() >= minArmPos) {
                wBot.encodedSlipperySlides((int) (wBot.armMotorLeft.getCurrentPosition() - (slideSpeed * ly2)), MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() >= maxArmPos){
                wBot.encodedSlipperySlides(maxArmPos, MonkeyMap.slidePowerEncoder);
            }
            if(wBot.armMotorLeft.getTargetPosition() <= minArmPos){
                wBot.encodedSlipperySlides(minArmPos, MonkeyMap.slidePowerEncoder);
            }


            telemetry.addLine("armMotorLeftPow: " + wBot.armMotorLeft.getPower() + " armMotorRightPow: " + wBot.armMotorRight.getPower());

            if(Math.abs(rx2) > 0) {
                wBot.correctorServo.setPosition(wBot.correctorServo.getPosition() + MonkeyMap.correctorServoSpeed * rx2);
            }
//            if(Math.abs(ry2) > 0){
//                wBot.rotatorServo.setPosition(wBot.rotatorServo.getPosition() + MonkeyMap.rotatorServoSpeed * ry2);
//            }
            if(rt2 > 0 && flipperPos < lowestFlipperPos){
                wBot.setFlipperPos(flipperPos + MonkeyMap.flipperServoSpeed * rt2);
                if(retractSlidesOnFlip){
                    wBot.resetSlides();
                }
            }
            else if(lt2 > 0 && flipperPos > maxFlipperPos){
                wBot.setFlipperPos(flipperPos - MonkeyMap.flipperServoSpeed * lt2);
                if(retractSlidesOnFlip){
                    wBot.resetSlides();
                }
            }


//            if(rb2 && rb2Pressable){
//                wBot.rotatorPickUpAndPlace();
//            }
//            if(lb2 && lb2Pressable){
//                wBot.setRotatorFlush();
//            }
            if(wBot.flipperServoRight.getPosition() > lowestFlipperPosDown){
                wBot.setRotatorFlush();
            }
            else{
                double flipperScalar = Math.abs((wBot.flipperServoRight.getPosition()/(lowestFlipperPosDown-highestFlipperPos)) - 1);
                double newRotatorPos = maxRotatorPosDown + (flipperScalar*rotatorRange);
                wBot.rotatorServo.setPosition(newRotatorPos);
            }
            //0.218 - 0.783


            if(dpu2 && dpu2Pressable){
                wBot.setCorrectorMid();
            }

//            if(dpd2 && dpd2Pressable){
//                wBot.toggleAirplane();
//            }

//            if(flipperPos > wBot.flipperPosUp){
//                MonkeyMap.holdPowerForSlides = holdPowerDown;
//            }
//            else if(flipperPos > flipperPosFlushUp || flipperPos < flipperPosFlushDown){
//                MonkeyMap.holdPowerForSlides = holdPowerFlush;
//            }
//            else if(flipperPos > flipperPos30DegUp || flipperPos < flipperPos30DegDown){
//                MonkeyMap.holdPowerForSlides = holdPower30Deg;
//            }
//            else{
//                MonkeyMap.holdPowerForSlides = holdPowerUp;
//            }
            if(wBot.leftGrabberOpen && wBot.rightGrabberOpen){
                wBot.grabberIsOpen = true;
            }
            if(!wBot.leftGrabberOpen && !wBot.rightGrabberOpen){
                wBot.grabberIsOpen = false;
            }

            a1Pressable = !a1;
            rb2Pressable = !rb2;
            lb2Pressable = !lb2;
            x2Pressable = !x2;
            b2Pressable = !b2;
            a2Pressable = !a2;
            y2Pressable = !y2;
            dpu2Pressable = !dpu2;
            dpd2Pressable = !dpd2;

            telemetry.addData("DistanceDetected", color.getDistance(DistanceUnit.INCH));

//            telemetry.addLine("slidesPox: " + wBot.armMotorRight.getCurrentPosition());
//            telemetry.addLine("slidesPos (left): " + wBot.armMotorLeft.getCurrentPosition());
//            telemetry.addLine("slidesPos target: " + wBot.armMotorRight.getTargetPosition());
//            telemetry.addLine("Rotator Pos: " + wBot.rotatorServo.getPosition());
////            telemetry.addLine("Conveyor motor pow: " + wBot.conveyerMotor.getPower());
//            telemetry.addLine("Flipper Servo Left Pos: " + flipperPos + " Flipper Servo Right Pos: " + wBot.flipperServoRight.getPosition());
//            telemetry.addLine("ly2 is: " + ly2);
//            telemetry.addLine("rx2 is: " + rx2);
//            telemetry.addLine("ry2 is: " + ry2);
//            telemetry.addLine("Corrector Servo pos is " + wBot.correctorServo.getPosition());


            telemetry.update();
        }
    }
}
