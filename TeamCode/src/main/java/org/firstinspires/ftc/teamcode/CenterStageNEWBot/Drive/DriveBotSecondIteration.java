package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;

@Config
@TeleOp
public class DriveBotSecondIteration extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);
    public ElapsedTime planeTime = new ElapsedTime();
    public static double timeForPlaneWait = 2;
    public static double divisorForSlidePowersUp = 1;
    public static double divisorForSlidePowersDown = 1;
    public static double maxFlipperPos = 0;
    public static double flipperPosFlushUp = MonkeyMap.flipperPosUp -0.02, flipperPosFlushDown = 0.2;
    public static double flipperPos30DegUp = 0.68, flipperPos30DegDown = 0.35;
    public static double holdPowerFlush = 0, holdPower30Deg = -0.1, holdPowerUp = -0.2;



    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);

        boolean a2Pressable = true;
        boolean b2Pressable = true;
        boolean x2Pressable = true;
        boolean y2Pressable = true;
        boolean dpu2Pressable = true;
        boolean dpl2Pressable = true;
        boolean dpr2Pressable = true;
        boolean lb2Pressable = true;
        boolean rb2Pressable = true;
//        boolean dpd2Pressable = true;

        planeTime.reset();
        wBot.setRotatorUp();
        wBot.setCorrectorMid();
        wBot.flipUp();
        wBot.openGrabber();

        waitForStart();

        while(opModeIsActive()){
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;
            double flipperPos = wBot.flipperServoLeft.getPosition();

            telemetry.addLine("lx1: " + lx1 + " ly1: " + ly1 + " rx1 " + rx1);

            double powerFromTriggers = Math.abs(lx1) + Math.abs(ly1) + Math.abs(rx1);

//            drive.moveInTeleop(-lx1, -ly1, rx1, powerFromTriggers);
            drive.moveInTeleop(lx1, ly1, rx1, powerFromTriggers);

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
            boolean lsb2 = gamepad2.left_stick_button;
            boolean rsb2 = gamepad2.right_stick_button;


            if(a2 && a2Pressable){
                wBot.toggleGrabber();
            }
            if(x2 && x2Pressable){
                wBot.toggleLeftGrabber();
            }
            if(b2 && b2Pressable){
                wBot.toggleRightGrabber();
            }

            x2Pressable = !x2;
            b2Pressable = !b2;

            telemetry.addLine("Grabber Servo Pos (left) " + wBot.grabberServoLeft.getPosition() + " Grabber Servo Pos (Right) " + wBot.grabberServoRight.getPosition());

            a2Pressable = !a2;

            if(y2 && y2Pressable){
                wBot.toggleFlipper();
            }
            telemetry.addLine("y2 is " + y2);

            y2Pressable = !y2;

            if(lsb2 && rsb2){
                wBot.loadPixels();
                planeTime.reset();
            }
            else if(planeTime.seconds() > timeForPlaneWait){
                wBot.stopLoadingPixels();
            }

            if(ly2 < 0){
                wBot.setSlidePowers(ly2/divisorForSlidePowersUp);
            }
            else if(ly2 > 0){
                wBot.setSlidePowers(ly2/divisorForSlidePowersDown);
            }
            else{
                wBot.setSlidePowers(MonkeyMap.holdPowerForSlides);
            }
            telemetry.addLine("armMotorLeftPow: " + wBot.armMotorLeft.getPower() + " armMotorRightPow: " + wBot.armMotorRight.getPower());

            if(Math.abs(rx2) > 0) {
                wBot.correctorServo.setPosition(wBot.correctorServo.getPosition() + MonkeyMap.correctorServoSpeed * rx2);
            }
            if(Math.abs(ry2) > 0){
                wBot.rotatorServo.setPosition(wBot.rotatorServo.getPosition() + MonkeyMap.rotatorServoSpeed * ry2);
            }
            if(rt2 > 0){
                wBot.setFlipperPos(flipperPos + MonkeyMap.flipperServoSpeed * rt2);
            }
            else if(lt2 > 0 && flipperPos > maxFlipperPos){
                wBot.setFlipperPos(flipperPos - MonkeyMap.flipperServoSpeed * lt2);
            }

            if(rb2 && rb2Pressable){
                wBot.rotatorPickUpAndPlace();
            }
            if(lb2 && lb2Pressable){
                wBot.setRotatorFlush();
            }
            if(flipperPos > flipperPosFlushUp || flipperPos < flipperPosFlushDown){
                MonkeyMap.holdPowerForSlides = holdPowerFlush;
            }
            else if(flipperPos > flipperPos30DegUp || flipperPos < flipperPos30DegDown){
                MonkeyMap.holdPowerForSlides = holdPower30Deg;
            }
            else{
                MonkeyMap.holdPowerForSlides = holdPowerUp;
            }

            rb2Pressable = !rb2;
            lb2Pressable = !lb2;

            telemetry.addLine("Rotator Pos: " + wBot.rotatorServo.getPosition());
            telemetry.addLine("Conveyor motor pow: " + wBot.conveyerMotor.getPower());
            telemetry.addLine("Flipper Servo Left Pos: " + flipperPos + " Flipper Servo Right Pos: " + wBot.flipperServoRight.getPosition());
            telemetry.addLine("ly2 is: " + ly2);
            telemetry.addLine("rx2 is: " + rx2);
            telemetry.addLine("ry2 is: " + ry2);
            telemetry.addLine("Corrector Servo pos is " + wBot.correctorServo.getPosition());


            telemetry.update();
        }
    }
}
