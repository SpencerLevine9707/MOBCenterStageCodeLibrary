package org.firstinspires.ftc.teamcode.OLDSTUFF.CenterStageImportantFiles.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OLDSTUFF.CenterStageImportantFiles.HardwareMaps.MonkeyMapOLDBOT;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.MecanumDrive;

@Config
@TeleOp
@Disabled
public class DriveBotFirstIteration extends LinearOpMode {
    MonkeyMapOLDBOT wBot = new MonkeyMapOLDBOT(this);
    public ElapsedTime planeTime = new ElapsedTime();
    public static double timeForPlaneWait = 0.5;
    public static double divisorForSlidePowersUp = 1;
    public static double divisorForSlidePowersDown = 2;



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
//        boolean lb2Pressable = true;
//        boolean rb2Pressable = true;
//        boolean dpd2Pressable = true;

        planeTime.reset();
        wBot.toggleRotator();
        wBot.toggleFlipper();
        wBot.openGrabber();
        wBot.resetKnocker();

        waitForStart();

        while(opModeIsActive()){
            double lx1 = gamepad1.left_stick_x;
            double ly1 = gamepad1.left_stick_y;
            double rx1 = gamepad1.right_stick_x;

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
            boolean dpu2 = gamepad2.dpad_up;
            boolean dpd2 = gamepad2.dpad_down;
            boolean dpl2 = gamepad2.dpad_left;
            boolean dpr2 = gamepad2.dpad_right;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;

            if(a2 && a2Pressable){
                wBot.toggleGrabber();
            }
            telemetry.addLine("Grabber Servo Pos " + wBot.grabberServo.getPosition() + "\n is open? " + wBot.grabberIsOpen);

            a2Pressable = !a2;

            if(b2 && b2Pressable){
                wBot.toggleConveyer();
            }
            else if(x2){
                wBot.conveyerMotor.setPower(MonkeyMapOLDBOT.unloadPower);
                wBot.conveyerOn = false;
            }
            else if(!wBot.conveyerOn){
                wBot.conveyerMotor.setPower(MonkeyMapOLDBOT.stopLoadPower);
            }

            x2Pressable = !x2;

            b2Pressable = !b2;

            if(y2 && y2Pressable){
                wBot.toggleFlipper();
            }
            telemetry.addLine("y2 is " + y2);

            y2Pressable = !y2;

            if(lb2 && rb2 && dpd2 && planeTime.seconds() > timeForPlaneWait){
                wBot.toggleAirplane();
                planeTime.reset();
            }

            if(ly2 < 0){
                wBot.setSlidePowers(ly2/divisorForSlidePowersUp);
            }
            else if(ly2 > 0){
                wBot.setSlidePowers(ly2/divisorForSlidePowersDown);
            }
            else{
                wBot.setSlidePowers(MonkeyMapOLDBOT.holdPowerForSlides);
            }
            telemetry.addLine("ry2 is " + ry2);
            telemetry.addLine("armMotorLeftPow: " + wBot.armMotorLeft.getPower() + " armMotorRightPow: " + wBot.armMotorRight.getPower());

            if(dpu2 && dpu2Pressable){
                wBot.toggleRotator();
            }

            dpu2Pressable = !dpu2;

            if(dpl2 && dpl2Pressable){
                wBot.toggleKnocker();
            }
            dpl2Pressable = !dpl2;

            telemetry.addLine("Rotator Pos: " + wBot.rotatorServo.getPosition());
            telemetry.addLine("Conveyor motor pow: " + wBot.conveyerMotor.getPower());
            telemetry.addLine("Flipper Servo Left Pos: " + wBot.flipperServoLeft.getPosition() + " Flipper Servo Right Pos: " + wBot.flipperServoRight.getPosition());

//            wBot.intakeNoodleServo.setPosition(ly2);

            telemetry.addLine("ly2 is: " + ly2);

            telemetry.update();
        }
    }
}
