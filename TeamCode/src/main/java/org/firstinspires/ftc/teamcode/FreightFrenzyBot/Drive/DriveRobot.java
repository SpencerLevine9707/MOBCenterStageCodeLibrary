package org.firstinspires.ftc.teamcode.FreightFrenzyBot.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.AntiDriftDrive;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.MonkeyOperationsMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.MathsAndStuff;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

@Config
@TeleOp
public class DriveRobot extends LinearOpMode {
    MonkeyOperationsMap wBot = new MonkeyOperationsMap(this);
    SampleMecanumDrive driver;
    AntiDriftDrive drive;
    public static double maxSpeed = 1;
    public static double turnSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        driver = new SampleMecanumDrive(this.hardwareMap);
        drive = new AntiDriftDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


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
        boolean y1Pressable = true;
        boolean lb1Pressable = true;
        boolean rb1Pressable = true;
        boolean dpr1Pressable = true;
        boolean dpu1Pressable = true;
        boolean dpd1Pressable = true;
        boolean isReserveDrive = false;
        boolean armCameDown = true;
        boolean autoCorrectorAdjust = false;

        wBot.openGrabber();
        wBot.flipDown();
        driver.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        waitForStart();

        while(opModeIsActive()){
            double heading = driver.getPoseEstimate().getHeading();


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

            double speedMultiplier = maxSpeed*(lx1+ly1);

            if(speedMultiplier == 0){
                speedMultiplier = turnSpeed;
            }




            drive.moveInTeleop(lx1, ly1, rx1, speedMultiplier, heading);

            telemetry.addLine("lx1: " + lx1 + " ly1: " + ly1 + " rx1 " + rx1);

            double powerFromTriggers = Math.abs(lx1) + Math.abs(ly1) + Math.abs(rx1);

            if(a1 && a1Pressable){
                wBot.toggleGrabber();
            }
            if(y1 && y1Pressable){
                wBot.flipShared();
            }
            if(lb1 && lb1Pressable){
                wBot.flipDown();
            }
            if(dpd1 && dpd1Pressable){
                wBot.flipFirstLayer();
            }
            if(dpr1 && dpr1Pressable){
                wBot.flipSecondLayer();
            }
            if(dpu1 && dpu1Pressable){
                wBot.flipThirdLayer();
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

            a1Pressable = !a1;
            y1Pressable = !y1;
            lb1Pressable = !lb1;
            dpr1Pressable = !dpr1;
            dpu1Pressable = !dpu1;
            rb1Pressable = !rb1;
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

            double ADPower = (-ly1 + lx1);

            double BCPower = (-ly1 - lx1);

            double totAngDist = drive.targetHeading - MathsAndStuff.AngleWrap(heading);

            double turnPower = MathsAndStuff.AngleWrap(totAngDist);



            double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
            turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

            if (Math.abs(turningScale) < 1.0){
                turningScale = 1.0;
            }


            double fl = (ADPower - turnPower) / turningScale;
            double fr = (BCPower + turnPower) / turningScale;
            double bl = (BCPower - turnPower) / turningScale;
            double br = (ADPower + turnPower) / turningScale;

            telemetry.addData("TargetHeading ", drive.targetHeading);
            telemetry.addData("TurningScale ", turningScale);
            telemetry.addData("totAngDist ", totAngDist);
            telemetry.addData("ADPow ", ADPower);
            telemetry.addData("BCPow ", BCPower);
            telemetry.addData("fl ", fl);
            telemetry.addData("fr", fr);
            telemetry.addData("bl ", bl);
            telemetry.addData("br ", br);
            telemetry.addData("Heading", heading);

            telemetry.update();
        }
    }

}

