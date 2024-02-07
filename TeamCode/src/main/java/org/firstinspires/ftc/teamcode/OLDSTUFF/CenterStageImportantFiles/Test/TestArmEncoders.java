package org.firstinspires.ftc.teamcode.OLDSTUFF.CenterStageImportantFiles.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OLDSTUFF.CenterStageImportantFiles.HardwareMaps.MonkeyMapOLDBOT;

@Config
@TeleOp
public class TestArmEncoders extends LinearOpMode {
    MonkeyMapOLDBOT wBot = new MonkeyMapOLDBOT(this);
    public ElapsedTime planeTime = new ElapsedTime();

    public static int testSlidePos = -100;
    public static double testSlidePow = -0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        wBot.armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wBot.armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean dpu2Pressable = true;

        waitForStart();

        while(opModeIsActive()){
            boolean dpu2 = gamepad2.dpad_down;
            if(dpu2 && dpu2Pressable){
                wBot.encodedSlipperySlides(testSlidePos, testSlidePow);
            }

            dpu2Pressable = !dpu2;
            telemetry.addData("leftArmPos: ", wBot.armMotorLeft.getCurrentPosition());
            telemetry.addData("rightArmPos: ", wBot.armMotorRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
