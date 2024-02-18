package org.firstinspires.ftc.teamcode.CenterStageNEWBot.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Config
@TeleOp
public class ResetEncoders extends LinearOpMode {
    MonkeyMap wBot = new MonkeyMap(this);


    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.resetFlipperPos();
        wBot.resetSlidePoses();
        wBot.flipUp();
        wBot.resetSlides();

        waitForStart();

        while(opModeIsActive()){
            terminateOpModeNow();
        }
    }

}

