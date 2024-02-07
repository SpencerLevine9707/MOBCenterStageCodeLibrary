package org.firstinspires.ftc.teamcode.OLDSTUFF.conceptCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.JacksJunk;
import org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp
public class learningForNewCoders extends LinearOpMode {

    JacksJunk wBot = new JacksJunk(this);
    public static double secretDouble = 5.0;
    public static double driveSpeed = 0.5;
    OpenCvCamera webcam;


    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        wBot.init(true);
        MecanumDrive driveController = new MecanumDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("penis");
            telemetry.addLine("cock " + secretDouble);

            double stickLx = this.gamepad1.left_stick_x;
            double stickLy = this.gamepad1.left_stick_y;
            double stickRx = this.gamepad1.right_stick_x;

            boolean rb = this.gamepad1.right_bumper;
            boolean lb = this.gamepad1.left_bumper;

            if(rb){
                wBot.spencerLikesKids.setPosition(0);
            }

            if(lb){
                wBot.spencerLikesKids.setPosition(1);
            }

            driveController.moveInTeleop(stickLx, stickLy, stickRx, driveSpeed);

            telemetry.update();
        }
    }

}
