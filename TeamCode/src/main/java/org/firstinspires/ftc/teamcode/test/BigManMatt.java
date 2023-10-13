package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareMaps.JacksJunk;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp
@Disabled
public class BigManMatt extends LinearOpMode {

    JacksJunk wBot = new JacksJunk(this);
    public static double secretDouble = 5.0;
    public static double driveSpeed = 0.5;

    public static double servoPos1 = 0, servoPos2 = 1;
    OpenCvCamera webcam;


    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        wBot.init(true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        wBot.grabberServo1.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("penis");
            telemetry.addLine("cock " + secretDouble);



            boolean rb = this.gamepad1.right_bumper;
            boolean lb = this.gamepad1.left_bumper;

            if(rb){
                wBot.grabberServo1.setPosition(servoPos1);
                wBot.spencerLikesKids.setPosition(servoPos1);
            }

            if(lb){
                wBot.grabberServo1.setPosition(servoPos2);
                wBot.spencerLikesKids.setPosition(servoPos2);
            }

            telemetry.update();
        }
    }

}
