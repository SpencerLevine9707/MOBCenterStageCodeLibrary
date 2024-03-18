package org.firstinspires.ftc.teamcode.FreightFrenzyBot.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.AntiDriftDrive;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.MonkeyOperationsMap;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

@Config
@Autonomous
public class HuskyLensDriveInLine extends LinearOpMode {
    MonkeyOperationsMap wBot = new MonkeyOperationsMap(this);
    SampleMecanumDrive driver;
    AntiDriftDrive drive;
    public static double maxSpeed = 0.1;
    public static double angleToAdd = 0;
    public static double defaultAngle = 90;
    public static double angleToAddHeading = 90;
    public static double noMoveTheshold = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        wBot.init();
        wBot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.LINE_TRACKING);
        wBot.flipDown();
        wBot.openGrabber();
        double theta = 0;
        double speedToMoveAt = maxSpeed;
        drive = new AntiDriftDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);
        waitForStart();

        while(opModeIsActive()){
            HuskyLens.Arrow[] arrow = wBot.huskyLens.arrows();

            for (HuskyLens.Arrow arrows : arrow) {
                telemetry.addData("Arrow", arrows.toString());
            }
            telemetry.addData("ArrowLen", arrow.length);

            for (HuskyLens.Algorithm c : HuskyLens.Algorithm.values()){
               telemetry.addData("NotSure", c);
            }

            int xDist = 0;
            int yDist = 0;

            if(arrow.length>0){
                xDist = arrow[0].x_target - arrow[0].x_origin;
                yDist = arrow[0].y_target - arrow[0].y_origin;
                theta = Math.atan2(yDist, xDist) + Math.toRadians(angleToAdd);
            }
            else{
                theta = Math.toRadians(defaultAngle);
            }
            double thetaToMoveTowards = theta - Math.toRadians(angleToAddHeading);
            if(Math.abs(thetaToMoveTowards) > Math.toRadians(noMoveTheshold)){
                speedToMoveAt = 0;
            }
            else{
                speedToMoveAt = maxSpeed;
            }



            telemetry.addData("xDist", xDist);
            telemetry.addData("yDist", yDist);

            telemetry.addData("theta", theta);
            telemetry.addData("thetaDeg", Math.toDegrees(theta));

        drive.moveTowardsAngleThetaAndChangeAngle(theta, thetaToMoveTowards, speedToMoveAt);
            telemetry.update();
        }

    }
}
