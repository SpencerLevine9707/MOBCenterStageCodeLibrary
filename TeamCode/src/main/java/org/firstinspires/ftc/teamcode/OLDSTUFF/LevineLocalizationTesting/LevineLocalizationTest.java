package org.firstinspires.ftc.teamcode.OLDSTUFF.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.LevineLocalizationMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.MathsAndStuff;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;


@Config
@Autonomous(group = "Levine Local")
public class LevineLocalizationTest extends LinearOpMode {
    LevineLocalizationMap wMap = new LevineLocalizationMap(this);
    public static double xPosTargetPos = 20;
    public static double yPosTargetPos = 0;
    public static double headingTargetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d firstPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d targetPose = new Pose2d(xPosTargetPos, yPosTargetPos, Math.toRadians(headingTargetPos));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wMap.init(firstPose);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        boolean xDone = false;
        boolean yDone = false;

        waitForStart();

        while (opModeIsActive()) {

//            wMap.updateTicksAuto();
//            wMap.updatePose();

            drive.update();

            Pose2d currPose = drive.getPoseEstimate();

            double xDist = targetPose.getX() - currPose.getX();
            double yDist = targetPose.getY() - currPose.getY();
            double angDist = targetPose.getHeading() - currPose.getHeading();

            double distToTarget = Math.hypot(xDist, yDist);
            double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.startingPose.getHeading());

            //Error X
            double relDistX = Math.cos(theta) * distToTarget;
            double relErrorX = (Math.cos(theta) * LevineLocalizationMap.poseError);

            double relDistY = Math.sin(theta) * distToTarget;
            double relErrorY = (Math.sin(theta) * LevineLocalizationMap.poseError);
            if (Math.abs(relDistX) < Math.abs(relErrorX)) {
                System.out.println("X is done");
                xDone = true;
            }
            if (Math.abs(relDistY) < Math.abs(relErrorY)) {
                System.out.println("Y is done");
                yDone = true;
            }
            if(!(xDone && yDone)){
                wMap.setMotorPowers(currPose, targetPose);
            }
            else{
                requestOpModeStop();
            }


//            telemetry.addLine("CurrPose: " + currPose);
//            telemetry.addLine("CurrLeftE: " + wMap.leftEncoder.getCurrentPosition());
//            telemetry.addLine("CurrRightE: " + wMap.rightEncoder.getCurrentPosition());
//            telemetry.addLine("CurrCenterE: " + wMap.centerEncoder.getCurrentPosition());
//            telemetry.addLine("Ticks between: " + wMap.odos.getTicksBetween(wMap.odos.currPose, targetPose));
            telemetry.addLine("Motor powers: " + wMap.getPowers(currPose, targetPose));
            telemetry.addLine("Error X: " + relErrorX + " Error Y: " + relErrorY);
            telemetry.addLine("X done? " + xDone + " Y done? " + yDone);
            telemetry.addLine("relDistX " + relDistX + " relDistY " + relDistY);
            telemetry.update();
        }
    }
}