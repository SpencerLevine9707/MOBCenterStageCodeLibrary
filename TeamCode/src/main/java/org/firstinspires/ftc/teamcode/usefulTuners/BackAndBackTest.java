package org.firstinspires.ftc.teamcode.usefulTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.util.Encoder;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class BackAndBackTest extends LinearOpMode {

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public static double DISTANCE = 50;
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void runOpMode() throws InterruptedException {

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontLeft"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft"));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPos = new Pose2d();
        Pose2d goingPos = new Pose2d(-DISTANCE, startingPos.getY(), startingPos.getHeading());
        Pose2d goingPosAfterTurn = new Pose2d(-DISTANCE, startingPos.getY(), startingPos.getHeading() + Math.toRadians(180));
        Pose2d endingPos = new Pose2d(startingPos.getX(), startingPos.getY(), startingPos.getHeading() + Math.toRadians(180));

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(goingPos)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(goingPosAfterTurn)
                .lineToLinearHeading(endingPos)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            drive.followTrajectory(trajectoryForward);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Left Encoder ", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder ", rightEncoder.getCurrentPosition());
            telemetry.addData("Front Encoder ", frontEncoder.getCurrentPosition());
            telemetry.update();

            drive.turn(Math.toRadians(180));

            poseEstimate = drive.getPoseEstimate();
            drive.followTrajectory(trajectoryBackward);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Left Encoder ", leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder ", rightEncoder.getCurrentPosition());
            telemetry.addData("Front Encoder ", frontEncoder.getCurrentPosition());
            telemetry.update();

            drive.turn(Math.toRadians(180));

        }

    }
}