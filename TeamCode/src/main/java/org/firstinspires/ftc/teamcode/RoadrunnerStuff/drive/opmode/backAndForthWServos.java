package org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

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
//@Config
@Autonomous(group = "drive")
@Disabled
public class backAndForthWServos extends LinearOpMode {

    public static double DISTANCE = 50;
    public Servo grabberServo1;
    public Servo grabberServo2;
    public Servo rotatorServo;
    public Servo armServo1;
    public Servo armServo2;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        rotatorServo = hardwareMap.get(Servo.class, "rotatorServo");
        grabberServo1 = hardwareMap.get(Servo.class, "grabberServo1");
        grabberServo2 = hardwareMap.get(Servo.class, "grabberServo2");

        grabberServo1.setDirection(Servo.Direction.REVERSE);

        armServo1.setDirection(Servo.Direction.REVERSE);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .addDisplacementMarker(()->{
                    armServo1.setPosition(0.5);
                    armServo2.setPosition(0.5);
                })
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .addDisplacementMarker(()->{
                    armServo1.setPosition(0.79);
                    armServo2.setPosition(0.79);
                })
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
    }
}