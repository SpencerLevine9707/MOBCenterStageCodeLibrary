package org.firstinspires.ftc.teamcode.hardwareMaps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadrunnerStuff.util.Encoder;

//@Config
public class PurePursuitMap {
    public static double trackWidth = 10.1;
    public static double wheelDiameter = 1.37794;
    public static double ticksToInches;
    public static double centerWheelOffset = -6.69;
    public static double ticksPerRev = 8192;

    public static double movementSpeed = 0.8;

    public static double maxVelocity = 1.5;
    public static double maxAcceleration = 1.5;

    public static double turnSpeed = 0.8;

    public static double posError = 0.003175;
    public static double angError = Math.toRadians(1);
    public static double timeout = -1;
    public static double followRadius = 0.03;


    private CommandOpMode myOpMode;

    public HolonomicOdometry m_robotOdometry;
    public OdometrySubsystem m_odometry;
//    public PurePursuitCommand ppCommand;
    public MecanumDrive m_robotDrive;
    public Motor frontLeft, frontRight, backLeft, backRight;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    public TrajectoryConfig constraints = new TrajectoryConfig(maxVelocity, maxAcceleration);

    public PurePursuitMap(CommandOpMode opmode){
        myOpMode = opmode;
    }

    public void init() {
        frontLeft = new Motor(myOpMode.hardwareMap, "frontLeft");
        frontRight = new Motor(myOpMode.hardwareMap, "frontRight");
        backLeft = new Motor(myOpMode.hardwareMap, "backLeft");
        backRight = new Motor(myOpMode.hardwareMap, "backRight");

        // create our drive object
        m_robotDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        leftEncoder = new MotorEx(myOpMode.hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(myOpMode.hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(myOpMode.hardwareMap, "backLeft");

        ticksToInches = wheelDiameter * Math.PI / ticksPerRev;

        m_robotOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * ticksToInches,
                () -> rightEncoder.getCurrentPosition() * ticksToInches,
                () -> centerEncoder.getCurrentPosition() * ticksToInches,
                trackWidth, centerWheelOffset
        );

        m_odometry = new OdometrySubsystem(m_robotOdometry);

    }
    public static Rotation2d angle(double angInDeg){
        return new Rotation2d(Math.toRadians(angInDeg));
    }
}
