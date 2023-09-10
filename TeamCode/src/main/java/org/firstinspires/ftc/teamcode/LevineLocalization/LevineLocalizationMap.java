package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LevineLocalizationMap {
    public static double trackWidth = 10.1;
    public static double wheelDiameter = 1.37794;
//    public static double centerWheelOffset = -6.69;
    public static double centerWheelOffset = -6.3;
    public static double ticksPerRev = 8192;
    public static double ticksToInches = wheelDiameter * Math.PI / ticksPerRev;

    public static double xMultiplier = 1;
    public static double yMultiplier = 1;
    public static double maxVelocity = 40;
    public static double maxAcceleration = 30;

    public static double maxAngularVelocity = Math.toRadians(100);

    public static double maxAngularAcceleration = Math.toRadians(100);

    public static double poseError = 0.75;
    public static double angError = Math.toRadians(1);

    public static double poseErrorInBetween = poseError*10;
    public static double angErrorInBetween = angError*20;
    public static double timeout = -1;

    public static double followRadius = 10;

    public static double followRadiusAdditive = 5;

    public static double poseFollowCoef = 1;

    public static double angFollowerCoef = 1;

    public static double speedMultiplier = 1;

    private LinearOpMode myOpMode;

    public MecanumDrive m_robotDrive;
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public OdoWheelSetUp odos;
    public MotorEx leftEncoder, rightEncoder, centerEncoder;

    public TrajectoryConfig constraints = new TrajectoryConfig(maxVelocity, maxAcceleration);

    public VoltageSensor batteryVoltageSensor;

    public LevineLocalizationMap(LinearOpMode opmode){
        myOpMode = opmode;
    }

    public void init() {
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEncoder = new MotorEx(myOpMode.hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(myOpMode.hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(myOpMode.hardwareMap, "backLeft");

        batteryVoltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();

        odos = new OdoWheelSetUp(leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(), centerEncoder.getCurrentPosition(), myOpMode);
    }
    public void init(Pose2d startingPose) {
        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, ("frontLeft")); //port 3
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, ("frontRight")); //port 2
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, ("backLeft")); //port 1
        backRight = myOpMode.hardwareMap.get(DcMotor.class, ("backRight"));  //port 0

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEncoder = new MotorEx(myOpMode.hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(myOpMode.hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(myOpMode.hardwareMap, "backLeft");

        batteryVoltageSensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        Telemetry telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();

        odos = new OdoWheelSetUp(leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(), centerEncoder.getCurrentPosition(), startingPose, myOpMode);
    }
    public void updateTicksAuto(){
        odos.updateTicksManual(leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition(), centerEncoder.getCurrentPosition());
    }
    public void updatePose(){
        odos.updateCurrPose();
    }
    public void setMotorPowers(Pose2d currPose, Pose2d targetPose){
        // Get Powers
        WheelPowers powers = odos.getPowers(currPose, targetPose);

        // Set powers
        frontLeft.setPower(powers.flp);
        frontRight.setPower(powers.frp);
        backLeft.setPower(powers.blp);
        backRight.setPower(powers.brp);
    }
    public void setMotorPowers(Pose2d currPose, Pose2d targetPose, double speedMultiplier){
        // Get Powers
        WheelPowers powers = odos.getPowers(currPose, targetPose, speedMultiplier);

        // Set powers
        frontLeft.setPower(powers.flp);
        frontRight.setPower(powers.frp);
        backLeft.setPower(powers.blp);
        backRight.setPower(powers.brp);
    }
    public void stopMotors(){
        // Set powers
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
