package org.firstinspires.ftc.teamcode.FreightFrenzyBot.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMUNew;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
import java.util.function.Function;

@Config
@TeleOp(group = "Center Stage")
public class IMUAndHuskyTest extends LinearOpMode {

//    ModernRoboticsI2cGyro IMU;
    BNO055IMU IMU2;
    ElapsedTime integrationTime = new ElapsedTime();
    public static int steps = 10000;
    public static double accelMessingUpVal = 0;
//    BNO055IMUNew IMU3;
//    BHI260IMU IMU4;

    public static int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() throws InterruptedException {

//         IMU = hardwareMap.get(ModernRoboticsI2cGyro.class, "IMU");
         IMU2 = hardwareMap.get(BNO055IMU.class, "IMU2");
//         IMU3 = hardwareMap.get(BNO055IMUNew.class, "IMU3");
//         IMU4 = hardwareMap.get(BHI260IMU.class, "IMU4");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();

        boolean a1Pressable = true;
        boolean b1Pressable = true;

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        IMU2.initialize(parameters2);

        integrationTime.reset();


        double currTime = 0;
        Pose2d currVel = new Pose2d();
        Pose2d prevVel = new Pose2d();
        Pose2d currPose = new Pose2d();
        Pose2d prevPose = new Pose2d();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        integrationTime.reset();




        while(opModeIsActive()){
            currTime = integrationTime.seconds();
            prevVel = currVel;
            prevPose = currPose;
            if (!rateLimit.hasExpired()) {
                continue;
            }

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }

//            IMU.initialize();
//            double x = IMU.rawX();
//            double y = IMU.rawY();
//            double z = IMU.rawZ();
//            double heading = IMU.getHeading();
//            Orientation angularAxis = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//            String status = IMU.status();


//
            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            if(a1 && a1Pressable){
                IMU2.startAccelerationIntegration(new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 1), new Velocity(DistanceUnit.INCH, 0.0, 0.0, 0.0, 1), 50);
            }
            else if(b1 && b1Pressable){
                IMU2.stopAccelerationIntegration();
            }
            a1Pressable = !a1;
            b1Pressable = !b1;


            Position position2 = IMU2.getPosition();
            Velocity velocity2 = IMU2.getVelocity();
            Acceleration acceleration2 = IMU2.getAcceleration();
            Orientation angle2 = IMU2.getAngularOrientation();
            BNO055IMU.CalibrationStatus IMUCal = IMU2.getCalibrationStatus();
            Acceleration gravity = IMU2.getGravity();


            Acceleration linearAcceleration = IMU2.getLinearAcceleration().toUnit(DistanceUnit.INCH);
            double xAccel = linearAcceleration.xAccel;
            double yAccel = linearAcceleration.yAccel;
            if(Math.abs(linearAcceleration.xAccel) < accelMessingUpVal){
                xAccel = 0;
            }
            if(Math.abs(linearAcceleration.yAccel) < accelMessingUpVal){
                yAccel = 0;
            }
            currVel = new Pose2d(prevVel.getX() + xAccel*currTime, prevVel.getY() + yAccel*currTime, Math.toRadians(90));
            //Might be prev vel
            currPose = new Pose2d(prevPose.getX() + currVel.getX()*currTime + 0.5 * xAccel * currTime * currTime, prevPose.getY() + currVel.getY()*currTime + 0.5 * yAccel * currTime * currTime, IMU2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

            MagneticFlux magField = IMU2.getMagneticFieldStrength();
            Acceleration overallAccel = IMU2.getOverallAcceleration();
            AngularVelocity angVel = IMU2.getAngularVelocity();
            Quaternion squaterOr = IMU2.getQuaternionOrientation();
            BNO055IMU.SystemError error = IMU2.getSystemError();
            boolean accelerationCalibrated = IMU2.isAccelerometerCalibrated();

            telemetry.addData("xAccel", xAccel);
            telemetry.addData("yAccel", yAccel);
            telemetry.addData("xVel", currVel.getX());
            telemetry.addData("yVel", currVel.getY());
            telemetry.addData("xPos", currPose.getX());
            telemetry.addData("yPos", currPose.getY());

            telemetry.addData("linearAccel", linearAcceleration);
            telemetry.addData("Derived Vel", currVel);
            telemetry.addData("Derived Pos", currPose);
            telemetry.addData("currTime", currTime);

            telemetry.addData("position 2", position2);
            telemetry.addData("velocity 2", velocity2);
            telemetry.addData("acceleration 2", acceleration2);
            telemetry.addData("Angle2", angle2);
            telemetry.addData("Is Calibrated?", IMUCal);
            telemetry.addData("gravity", gravity);
            telemetry.addData("magField", magField);
            telemetry.addData("overallAccel", overallAccel);
            telemetry.addData("angVel", angVel);
            telemetry.addData("squaterOr", squaterOr);
            telemetry.addData("error", error);
            telemetry.addData("AcclerationCalibrated", accelerationCalibrated);


            telemetry.update();
            integrationTime.reset();
        }
    }
    static double integrate(double a, double b, int n, Function<Double, Double> f) {
        double h = (b - a) / n; // Step size
        double sum = 0.5 * (f.apply(a) + f.apply(b));
        for (int i = 1; i < n; i++) {
            double x = a + i * h;
            sum += f.apply(x);
        }
        return sum * h;
    }
}

