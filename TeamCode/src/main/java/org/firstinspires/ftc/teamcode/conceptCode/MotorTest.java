package org.firstinspires.ftc.teamcode.conceptCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
@Disabled
public class MotorTest extends LinearOpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight;

    public ElapsedTime runtime = new ElapsedTime();

    public static double timeToRun = 60;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double finalTime = 0;

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            if (runtime.seconds() < timeToRun) {
                frontLeft.setPower(1);
                frontRight.setPower(1);
                backLeft.setPower(1);
                backRight.setPower(1);

                double ticksCurrent0 = frontLeft.getCurrentPosition();
                double ticksCurrent1 = frontRight.getCurrentPosition();
                double ticksCurrent2 = backLeft.getCurrentPosition();
                double ticksCurrent3 = backRight.getCurrentPosition();


                telemetry.addLine("motor 0 (frontLeft) Position/Ticks: " + ticksCurrent0);
                telemetry.addLine("motor 1 (frontRight) Position/Ticks: " + ticksCurrent1);
                telemetry.addLine("motor 2 (frontLeft) Position/Ticks: " + ticksCurrent2);
                telemetry.addLine("motor 3 (frontLeft) Position/Ticks: " + ticksCurrent3);

                telemetry.addLine("\nmotor 0 (frontLeft) current TPS: " + ticksCurrent0 / runtime.seconds());
                telemetry.addLine("motor 1 (frontRight) current TPS: " + ticksCurrent1 / runtime.seconds());
                telemetry.addLine("motor 2 (frontLeft) current TPS: " + ticksCurrent2 / runtime.seconds());
                telemetry.addLine("motor 3 (frontLeft) current TPS: " + ticksCurrent3 / runtime.seconds());

                telemetry.addLine("\nmotor 0 (frontLeft) current RPM: " + (ticksCurrent0 / 537.7) / (runtime.seconds()/ 60));
                telemetry.addLine("motor 1 (frontRight) current RPM: " + (ticksCurrent1 / 537.7) / (runtime.seconds()/ 60));
                telemetry.addLine("motor 2 (frontLeft) current RPM: " + (ticksCurrent2 / 537.7) / (runtime.seconds()/ 60));
                telemetry.addLine("motor 3 (frontLeft) current RPM: " + (ticksCurrent3 / 537.7) / (runtime.seconds()/ 60));

                telemetry.update();
            } else {

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                if (finalTime == 0 && !(frontLeft.isBusy() || frontLeft.isBusy() || frontLeft.isBusy() || frontLeft.isBusy())) {
                    finalTime = runtime.seconds();

                    double ticksFinal0 = frontLeft.getCurrentPosition();
                    double ticksFinal1 = frontRight.getCurrentPosition();
                    double ticksFinal2 = backLeft.getCurrentPosition();
                    double ticksFinal3 = backRight.getCurrentPosition();


                    telemetry.addLine("motor 0 (frontLeft) FINAL Position: " + ticksFinal0);
                    telemetry.addLine("motor 1 (frontRight) FINAL Position: " + ticksFinal1);
                    telemetry.addLine("motor 2 (frontLeft) FINAL Position: " + ticksFinal2);
                    telemetry.addLine("motor 3 (frontLeft) FINAL Position: " + ticksFinal3);

                    telemetry.addLine("\nmotor 0 (frontLeft) FINAL TPS: " + ticksFinal0 / finalTime);
                    telemetry.addLine("motor 1 (frontRight) FINAL TPS: " + ticksFinal1 / finalTime);
                    telemetry.addLine("motor 2 (frontLeft) FINAL TPS: " + ticksFinal2 / finalTime);
                    telemetry.addLine("motor 3 (frontLeft) FINAL TPS: " + ticksFinal3 / finalTime);

                    telemetry.addLine("\nmotor 0 (frontLeft) FINAL RPM: " + (ticksFinal0 / 537.7) / (finalTime/60));
                    telemetry.addLine("motor 1 (frontRight) FINAL RPM: " + (ticksFinal1 / 537.7) / (finalTime/60));
                    telemetry.addLine("motor 2 (frontLeft) FINAL RPM: " + (ticksFinal2 / 537.7) / (finalTime/60));
                    telemetry.addLine("motor 3 (frontLeft) FINAL RPM: " + (ticksFinal3 / 537.7) / (finalTime/60));

                    telemetry.addLine("\nRecommended adjustments for each motor:");

                    double numerator = Math.min(Math.min(ticksFinal0, ticksFinal1), Math.min(ticksFinal2, ticksFinal3));

                    telemetry.addLine("motor 0 (frontLeft) recommended ratio: " + numerator / ticksFinal0);
                    telemetry.addLine("motor 1 (frontRight) recommended ratio: " + numerator / ticksFinal1);
                    telemetry.addLine("motor 2 (backLeft) recommended ratio: " + numerator / ticksFinal2);
                    telemetry.addLine("motor 3 (backRight) recommended ratio: " + numerator / ticksFinal3);

                    telemetry.update();
                }
            }

        }
    }
}
