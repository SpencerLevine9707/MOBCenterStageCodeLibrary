package org.firstinspires.ftc.teamcode.usefulTuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareMaps.JacksJunk;

import java.util.ArrayList;


@TeleOp(group = "test")
@Disabled
public class BatteryTuner extends LinearOpMode {

    JacksJunk wBot = new JacksJunk(this);

    public DcMotor flipperMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        flipperMotor = hardwareMap.get(DcMotor.class, ("flipperMotor"));
        wBot.init(true);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        ArrayList<Double> batteryStorage = new ArrayList<Double>();
        waitForStart();

        wBot.runtime.reset();

        while(opModeIsActive()){
            wBot.armMotor1.setPower(1);
            flipperMotor.setPower(1);
            wBot.backLeft.setPower(1);
            wBot.frontLeft.setPower(1);
            wBot.frontRight.setPower(1);
            wBot.backRight.setPower(1);



            if(wBot.runtime.seconds() > 5){
                batteryStorage.add(wBot.batteryVoltageSensor.getVoltage());
                wBot.runtime.reset();
            }
            telemetry.addData("Battery Voltage current: ", wBot.batteryVoltageSensor.getVoltage());
            telemetry.addData("Battery Voltage list: ", batteryStorage);

            telemetry.update();
        }
    }
}
