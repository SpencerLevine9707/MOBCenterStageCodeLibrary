//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.checkerframework.checker.units.qual.C;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
//
//@Config
//@Autonomous
//@Disabled
//public class ContinuousServosCoveyer extends LinearOpMode {
//    MonkeyMap wBot = new MonkeyMap(this);
//
//    DcMotor motor1;
//
//    public static double servoPower = 0.5;
//    public static double servoPowerStop = 0.5;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        motor1 = hardwareMap.get(DcMotor.class, "testMotor");
//
//        wBot.init();
//        waitForStart();
//
//        while(opModeIsActive()){
//            double lt = gamepad1.left_trigger;
//            if(lt > 0){
//                wBot.conveyerServoLeft.setPosition(lt);
//            }
//            else{
//                wBot.conveyerServoLeft.setPosition(servoPowerStop);
//            }
//            double rt = gamepad1.right_trigger;
//            if(rt > 0){
//                motor1.setPower(rt);
//            }
//            else{
//                motor1.setPower(0);
//            }
//
//
////            wBot.conveyerServoLeft.setPosition(servoPower);
//        }
//    }
//}
