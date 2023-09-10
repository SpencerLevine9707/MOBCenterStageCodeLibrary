package org.firstinspires.ftc.teamcode.conceptCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardwareMaps.JacksJunk;

@Config
@Autonomous
@Disabled
public class Jerktonomous extends LinearOpMode {
    JacksJunk wBot = new JacksJunk(this);
    public static int sleepTimeJerk = 150;
    public static int speedUpMultiplier = 1;
    public static double jerkingAngle = 0.15;
    public static double wTechnique1 = 0.9;
    public static double wTechnique2 = 1;
    public static int wJerkUp = 100;
    public static int stopNumGo = 60;
    public static int stopNumStop = -10;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean isSlowDown = false;
        int k = 0;
        wBot.init(false);
        wBot.closeGrabber();
        wBot.jacksEncodedCock(wJerkUp, 1);

        waitForStart();

        while (opModeIsActive()){
            if(k >= stopNumGo){
                isSlowDown = true;
            }
            else if(k <= stopNumStop){
                requestOpModeStop();
            }

            wBot.rotatorServo.setPosition(wTechnique1);
            wBot.jacksEncodedShaft(JacksJunk.armServoPickUpPos);
            sleep(sleepTimeJerk - (k*speedUpMultiplier));
            wBot.jacksEncodedShaft(JacksJunk.armServoPickUpPos + jerkingAngle);
            wBot.rotatorServo.setPosition(wTechnique2);
            sleep(sleepTimeJerk - (k*speedUpMultiplier));

            if(isSlowDown){
                k--;
            }
            else{
                k++;
            }

        }
    }
}
