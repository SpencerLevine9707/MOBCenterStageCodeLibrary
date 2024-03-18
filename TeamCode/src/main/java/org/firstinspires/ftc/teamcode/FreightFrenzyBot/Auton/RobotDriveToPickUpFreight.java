package org.firstinspires.ftc.teamcode.FreightFrenzyBot.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.ActionRunnerFreightFrenzy;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.AntiDriftDrive;
import org.firstinspires.ftc.teamcode.FreightFrenzyBot.HardwareMaps.MonkeyOperationsMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@Autonomous
public class RobotDriveToPickUpFreight extends LinearOpMode {
    MonkeyOperationsMap wBot = new MonkeyOperationsMap(this);
    SampleMecanumDrive driver;
    AntiDriftDrive drive;
    public static double maxSpeed = 0.25;
    public static double angleToAdd = 90;
    public static double defaultAngle = 90;
    public static double angleToAddHeading = 90;
    public static double noMoveTheshold = 5;
    public static int midX = 160;
    public static int midY = 120;
    public static double maxX = 320;
    public static double maxThetaHeading = -100;
    public static double areaToPickUp = 800;
    public static double xPosStart = 0, yPosStart = 0, headingStart = 90;
    public static double xPosEnd = 0, yPosEnd = 15, headingEnd = 90;
    public static double xPosMax = -10, yPosMax = 40;
    public static double distToPickUp = 15;
    public static double maxVelPickUpFreight = 40;
    public static double maxVelBackToStart = 30;
    public ElapsedTime timeSinceDropOff = new ElapsedTime();
    public static int sleepTimeFlipSharedFlip = 1500, sleepTimePickUp = 200, sleepTimeUnload = 200, sleepTimeFlipDown = 300;
    public static double  timeToWaitForFlipBack = 1.5;
    ActionRunnerFreightFrenzy actionRunner = new ActionRunnerFreightFrenzy(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        wBot.init();
        wBot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        wBot.flipVisionDetect();
        wBot.openGrabber();
        double theta = 0;
        double area = 0;
        double speedToMoveAt = maxSpeed;
        drive = new AntiDriftDrive(wBot.frontLeft, wBot.frontRight, wBot.backLeft, wBot.backRight);
        driver = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(xPosStart, yPosStart, Math.toRadians(headingStart));
        Pose2d end = new Pose2d(xPosEnd, yPosEnd, Math.toRadians(headingEnd));
        waitForStart();

        timeSinceDropOff.reset();
        posesToGoTo.add(new PosesAndActions(start, ""));
        posesToGoTo.add(new PosesAndActions(end, ""));
        follower.init(posesToGoTo, false, true);
        follower.goToPoints(true);



        while(opModeIsActive()){
            follower.drive.update();
            Pose2d currPose = follower.drive.getPoseEstimate();
            HuskyLens.Block[] blocks = wBot.huskyLens.blocks();

            for (HuskyLens.Block arrows : blocks) {
                telemetry.addData("Arrow", arrows.toString());
            }
            telemetry.addData("ArrowLen", blocks.length);

            for (HuskyLens.Algorithm c : HuskyLens.Algorithm.values()){
                telemetry.addData("NotSure", c);
            }

            int xDist = 0;
            int yDist = 0;

            if(blocks.length>0){
//                xDist = blocks[0].x - midX;
//                yDist = blocks[0].y - midY;
//                theta = Math.atan2(yDist, xDist) + Math.toRadians(angleToAdd);
                theta = Math.toRadians(angleToAdd) + Math.toRadians((((blocks[0].x/maxX)*maxThetaHeading*2)-maxThetaHeading));
                area = blocks[0].height * blocks[0].width;
            }
            else{
                theta = Math.toRadians(defaultAngle);
                area = 0;
            }
            if(timeSinceDropOff.seconds() < timeToWaitForFlipBack){
                area = 0;
            }
//            if(Math.abs(thetaToMoveTowards) > Math.toRadians(noMoveTheshold)){
//                speedToMoveAt = 0;
//            }
//            else{
//                speedToMoveAt = maxSpeed;
//            }
            telemetry.addData("Area", area);
            if(area > areaToPickUp){
                posesToGoTo.clear();
                wBot.flipDown();
                sleep(sleepTimeFlipDown);
                if(currPose.getY()+distToPickUp > yPosMax){
                    posesToGoTo.add(new PosesAndActions(new Pose2d(currPose.getX(), yPosMax, Math.toRadians(headingStart)), ""));
                }
                else{
                    posesToGoTo.add(new PosesAndActions(new Pose2d(currPose.getX(), currPose.getY()+distToPickUp, Math.toRadians(headingStart)), ""));
                }

                follower.reinit(posesToGoTo);
                follower.goToPoints(true, maxVelPickUpFreight);
                wBot.closeGrabber();
                sleep(sleepTimePickUp);
                wBot.flipShared();
//                sleep(sleepTimeFlipSharedFlip);
                posesToGoTo.clear();
                posesToGoTo.add(new PosesAndActions(start, ""));
                follower.reinit(posesToGoTo);
                follower.goToPoints(true, maxVelBackToStart);
                wBot.openGrabber();
                sleep(sleepTimeUnload);
                wBot.flipVisionDetect();
//              sleep(sleepTimeWaitForFlipBack);
                timeSinceDropOff.reset();
            }
            if(currPose.getY()>yPosMax){
                posesToGoTo.clear();
                posesToGoTo.add(new PosesAndActions(end, ""));
                follower.reinit(posesToGoTo);
                follower.goToPoints(true, maxVelBackToStart);
            }





            if(blocks.length>0){
                telemetry.addData("x", blocks[0].x);
                telemetry.addData("xMath1", (((blocks[0].x/maxX)*maxThetaHeading*2)-maxThetaHeading));
            }

            telemetry.addData("theta", theta);
            telemetry.addData("thetaDeg", Math.toDegrees(theta));

            drive.moveTowardsAngleTheta(theta, speedToMoveAt, currPose.getHeading() - Math.toRadians(headingStart));
            telemetry.update();
        }

    }
}
