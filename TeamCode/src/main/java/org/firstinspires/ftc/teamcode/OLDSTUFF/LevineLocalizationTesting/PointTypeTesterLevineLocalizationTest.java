package org.firstinspires.ftc.teamcode.OLDSTUFF.LevineLocalizationTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.LevineLocalizationMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.MathsAndStuff;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointType;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@Autonomous(group = "Levine Local")
public class PointTypeTesterLevineLocalizationTest extends LinearOpMode {
    LevineLocalizationMap wMap = new LevineLocalizationMap(this);
    public ElapsedTime runtime = new ElapsedTime();
    public static double isBuggingRuntimeToSlow = 2;
    public static double isBuggingRuntimeToStop = 4;
    public static double slowSpeed = 0.5;
    public static double almostDoneSpeed = 0.45;
    public static double evenSlowerSpeed = 0.4;
    public static double offCourseSpeed = 1.5;
    public static double offCourseError = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d firstPose = new Pose2d(0, 0, Math.toRadians(90));
        boolean xDone = false;
        boolean yDone = false;
        boolean angDone = false;
        int donePointCounter = 0;

        wMap.init(firstPose);

        ArrayList<Pose2d> posesToGoTo = new ArrayList<>();
        ArrayList<String> trajTypes = new ArrayList<>();
        ArrayList<PointType> pointTypes = new ArrayList<>();
        ArrayList<Pose2d> inBetweenPoints = new ArrayList<>();
        ArrayList<PointType> pointTypesInBetween = new ArrayList<>();

        posesToGoTo.add(firstPose);
        posesToGoTo.add(new Pose2d(0, 20, Math.toRadians(0)));
        posesToGoTo.add(new Pose2d(20, 0, Math.toRadians(90)));
        posesToGoTo.add(new Pose2d(35, 20, Math.toRadians(180)));
//        posesToGoTo.add(new Pose2d(-10, 4, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(-10, -30, Math.toRadians(0)));
//        posesToGoTo.add(new Pose2d(-60, -50, -20));
//        posesToGoTo.add(new Pose2d(66, 6, -100));
//        posesToGoTo.add(new Pose2d(52, -60, -2000));
//        posesToGoTo.add(new Pose2d(1, -10, -100));

        for (int i = 1; i < posesToGoTo.size(); i++) {
            trajTypes.add("");
//                pointTypes.add(new PointType("mid"));
        }
        for (int i = 0; i < posesToGoTo.size(); i++) {
//                trajTypes.add("");
            pointTypes.add(new PointType("mid"));
        }
        pointTypes.set(pointTypes.size()-1, new PointType("end"));

        double theoreticalTheta;
        for(int i = 1; i < posesToGoTo.size(); i++) {
            Pose2d startingPos = posesToGoTo.get(i - 1);
            Pose2d targetPos = posesToGoTo.get(i);

            double xDist = targetPos.getX() - startingPos.getX();

            double yDist = targetPos.getY() - startingPos.getY();

            double totDistToTarget = Math.hypot(xDist, yDist);

            //Iteration for poses calculator
            int iterationsForPoses = (int) ((totDistToTarget / 2) * LevineLocalizationMap.poseFollowCoef);

            double distToTarget = totDistToTarget / iterationsForPoses;

            theoreticalTheta = MathsAndStuff.AngleWrap(Math.atan2(yDist, xDist) + wMap.startingPose.getHeading());

            for (int j = 0; j < iterationsForPoses; j++) {
//                theoreticalAngle += angleForMrGeorge;
                double relDistX = Math.cos(theoreticalTheta) * (distToTarget * j);
                double relDistY = Math.sin(theoreticalTheta) * (distToTarget * j);
                inBetweenPoints.add(new Pose2d(startingPos.getX() + relDistX, startingPos.getY() + relDistY, targetPos.getHeading()));
            }

            //Get the cheek lengths
            int leftCheekIts = (int) ((LevineLocalizationMap.followRadius / 2) * LevineLocalizationMap.poseFollowCoef);
            int rightCheeckIts;
            if (pointTypes.get(i).type.equals("mid")) {
                rightCheeckIts = leftCheekIts;
            } else {
                rightCheeckIts = 0;
            }
            if (leftCheekIts + rightCheeckIts >= iterationsForPoses) {
                leftCheekIts = iterationsForPoses / 2;
                rightCheeckIts = iterationsForPoses - leftCheekIts;
            }

            int meatyMiddle = iterationsForPoses - (leftCheekIts + rightCheeckIts);

            for (int j = 0; j < leftCheekIts; j++) {
                pointTypesInBetween.add(new PointType("mid"));
            }
            for (int j = 0; j < meatyMiddle; j++) {
                pointTypesInBetween.add(new PointType("inside"));
            }
            for (int j = 0; j < rightCheeckIts; j++) {
                pointTypesInBetween.add(new PointType("mid"));
            }
            if (rightCheeckIts == 0) {
                pointTypesInBetween.set(pointTypesInBetween.size() - 2, new PointType("end"));
                pointTypesInBetween.set(pointTypesInBetween.size() - 1, new PointType("end"));
            }
        }
        inBetweenPoints.add(new Pose2d(posesToGoTo.get(posesToGoTo.size()-1).getX(), posesToGoTo.get(posesToGoTo.size()-1).getY(), posesToGoTo.get(posesToGoTo.size()-1).getHeading()));
        pointTypesInBetween.add(new PointType("final"));

//        inBetweenPoints.remove(0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Poses to go to " + posesToGoTo);
        telemetry.addLine("Poses in between " + inBetweenPoints);
        telemetry.update();

        Pose2d startOfNewGo = new Pose2d();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            Pose2d currPose = drive.getPoseEstimate();

            if (!inBetweenPoints.isEmpty()) {
                double isBuggingChecker = runtime.seconds();
                Pose2d targetPose = inBetweenPoints.get(0);
                double roomForPoseError = pointTypesInBetween.get(0).followRadius/2;
                angDone = true;
                if(pointTypesInBetween.get(0).type.equals("final") || pointTypesInBetween.get(0).type.equals("end")){
                    angDone = false;
                }
                double fastestXDist = targetPose.getX() - startOfNewGo.getX();
                double fastestYDist = targetPose.getY() - startOfNewGo.getY();

                double xDist = targetPose.getX() - currPose.getX();
                double yDist = targetPose.getY() - currPose.getY();
                double angDist = targetPose.getHeading() - currPose.getHeading();

                double distToTarget = Math.hypot(xDist, yDist);
                double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.startingPose.getHeading());

                //Error X
                double relDistX = Math.cos(theta) * distToTarget;
                double relErrorX = (Math.cos(theta) * roomForPoseError);

                double relDistY = Math.sin(theta) * distToTarget;
                double relErrorY = (Math.sin(theta) * roomForPoseError);
                if (Math.abs(relDistX) < Math.abs(relErrorX)) {
                    xDone = true;
                }
                if (Math.abs(relDistY) < Math.abs(relErrorY)) {
                    yDone = true;
                }
                if(pointTypesInBetween.get(0).type.equals("final")){
                    wMap.setMotorPowers(currPose, targetPose, evenSlowerSpeed);
                }
                else if(pointTypesInBetween.get(0).type.equals("end")){
                    wMap.setMotorPowers(currPose, targetPose, almostDoneSpeed);
                }
                else if(isBuggingChecker > isBuggingRuntimeToStop){
                    xDone = true;
                    yDone = true;
                }
                else if(!(pointTypesInBetween.get(0).type.equals("mid")) && (Math.abs(relDistX) > (fastestXDist + offCourseError) || Math.abs(relDistY) > (fastestYDist + offCourseError))){
                    wMap.setMotorPowers(currPose, targetPose, offCourseSpeed);
                }
                else if(isBuggingChecker > isBuggingRuntimeToSlow){
                    wMap.setMotorPowers(currPose, targetPose, slowSpeed);
                }
                else{
                    wMap.setMotorPowers(currPose, targetPose, slowSpeed);
                }
                if(Math.abs(angDist) < Math.abs(LevineLocalizationMap.angError)){
                    angDone = true;
                }
//                if(!((angDist > LevineLocalizationMap.angError)&&(angDist < -LevineLocalizationMap.angError))){
//                    angDone = true;
//                }

//                telemetry.addLine("Ticks between: " + wMap.odos.getTicksBetween(wMap.odos.currPose, targetPose));
                telemetry.addLine("Poses to go to " + posesToGoTo);
                telemetry.addLine("Poses in between " + inBetweenPoints);
                telemetry.addLine("Target Pose: " + targetPose);
                telemetry.addLine("isBuggingChecker: " + isBuggingChecker);
                telemetry.addLine("relDistX: " + relDistX + "relDistY: " + relDistY);
                telemetry.addLine("Motor powers: " + wMap.getPowers(currPose, targetPose));
                telemetry.addLine("Ang Done? " + angDone);
                telemetry.addLine("X Done " + xDone);
                telemetry.addLine("Y Done " + yDone);
                telemetry.addLine("Done Points Counter: " + donePointCounter);
//                telemetry.addLine("offCourseErrorStuff cos " + offCourseError*Math.cos(theta) + "Sin " + offCourseError*Math.sin(theta));
                telemetry.update();

                if(xDone&&yDone&&angDone){
                    runtime.reset();
                    inBetweenPoints.remove(0);
                    pointTypesInBetween.remove(0);

                    donePointCounter++;
                    xDone = false;
                    yDone = false;
                    startOfNewGo = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getHeading());
//                    angDone = false;
                }
            }
            else{
                requestOpModeStop();
            }
        }
    }
}