package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.LevineLocalizationMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.MathsAndStuff;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointType;
import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
public class PointFollower {
    LinearOpMode myOpMode;
    LevineLocalizationMap wMap;
    Telemetry telemetry;
    SampleMecanumDrive drive;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime velTime = new ElapsedTime();
    public static double isBuggingRuntimeToStop;
    public static double isBuggingRuntimeToStopError = 0.5;
    public static double minMotorPow = 0.05;
    public static double notPowPow = 0;

    public static double slowSpeed = 0.5;
    public static double almostDoneSpeed = 0.45;
    public static double evenSlowerSpeed = 0.4;
    public static double offCourseSpeed = 1.5;
    public static double offCourseError = 1;
    public static double motorStuckPow = 1;
    public static int finalCounter = 0;
    public static int endCounter = 0;
    public static int almostDoneCounter = 0;
    ArrayList<Pose2d> posesToGoTo = new ArrayList<Pose2d>();
    ArrayList<String> trajTypes = new ArrayList<>();
    ArrayList<PointType> pointTypes = new ArrayList<>();
    ArrayList<Pose2d> inBetweenPoints = new ArrayList<>();
    ArrayList<PointType> pointTypesInBetween = new ArrayList<>();
    public boolean xDone = false;
    boolean yDone = false;
    double currVelocityLessCount = 0;
    public static double velLessZeroMax = 10;
    boolean angDone = false;
    int donePointCounter = 0;
    Pose2d startOfNewGo = new Pose2d();
    public static double noVelVel = 1.5;
    Pose2d prevPoseForVel = new Pose2d();
    double currVelocity = 0;
    public static double targetVelocity = 50;
    public static double maxVel = 50;
    public static double almostDoneVel = 20;
    public static double slowVel = almostDoneVel;
    public static double evenSlowerVel = 2;
    public static double currPower = 0.5;
    public static double accelerationConst = 200;
    public static double distToTarg;
    public static PIDCoefficients PIDVals = new PIDCoefficients(0.25, 0, 0.5);

    public PointFollower(LinearOpMode opmode) {
        myOpMode = opmode;
        wMap = new LevineLocalizationMap(this.myOpMode);
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void init(ArrayList<Pose2d> posesToGoTo) {
        wMap.init(posesToGoTo.get(0));
        startOfNewGo = posesToGoTo.get(0);
        drive = new SampleMecanumDrive(this.myOpMode.hardwareMap);
        drive.setPoseEstimate(posesToGoTo.get(0));

        this.posesToGoTo.clear();
        trajTypes.clear();
        pointTypes.clear();
        inBetweenPoints.clear();
        pointTypesInBetween.clear();

        this.posesToGoTo.addAll(posesToGoTo);

        for (int i = 1; i < this.posesToGoTo.size(); i++) {
            trajTypes.add("");
//                pointTypes.add(new PointType("mid"));
        }
        for (int i = 0; i < this.posesToGoTo.size(); i++) {
//                trajTypes.add("");
            pointTypes.add(new PointType("mid"));
        }
        pointTypes.set(pointTypes.size() - 1, new PointType("end"));

        double theoreticalTheta;
        for (int i = 1; i < this.posesToGoTo.size(); i++) {
            Pose2d startingPos = this.posesToGoTo.get(i - 1);
            Pose2d targetPos = this.posesToGoTo.get(i);

            double xDist = targetPos.getX() - startingPos.getX();

            double yDist = targetPos.getY() - startingPos.getY();

            double totDistToTarget = Math.hypot(xDist, yDist);

            //Iteration for poses calculator
            int iterationsForPoses = (int) ((totDistToTarget / 2) * LevineLocalizationMap.poseFollowCoef);

            double distToTarget = totDistToTarget / iterationsForPoses;

            distToTarg = distToTarget;


            theoreticalTheta = MathsAndStuff.AngleWrap(Math.atan2(yDist, xDist));

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
//            telemetry.addLine("meatyMiddle: " + meatyMiddle);

            for (int j = 0; j < leftCheekIts; j++) {
                pointTypesInBetween.add(new PointType("mid"));
            }
            for (int j = 0; j < meatyMiddle; j++) {
                pointTypesInBetween.add(new PointType("inside"));
            }
            for (int j = 0; j < rightCheeckIts; j++) {
                pointTypesInBetween.add(new PointType("mid"));
            }
        }
        for (int j = 3; j < 6; j++) {
            if (pointTypesInBetween.size() - j > 0) {
                pointTypesInBetween.set(pointTypesInBetween.size() - j, new PointType("almostdone"));
            }
        }
        for (int j = 1; j < 3; j++) {
            if (pointTypesInBetween.size() - j > 0) {
                pointTypesInBetween.set(pointTypesInBetween.size() - j, new PointType("end"));
            }
        }
        inBetweenPoints.add(new Pose2d(this.posesToGoTo.get(this.posesToGoTo.size() - 1).getX(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).getY(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).getHeading()));
        pointTypesInBetween.add(new PointType("final"));

//        inBetweenPoints.remove(0);


        telemetry.addLine("Poses to go to " + this.posesToGoTo);
        telemetry.addLine("Poses in between " + inBetweenPoints);
        telemetry.addLine("PointTypes in between: " + pointTypesInBetween);
        telemetry.update();
    }

    public void reinit(ArrayList<Pose2d> posesToGoTo) {
        posesToGoTo.add(0, drive.getPoseEstimate());
        init(posesToGoTo);
        telemetry.addLine("poses to go to 1 is " + posesToGoTo.get(0));
    }

    public Pose2d goToPoints(boolean stopAfter) {
        startOfNewGo = drive.getPoseEstimate();
        prevPoseForVel = drive.getPoseEstimate();
        currPower = 0;
        GetVelocityPIDController getVel = new GetVelocityPIDController(PIDVals, targetVelocity);
        velTime.reset();
        Pose2d currPose = drive.getPoseEstimate();

        while (!(inBetweenPoints.isEmpty())) {
            getVel.changeTarget(targetVelocity);
            drive.update();

            currPose = drive.getPoseEstimate();

            double timeForVel = velTime.seconds();

            double totDistForVel = Math.hypot(currPose.getX() - prevPoseForVel.getX(), currPose.getY() - prevPoseForVel.getY());
            currVelocity = Math.abs(totDistForVel / timeForVel);
            isBuggingRuntimeToStop = distToTarg / isBuggingRuntimeToStop + isBuggingRuntimeToStopError;

            if (!inBetweenPoints.isEmpty()) {
//                if(currVelocity < noVelVel){
//                    currVelocityLessCount += 1;
//                }
//                else{
//                    currVelocityLessCount = 0;
//                }
//
//                if(currVelocityLessCount > velLessZeroMax){
//                    wMap.frontLeft.setPower(motorStuckPow);
//                    wMap.frontRight.setPower(motorStuckPow);
//                    wMap.backLeft.setPower(motorStuckPow);
//                    wMap.backRight.setPower(motorStuckPow);
//                    currVelocityLessCount =  0;
//                }
//                if(Math.abs(wMap.backRight.getPower()) < minMotorPow){
//                    wMap.backRight.setPower(notPowPow);
//                }
//                if(Math.abs(wMap.backLeft.getPower()) < minMotorPow){
//                    wMap.backLeft.setPower(notPowPow);
//                }
//                if(Math.abs(wMap.frontRight.getPower()) < minMotorPow){
//                    wMap.frontRight.setPower(notPowPow);
//                }
//                if(Math.abs(wMap.frontLeft.getPower()) < minMotorPow){
//                    wMap.frontLeft.setPower(notPowPow);
//                }
                double isBuggingChecker = runtime.seconds();
                Pose2d targetPose = inBetweenPoints.get(0);
                double roomForPoseError;
                if (stopAfter){
                    roomForPoseError = pointTypesInBetween.get(0).followRadius / 2;
                }
                else{
                    roomForPoseError = new PointType("inside").followRadius / 2;
                }

                angDone = !pointTypesInBetween.get(0).type.equals("final") && !pointTypesInBetween.get(0).type.equals("end");
                double fastestXDist = targetPose.getX() - startOfNewGo.getX();
                double fastestYDist = targetPose.getY() - startOfNewGo.getY();

                double xDist = targetPose.getX() - currPose.getX();
                double yDist = targetPose.getY() - currPose.getY();
                double angDist = targetPose.getHeading() - currPose.getHeading();

                double distToTarget = Math.hypot(xDist, yDist);
                double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.odos.startingPose.getHeading());

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

                if (stopAfter){
                    switch (pointTypesInBetween.get(0).type) {
                        case "final":
                            targetVelocity = evenSlowerVel;
                            finalCounter++;
                            break;
                        case "end":
                            targetVelocity = slowVel;
                            endCounter++;
                            break;
                        case "almostdone":
                            targetVelocity = almostDoneVel;
                            almostDoneCounter++;
                            break;
                        default:
                            targetVelocity = maxVel;
                            break;
                    }
                }
                else{
                    targetVelocity = maxVel;
                }
                if (currVelocity < targetVelocity || currVelocity > targetVelocity) {
                    currPower += getVel.calculate(currVelocity) / accelerationConst;
                }
                if (currPower < 0) {
                    currPower = 0;
                }
                if (currPower > 1) {
                    currPower = 1;
                }

                wMap.setMotorPowers(currPose, targetPose, currPower);

                if (isBuggingChecker > isBuggingRuntimeToStop) {
                    xDone = true;
                    yDone = true;
                    angDone = true;
//                } else if (!(pointTypesInBetween.get(0).type.equals("mid")) && (Math.abs(relDistX) > Math.abs(fastestXDist + offCourseError) || Math.abs(relDistY) > Math.abs(fastestYDist + offCourseError))) {
//                    wMap.setMotorPowers(currPose, targetPose, offCourseSpeed);
                }
                if (Math.abs(angDist) < Math.abs(LevineLocalizationMap.angError)) {
                    angDone = true;
                }

                prevPoseForVel = currPose;
                velTime.reset();

                telemetry.addData("Velocity ", currVelocity);
                telemetry.addData("posesToGoTo ", posesToGoTo);
//                telemetry.addLine("Poses to go to " + posesToGoTo);
//                telemetry.addLine("Poses in between " + inBetweenPoints);
                telemetry.addLine("Target Pose: " + targetPose);
                telemetry.addLine("isBuggingChecker: " + isBuggingChecker);
//                telemetry.addLine("relDistX: " + relDistX + "relDistY: " + relDistY);
                telemetry.addLine("Motor powers: " + wMap.odos.getPowers(currPose, targetPose));
//                telemetry.addLine("Ang Done? " + angDone);
//                telemetry.addLine("X Done " + xDone);
//                telemetry.addLine("Y Done " + yDone);
//                telemetry.addLine("Done Points Counter: " + donePointCounter);
                telemetry.addLine("CurrSpeed: " + currPower);
                telemetry.addLine("Curr Targ Vel " + targetVelocity);
                telemetry.addLine("distToTarg " + distToTarg);
                telemetry.addLine("is bugging time to stop is " + isBuggingRuntimeToStop);
//                telemetry.addLine("finalCounter: " + finalCounter);
//                telemetry.addLine("endCounter: " + endCounter);
//                telemetry.addLine("almostDoneCounter: " + almostDoneCounter);
                telemetry.update();

                if (xDone && yDone && angDone) {
                    runtime.reset();
                    inBetweenPoints.remove(0);
                    pointTypesInBetween.remove(0);
//                    currPower = 0.5;

                    donePointCounter++;
                    xDone = false;
                    yDone = false;
                    startOfNewGo = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getHeading());
//                    angDone = false;
                }
            } else if(stopAfter) {
                wMap.stopMotors();
            }
        }
        return currPose;
    }
}