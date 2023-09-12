//package org.firstinspires.ftc.teamcode.conceptCode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.hardwareMaps.JacksJunk;
//import org.firstinspires.ftc.teamcode.hardwareMaps.openCVTestLeft;
//import org.firstinspires.ftc.teamcode.hardwareMaps.OpenCVLookForPole;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//
//@Config
//@Autonomous(group="drive")
//@Disabled
//public class VisionWithRoadrunner extends LinearOpMode {
//    JacksJunk wBot = new JacksJunk(this);
//
//    // Define Webcam
//    OpenCvCamera webcam;
//
//    // Create Pipeline
//    static openCVTestLeft.OpenCV_Pipeline pipeline;
//
//    static OpenCVLookForPole.OpenCV_Pipeline wPipeline;
//
//    public static boolean doesConeAdjust = false, doesPoleDistanceAdjust = false, doesEndEarly = false, doesVisionAdjust = true;
//
//    @Override
//    public void runOpMode() {
//        int zone = 0;
//
//        wBot.init(false);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        wBot.definePosesAndTrajectoriesAuton(false, drive);
//
//        Trajectory pickUpCone;
//
//        Trajectory placeOtherCones;
//
//
//        wBot.jacksEncodedShaft(JacksJunk.armServoUpPos);
//
//
//        // Set up webcam
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        // Set up pipeline
//        pipeline = new openCVTestLeft.OpenCV_Pipeline();
//        webcam.setPipeline(pipeline);
//
//        // Start camera streaming
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        while (opModeInInit()) {
//            //Telemetry readings for the HSV values in each region
//            telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
//
//            if ((pipeline.HSV_Value_2[0] < 88)) {
//                zone = 2;
//                telemetry.addLine("Yellow");
//            } else if (pipeline.HSV_Value_2[0] < 183) {
//                zone = 1;
//                telemetry.addLine("Cyan");
//            } else {
//                zone = 0;
//                telemetry.addLine("Magenta");
//            }
//
//            telemetry.update();
//        }
//
//        wBot.runtime.reset();
//
////        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, wBot.matchStart);
//
//        wBot.runtime.reset();
//
//        wBot.closeGrabber();
//        sleep(JacksJunk.sleepTimeGrabberCloseStart);
//        wBot.rotateNext();
//
//        //lines up for cone 1; no adjustments
//        drive.followTrajectory(wBot.placeConePreload);
//
//
//        double dispX;
//        double dispY;
//        double dist;
//
//        double headingToPole;
//
//        boolean isFirstTime;
//
//        //array for for loop iykyk
//        //Ill give credit where it is due: david wrote part of this but based if off my code
//        for (int i = 0; i <= JacksJunk.coneHeights.length && opModeIsActive(); i++) {
//
//            isFirstTime = i == 0;
//
//            telemetry.addLine("\nPlace pos " + i + ": " + wBot.placePosHigh);
//            telemetry.addLine("PickUp pos " + i + ": " + wBot.CONES);
//            telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//            telemetry.update();
//
//            //line up for place
//            if (i != 0) {
//                placeOtherCones = wBot.adjustOtherPlaces(drive, wBot.CONES, wBot.placePosHigh, JacksJunk.placeTanHighJunct, JacksJunk.h3);
//                drive.followTrajectory(placeOtherCones);
//            }
//
//            //get distance from pole
//            dist = wBot.jacksAsshole.getDistance(DistanceUnit.INCH);
//            telemetry.addLine("Place Distance " + i + ": " + dist);
//            telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//            telemetry.update();
//
//            //Becomes the thinker and readjust place dist
//            if (doesPoleDistanceAdjust) {
//                if (i == 0) {
//                    if (wBot.giveAnalConsent(dist)) {
//
//                        //calculates distance to adjust
//                        dispX = wBot.getDispPoles(dist, true, isFirstTime);
//                        dispY = wBot.getDispPoles(dist, false, isFirstTime);
//
//                        //adjust them with a temp with the old position
//                        Pose2d tempStart = wBot.placePosHigh;
//                        wBot.placePosHigh = wBot.getNewPlaceHigh(dispX, dispY, true);
//                        drive.followTrajectory(wBot.reAdjustPlace(drive, tempStart, wBot.placePosHigh));
//
//                        telemetry.addLine("\nAdjusted Place: \nDisp X: " + dispX + " \nDispY: " + dispY + "\n");
//                        telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//                        telemetry.update();
//                    }
//                }
//                else{
//                    if (wBot.giveAnalConsentAgain(dist)) {
//
//                        //calculates distance to adjust
//                        dispX = wBot.getDispPoles(dist, true, isFirstTime);
//                        dispY = wBot.getDispPoles(dist, false, isFirstTime);
//                        //adjust them with a temp with the old position
//                        Pose2d tempStart = wBot.placePosHigh;
//                        wBot.placePosHigh = wBot.getNewPlaceHigh(dispX, dispY, true);
//                        drive.followTrajectory(wBot.reAdjustPlace(drive, tempStart, wBot.placePosHigh));
//
//                        telemetry.addLine("\nAdjusted Place: \nDisp X: " + dispX + " \nDispY: " + dispY + "\n");
//                        telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//                        telemetry.update();
//                    }
//                }
//            }
//            else if(doesVisionAdjust){
//                FtcDashboard.getInstance().startCameraStream(webcam, 0);
//                cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//                // Set up pipeline
//
//                wPipeline = new OpenCVLookForPole.OpenCV_Pipeline();
//                webcam.setPipeline(wPipeline);
//
//                // Start camera streaming
//                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                    @Override
//                    public void onOpened() {
//                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                    }
//
//                    @Override
//                    public void onError(int errorCode) {
//                        doesVisionAdjust = false;
//                    }
//                });
//                //Telemetry readings for the HSV values in each region
////              telemetry.addData("Region 2", "%7d, %7d, %7d", pipeline.HSV_Value_2[0], pipeline.HSV_Value_2[1], pipeline.HSV_Value_2[2]);
//                int valForYellow = 88;
//                int hValueClosestToYellow = wPipeline.HSVValsStorage.get(0).hValue;
//                int closestToYellow = Math.abs(hValueClosestToYellow - valForYellow);
//                int indexClosestToYellow = 0;
//                //88 val for yellow
//                for(int j = 0; i < wPipeline.HSVValsStorage.size(); i++) {
//                    int tempClosestToYellow = Math.abs(wPipeline.HSVValsStorage.get(i).hValue - closestToYellow);
//                    if (tempClosestToYellow < closestToYellow){
//
//                        closestToYellow = tempClosestToYellow;
//
//                        hValueClosestToYellow = wPipeline.HSVValsStorage.get(i).hValue;
//
//                        indexClosestToYellow = j;
//                    }
//                }
//                int polePointInBox = ((int)(wPipeline.HSVValsStorage.size()/2))+1;
//                if(indexClosestToYellow != polePointInBox){
//                    if(indexClosestToYellow > polePointInBox){
//                        indexClosestToYellow -=  wPipeline.HSVValsStorage.size();
//                    }
//                    headingToPole = indexClosestToYellow * JacksJunk.turningNeededPerBox + wBot.placePosHigh.getHeading();
//
//                    drive.turn(headingToPole - wBot.placePosHigh.getHeading());
//
//                    wBot.placePosHigh = wBot.getNewHeadingPlaceHigh(headingToPole, true);
//
//                }
//            }
//
//
//            //place cone
//            wBot.placeConeAuto();
//
//            //park early if used too much time
//            if (doesEndEarly && wBot.runtime.seconds() > JacksJunk.parkEarlyPlace) wBot.getParkAndEndPlace(drive, zone, wBot.placePosHigh, wBot.park1, wBot.park2, wBot.park3);
//
//            //break so it doesn't do extra pickUp
//            if (i == JacksJunk.coneHeights.length) break;
//
//            //go to pick up using new place pos, no adjustment yet
////            if(i >= 4){
////                pickUpCone = wBot.adjustPickUpGetOutTheWay(drive, wBot.placePosHigh, wBot.CONES, JacksJunk.coneHeights[i]);
////            }
////            else{
////                pickUpCone = wBot.adjustPickUp(drive, wBot.placePosHigh, wBot.CONES, JacksJunk.coneHeights[i]);
////            }
//            pickUpCone = wBot.adjustPickUp(drive, wBot.placePosHigh, wBot.CONES, JacksJunk.coneHeights[i]);
//
//            drive.followTrajectory(pickUpCone);
//            //update distance
//            dist = wBot.jacksTip.getDistance(DistanceUnit.INCH);
//
//            telemetry.addLine("PickUp Distance " + i + ": " + dist);
//            telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//            telemetry.update();
//
//            //Becomes the thinker and readjusts to pick up cone
//            if (doesConeAdjust) {
//                if (wBot.givePenalConsent(dist)) { //too far or too close
//
//                    //calculate displacement
//                    dispX = wBot.getDispCones(dist); //negative bc distance is reversed
//
//                    //readjust
//                    Pose2d tempStart = wBot.CONES;
//                    wBot.CONES = wBot.getNewCONES(dispX);
//                    drive.followTrajectory(wBot.reAdjustPickUp(drive, tempStart, wBot.CONES));
//                    telemetry.addLine("\nAdjusted Pickup: \nDisp X: " + dispX + "\n");
//                    telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//                    telemetry.update();
//                }
//            }
//
//            //pick up cone
////            if(i >= 4){
////                wBot.pickUpConeAutoGetOutTheWay();
////            }
////            else{
////                wBot.pickUpConeAuto();
////            }
//            wBot.pickUpConeAuto();
//
//
//
//            //end early from cones pos
//            if (doesEndEarly && wBot.runtime.seconds() > JacksJunk.parkEarlyCones) wBot.getParkAndEndPickUp(drive, zone, wBot.CONES, wBot.park1, wBot.park2, wBot.park3);
//
//            telemetry.addLine("Runtime: " + wBot.runtime.seconds());
//            telemetry.update();
//        }
//        wBot.getParkAndEndPlace(drive, zone, wBot.placePosHigh, wBot.park1, wBot.park2, wBot.park3);
//        while (opModeIsActive()){
//            if(gamepad1.a){
//                if(zone == 0){
//                    drive.followTrajectory(wBot.backToStart1);
//                }
//                else if(zone == 1){
//                    wBot.jacksEncodedCock(0, JacksJunk.armPowerDown);
//                }
//                else if(zone == 2){
//                    drive.followTrajectory(wBot.backToStart3);
//                }
//                drive.followTrajectory(wBot.backToStart);
//                zone = 3;
//            }
//            telemetry.update();
//        }
//    }
//}
