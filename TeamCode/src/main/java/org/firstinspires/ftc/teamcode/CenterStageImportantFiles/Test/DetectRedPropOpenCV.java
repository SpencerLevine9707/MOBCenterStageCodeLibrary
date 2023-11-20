package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectRed;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "Center Stage")
@Disabled
public class DetectRedPropOpenCV extends LinearOpMode {
    OpenCvCamera webcam;
    public static int webcamWidth = 320;
    public static int webcamHeight = 240;
    public static double lineDist = 20;
    public static boolean isBlue = false;
    public static double offsetForPickUp = 8;
    static OpenCVDetectRed pipeline;
    MonkeyMap wBot = new MonkeyMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = false;
    public ElapsedTime timeForAuton = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
//        wBot.init();
//        wBot.initPoses();
//        wBot.toggleRotator();
//        wBot.flipUp();
//        wBot.openGrabber();
//        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new OpenCVDetectRed(telemetry);

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(webcamWidth, webcamHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        int zoneDetected = 0;

        waitForStart();

        while(opModeIsActive()){
            OpenCVDetectTeamProp.centerX = pipeline.centerX;
            OpenCVDetectTeamProp.centerY = pipeline.centerY;
            OpenCVDetectTeamProp.isDetected = pipeline.isDetected;
            if(OpenCVDetectTeamProp.centerX < 160){
                zoneDetected = 2;
            }
            else if(OpenCVDetectTeamProp.centerX > 160){
                zoneDetected = 3;
            }
            else if(!OpenCVDetectTeamProp.isDetected){
                zoneDetected = 1;
            }
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }
    }
}