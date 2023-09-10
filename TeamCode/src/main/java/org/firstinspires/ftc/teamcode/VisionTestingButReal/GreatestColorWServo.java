package org.firstinspires.ftc.teamcode.VisionTestingButReal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
@Disabled
public class GreatestColorWServo extends LinearOpMode {
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCVGreatestColorTest pipeline;

    public VoltageSensor batteryVoltageSensor;

    public Servo xServo, yServo;

    public static double xServoMidPos = 0.545;
    public static double yServoMidPos = 0;
    public static int webcamWidth = 320;
    public static int webcamHeight = 240;
    public static int webcamWidthCenter = webcamWidth/2;
    public static int webcamHeightCenter = webcamHeight/2;

    public static int xThreshhold = 70;
    public static int yThreshhold = 50;
    public static double servoMoveSpeedX = -0.0005;
    public static double servoMoveSpeedY = 0.0005;

    public static double throwingPower = 1;

    public DcMotor leftMotor, rightMotor;

    public static double lowerXRange = 0.35, upperXRange = 0.65;
    public static double lowerYRange = 0.49, upperYRange = 0.75;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();

        xServo = hardwareMap.get(Servo.class, ("xServo"));
        yServo = hardwareMap.get(Servo.class, ("yServo"));

        leftMotor = hardwareMap.get(DcMotor.class, ("leftMotor"));
        rightMotor = hardwareMap.get(DcMotor.class, ("rightMotor"));

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        xServo.scaleRange(lowerXRange, upperXRange);
        yServo.scaleRange(lowerYRange, upperYRange);

        pipeline = new OpenCVGreatestColorTest(telemetry);
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

        while (opModeInInit()) {
            telemetry.addLine("Running");
            xServo.setPosition(xServoMidPos);
            yServo.setPosition(yServoMidPos);
            telemetry.update();
        }
        
        while (opModeIsActive()){
            int xDist = OpenCVGreatestColorTest.centerX - webcamWidthCenter;
            int yDist = OpenCVGreatestColorTest.centerY - webcamHeightCenter;

            //Change x servo
            if(xDist < -xThreshhold){
                xServo.setPosition(xServo.getPosition()+servoMoveSpeedX);
            }
            else if(xDist > xThreshhold){
                xServo.setPosition(xServo.getPosition()-servoMoveSpeedX);
            }

            //Change y servo
            if(yDist < -yThreshhold){
                yServo.setPosition(yServo.getPosition()+servoMoveSpeedY);
            }
            else if(yDist > yThreshhold){
                yServo.setPosition(yServo.getPosition()-servoMoveSpeedY);
            }
            double theta = Math.toDegrees(Math.atan2(xDist, yDist));

            if(gamepad1.a){
                throwDisk(throwingPower);
            }
            if(gamepad1.b){
                stopMotor();
            }
            telemetry.addData("xDist = ", xDist);
            telemetry.addData("yDist = ", yDist);
            telemetry.addData("xServoPos = ", xServo.getPosition());
            telemetry.addData("yServoPos = ", yServo.getPosition());
            telemetry.addData("theta = ", theta);
            telemetry.update();
        }
    }
    public void throwDisk(double throwPower){
        leftMotor.setPower(throwPower);
        rightMotor.setPower(throwPower);
    }
    public void stopMotor(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
