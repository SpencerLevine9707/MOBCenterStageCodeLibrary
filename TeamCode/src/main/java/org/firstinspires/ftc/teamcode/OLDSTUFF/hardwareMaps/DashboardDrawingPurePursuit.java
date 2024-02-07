package org.firstinspires.ftc.teamcode.OLDSTUFF.hardwareMaps;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.path.Path;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import java.util.List;

public class DashboardDrawingPurePursuit {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 8; // in


//    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
//        double[] xPoints = new double[poseHistory.size()];
//        double[] yPoints = new double[poseHistory.size()];
//        for (int i = 0; i < poseHistory.size(); i++) {
//            Pose2d pose = poseHistory.get(i);
//            xPoints[i] = pose.getX();
//            yPoints[i] = pose.getY();
//        }
//        canvas.strokePolyline(xPoints, yPoints);
//    }
//
//    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
//        int samples = (int) Math.ceil(path.length() / resolution);
//        double[] xPoints = new double[samples];
//        double[] yPoints = new double[samples];
//        double dx = path.length() / (samples - 1);
//        for (int i = 0; i < samples; i++) {
//            double displacement = i * dx;
//            Pose2d pose = path.get(displacement);
//            xPoints[i] = pose.getX();
//            yPoints[i] = pose.getY();
//        }
//        canvas.strokePolyline(xPoints, yPoints);
//    }
//
//    public static void drawSampledPath(Canvas canvas, Path path) {
//        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
//    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX()*39.37, pose.getY()*39.37, ROBOT_RADIUS);
        double h = pose.getHeading();
        double x1 = pose.getX(), y1 = pose.getY();
        double x2 = pose.getX() + (Math.cos(h)*ROBOT_RADIUS), y2 = pose.getY() + (Math.sin(h)*ROBOT_RADIUS);
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
