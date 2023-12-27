package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BlueBeforeTrussPoses {
    //Starting pos
    public static double xPosStartingPos = -36, yPosStartingPos = -58;

    //After only poses
    public static double xPosPurplePixelPlacementAfter = 36, yPosPurplePixelPlacementAfter = -28;
    public static double xPosStartExtendFirstPlacementAfter = 41, yPosStartExtendFirstPlacementAfter = -28;
    public static double xPosFirstPlacementAfter = 41, yPosFirstPlacementAfter = -28;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -36, yPosGoAcrossForBeforeTrussPurplePixelFar = -5;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -56;

    //Far poses
    public static double xPosLineUpForPickUpFar = 20, yPosLineUpForPickUpFar = -11;
    public static double xPosStartArmExtendPickUpFar = 0, yPosStartArmExtendPickUpFar = -9;
    public static double xPosPickUpPixelFar = -28, yPosPickUpPixelFar = -7.5;
    public static double xPosPlacePixelFar = 43, yPosPlacePixelFar = -19;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 21, yPosStartArmExtendPlaceFar = -13;

    //Close poses
    public static double xPosLineUpForPickUpClose = 10, yPosLineUpForPickUpClose = -54;
    public static double xPosStartArmExtendPickUpClose = 0, yPosStartArmExtendPickUpClose = -52;
    public static double xPosPickUpPixelClose = -45, yPosPickUpPixelClose = -49;
    public static double xPosStartArmExtendPlaceClose = 21, yPosStartArmExtendPlaceClose = -50;
    public static double xPosPlacePixelClose = 43, yPosPlacePixelClose = -43;
    public static double xPosFlipAfterPlaceClose = 20, yPosFlipAfterPlaceClose = -42;


    public static double headingStartingPositionAndBeacon = Math.toRadians(90);
    public static double headingPlaceFar = Math.toRadians(-30), headingPlaceClose = Math.toRadians(30);
}
