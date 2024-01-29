package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RedBeforeTrussPoses {
    //Starting pos
    public static double xPosStartingPos = -31, yPosStartingPos = -58;

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 30.4, yPosPurplePixelPlacementAfterBeacon1 = -30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 22.4, yPosPurplePixelPlacementAfterBeacon23 = -30;
    public static double xPosStartExtendFirstPlacementAfter = 25, yPosStartExtendFirstPlacementAfter = -34;
    public static double xPosFirstPlacementAfter = 43, yPosFirstPlacementAfter = -28;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -30, yPosGoAcrossForBeforeTrussPurplePixelFar = -10;
    public static double xPosGoAroundPurplePixelBeacon2 = -50, yPosGoAroundPurplePixelBeacon2 = -15;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -43;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = -5;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = -5;
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = -5;
    public static double xPosPickUpPixelFar = -41, yPosPickUpPixelFar = -9;
    public static double xPosPlacePixelFar = 47, yPosPlacePixelFar = -22;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 35, yPosStartArmExtendPlaceFar = -5;

    //Close poses
    public static double xPosLineUpForPickUpClose = 13, yPosLineUpForPickUpClose = -54.5;
    public static double xPosStartArmExtendPickUpClose = 0, yPosStartArmExtendPickUpClose = -52;
    public static double xPosPickUpPixelClose = -43, yPosPickUpPixelClose = -44;
    public static double xPosStartArmExtendPlaceClose = 30, yPosStartArmExtendPlaceClose = -54.5;
    public static double xPosPlacePixelClose = 47, yPosPlacePixelClose = -42;
    public static double xPosFlipAfterPlaceClose = 20, yPosFlipAfterPlaceClose = -42;
    public static double xPosGoStraightThroughTrussClose = -40, yPosGoStraightThroughTrussClose = -54.5;
    public static double xPosMidStackPickUpFar = -40, yPosMidStackPickUpFar = -13;
    public static double xPosMidStackPickUpClose = -50, yPosMidStackPickUpClose = -50;

    public static double headingMidStackPickUpFar = Math.toRadians(150), headingMidStackPickUpClose = Math.toRadians(230);
    public static double headingStartingPositionAndBeacon = Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(160), headingPlaceClose = Math.toRadians(200);
    public static double headingWallBeaconBefore = Math.toRadians(69), headingTrussBeaconBefore = Math.toRadians(112), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(80), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(220), headingBeacon2PlacementBeforeClose = Math.toRadians(200), headingBeacon3PlacementBeforeClose = Math.toRadians(190);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(220), headingBeacon2PlacementBeforeFar = Math.toRadians(200), headingBeacon3PlacementBeforeFar = Math.toRadians(190);
}
