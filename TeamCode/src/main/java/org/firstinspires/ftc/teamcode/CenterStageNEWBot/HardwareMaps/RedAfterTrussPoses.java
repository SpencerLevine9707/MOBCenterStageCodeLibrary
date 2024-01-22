package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RedAfterTrussPoses {
    //Starting pos
    public static double xPosStartingPos = 16, yPosStartingPos = -58;

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 30.4, yPosPurplePixelPlacementAfterBeacon1 = -30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 30.4, yPosPurplePixelPlacementAfterBeacon23 = -30;
    public static double xPosStartExtendFirstPlacementAfter = 25, yPosStartExtendFirstPlacementAfter = -34;
    public static double xPosFirstPlacementAfter = 47, yPosFirstPlacementAfter = -30;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -36, yPosGoAcrossForBeforeTrussPurplePixelFar = -5;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -56;

    //Far poses
    public static double xPosLineUpForPickUpFar = 10, yPosLineUpForPickUpFar = -13;
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = -13;
    public static double xPosPickUpPixelFar = -41.5, yPosPickUpPixelFar = -13;
    public static double xPosPlacePixelFar = 47, yPosPlacePixelFar = -21;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 27, yPosStartArmExtendPlaceFar = -13;

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

    public static double headingMidStackPickUpFar = Math.toRadians(210), headingMidStackPickUpClose = Math.toRadians(130);
    public static double headingStartingPositionAndBeacon = Math.toRadians(90), headingPickUpClose = Math.toRadians(145);
    public static double headingPlaceFar = Math.toRadians(160), headingPlaceClose = Math.toRadians(200);
    public static double headingWallBeaconBefore = Math.toRadians(55), headingTrussBeaconBefore = Math.toRadians(125), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(100), headingBeacon1PlacementAfter = Math.toRadians(170), headingBeacon3PlacementAfter = Math.toRadians(190);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(60), headingBeacon2PlacementBeforeClose = Math.toRadians(40), headingBeacon3PlacementBeforeClose = Math.toRadians(20);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(-20), headingBeacon2PlacementBeforeFar = Math.toRadians(-40), headingBeacon3PlacementBeforeFar = Math.toRadians(-60);
}
