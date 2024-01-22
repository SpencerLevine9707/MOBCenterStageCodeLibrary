package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BlueBeforeTrussPoses {
    //Starting pos
    public static double xPosStartingPos = -36, yPosStartingPos = -58;

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 36, yPosPurplePixelPlacementAfterBeacon1 = -28;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 36, yPosPurplePixelPlacementAfterBeacon23 = -28;
    public static double xPosStartExtendFirstPlacementAfter = 41, yPosStartExtendFirstPlacementAfter = -28;
    public static double xPosFirstPlacementAfter = 41, yPosFirstPlacementAfter = -31;

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
    public static double xPosGoStraightThroughTrussClose = -40, yPosGoStraightThroughTrussClose = -49;
    public static double xPosMidStackPickUpFar = -43, yPosMidStackPickUpFar = -7.5;
    public static double xPosMidStackPickUpClose = -48, yPosMidStackPickUpClose = -41;

    public static double headingMidStackPickUpFar = Math.toRadians(-30), headingMidStackPickUpClose = Math.toRadians(45);
    public static double headingStartingPositionAndBeacon = -Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(30), headingPlaceClose = Math.toRadians(-30);
    public static double headingWallBeaconBefore = Math.toRadians(55), headingTrussBeaconBefore = Math.toRadians(125), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(100), headingBeacon1PlacementAfter = Math.toRadians(-15), headingBeacon3PlacementAfter = Math.toRadians(15);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(60), headingBeacon2PlacementBeforeClose = Math.toRadians(40), headingBeacon3PlacementBeforeClose = Math.toRadians(20);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(-20), headingBeacon2PlacementBeforeFar = Math.toRadians(-40), headingBeacon3PlacementBeforeFar = Math.toRadians(-60);
}
