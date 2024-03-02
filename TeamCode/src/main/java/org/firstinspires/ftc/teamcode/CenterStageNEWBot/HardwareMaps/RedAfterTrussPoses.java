package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RedAfterTrussPoses {
    //Starting pos
    public static double xPosStartingPos = 18.5, yPosStartingPos = -60;

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 38, yPosPurplePixelPlacementAfterBeacon1 = -30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 30, yPosPurplePixelPlacementAfterBeacon23 = -26;
    public static double xPosStartExtendFirstPlacementAfter = 45, yPosStartExtendFirstPlacementAfter = -32;
    public static double xPosFirstPlacementAfter = 48.5, yPosFirstPlacementAfterBeacon1 = -24, yPosFirstPlacementAfterBeacon2 = -32, yPosFirstPlacementAfterBeacon3 = -38;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -36, yPosGoAcrossForBeforeTrussPurplePixelFar = -5;
    public static double xPosGoAroundPurplePixelBeacon2 = -38, yPosGoAroundPurplePixelBeacon2 = -12;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -56;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = -10;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = -8;//This one
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = -10;
    public static double xPosPickUpPixelFar = -37, yPosPickUpPixelFar = -8;
    public static double xPosPlacePixelFar = 44.5, yPosPlacePixelFar = -30;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -10;
    public static double xPosStartArmExtendPlaceFar = 27, yPosStartArmExtendPlaceFar = -8;//This one

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

    //Park Poses
    public static double xPosParkTriangle = 57, yPosParkTriangle = -10;
    public static double xPosLineUpParkTriangle = 40, yPosLineUpParkTriangle = -10;
    public static double xPosParkSquare = 57, yPosParkSquare = -55;
    public static double xPosLineUpParkSquare = 40, yPosLineUpParkSquare = -55;

    public static double headingMidStackPickUpFar = Math.toRadians(210), headingMidStackPickUpClose = Math.toRadians(130);
    public static double headingStartingPositionAndBeacon = Math.toRadians(90), headingPickUpClose = Math.toRadians(145);
    public static double headingPlaceFar = Math.toRadians(0), headingPlaceClose = Math.toRadians(200);
    public static double headingWallBeaconBefore = Math.toRadians(55), headingTrussBeaconBefore = Math.toRadians(125), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(110), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(60), headingBeacon2PlacementBeforeClose = Math.toRadians(40), headingBeacon3PlacementBeforeClose = Math.toRadians(20);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(-20), headingBeacon2PlacementBeforeFar = Math.toRadians(-40), headingBeacon3PlacementBeforeFar = Math.toRadians(-60);
}
