package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class BlueAfterTrussPoses {
    //Starting pos
    public static double xPosStartingPos = 18.5, yPosStartingPos = 58;
    public static double xPosPickUpStackAnchorPoint = -38.968203472377176, yPosPickUpStackAnchorPoint = 5.396277482900213;
    public static int  slidesPosPickUpStackAnchorPoint = -617;
    //Supposed to be 5.5, -35

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 40, yPosPurplePixelPlacementAfterBeacon1 = 25;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 31, yPosPurplePixelPlacementAfterBeacon23 = 25;
    public static double xPosStartExtendFirstPlacementAfter = 45, yPosStartExtendFirstPlacementAfter = 29.5;
    public static double xPosFirstPlacementAfter = 50.75, yPosFirstPlacementAfterBeacon1 = 36, yPosFirstPlacementAfterBeacon2 = 29.5, yPosFirstPlacementAfterBeacon3 = 22;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -36, yPosGoAcrossForBeforeTrussPurplePixelFar = -5;
    public static double xPosGoAroundPurplePixelBeacon2 = -38, yPosGoAroundPurplePixelBeacon2 = -15;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -56;
    public static double xPosAfterPlaceFar = 39, yPosAfterPlaceFar = 30;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = 8;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = 5;//This one
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = 8;
    public static double xPosPickUpPixelFar = -31, yPosPickUpPixelFar = 1;
//    public static double xPosPickUpPixelFar = -34, yPosPickUpPixelFar = 5.5;
    public static double xPosPlacePixelFar = 47, yPosPlacePixelFar = 30;
//    public static double xPosPlacePixelFar = 46.5, yPosPlacePixelFar = 27.2;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 35, yPosStartArmExtendPlaceFar = 5;//This one

    //Close poses
    public static double xPosLineUpForPickUpClose = 13, yPosLineUpForPickUpClose = 54.5;
    public static double xPosStartArmExtendPickUpClose = 0, yPosStartArmExtendPickUpClose = -52;
    public static double xPosPickUpPixelClose = -43, yPosPickUpPixelClose = 44;
    public static double xPosStartArmExtendPlaceClose = 30, yPosStartArmExtendPlaceClose = 54.5;
    public static double xPosPlacePixelClose = 47, yPosPlacePixelClose = 42;
    public static double xPosFlipAfterPlaceClose = 20, yPosFlipAfterPlaceClose = -42;
    public static double xPosGoStraightThroughTrussClose = -40, yPosGoStraightThroughTrussClose = 54.5;
    public static double xPosMidStackPickUpFar = -40, yPosMidStackPickUpFar = 13;
    public static double xPosMidStackPickUpClose = -50, yPosMidStackPickUpClose = 50;

    //Park Poses
    public static double xPosParkTriangle = 57, yPosParkTriangle = 7;
    public static double xPosLineUpParkTriangle = 40, yPosLineUpParkTriangle = 7;
    public static double xPosParkSquare = 57, yPosParkSquare = 55;
    public static double xPosLineUpParkSquare = 40, yPosLineUpParkSquare = 55;

    public static double headingMidStackPickUpFar = Math.toRadians(150), headingMidStackPickUpClose = Math.toRadians(230);
    public static double headingStartingPositionAndBeacon = -Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(0), headingPlaceClose = Math.toRadians(160);
    public static double headingWallBeaconBefore = Math.toRadians(55), headingTrussBeaconBefore = Math.toRadians(125), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(100), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(60), headingBeacon2PlacementBeforeClose = Math.toRadians(40), headingBeacon3PlacementBeforeClose = Math.toRadians(20);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(-20), headingBeacon2PlacementBeforeFar = Math.toRadians(-40), headingBeacon3PlacementBeforeFar = Math.toRadians(-60);
}
