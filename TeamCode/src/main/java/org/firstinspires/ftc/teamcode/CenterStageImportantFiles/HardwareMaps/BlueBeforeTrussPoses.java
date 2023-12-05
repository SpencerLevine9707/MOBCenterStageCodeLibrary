package org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BlueBeforeTrussPoses {
    //X Poses
    public static double xPosStartingPosition = -35;
    public static double xPosBeacon1Preload = -26, xPosBeacon2Preload = -36, xPosBeacon3Preload = -45, xPosBeacon1LineUpBeforeTruss= -37, xPosBeacon1KnockingLineUpBeforeTruss = -51, xPosBeacon3LineUpAfterTruss = 13;
    public static double xPosPickUpSpot = -55, xPosStackKnockerPos = -50, xPosPickUpPosAfterKnocked = -55, xPosBeforePickUpAfterKnocked = -45;
    public static double xPosPlacement = 51, xPosUnderTruss = 20, xPosSlidesDownAfterPlace = 30, xPosUnderTrussGoingBack = 0, xPosAfterPlacePosForNoCrash = 42;
    public static double xPosLineUpForTruss = -42, xPosAfterPickUpNoPixelCrash = -40, xPosLineUpPlacement = 45, xPosLineUpForFirstPlacementAfterTruss = 20, xPosPutSlidesBackDownBeforePlace = 46;
    public static double xPosPlacementBeacon1 = 51, xPosPlacementBeacon2 = 51, xPosPlacementBeacon3 = 51;

    //Y Poses
    public static double yPosStartingPosition = 63;
    public static double yPosBeacon1Preload = 36, yPosBeacon2Preload = 36, yPosBeacon3Preload = 36, yPosBeacon1LineUpBeforeTruss = 36, yPosBeacon1KnockingLineUpBeforeTruss = 36, yPosBeacon3LineUpAfterTruss = 36;
    public static double yPosPickUpSpot = 35, yPosStackKnockerPos = 35, yPosBeforePickUpAfterKnocked = 42;
    public static double yPosPlacement = 50, yPosUnderTruss = 62, yPosSlidesDownAfterPlace = 62, yPosUnderTrussGoingBack = 62, yPosAfterPlacePosForNoCrash = 62;
    public static double yPosLineUpForTruss = 62, yPosAfterPickUpNoPixelCrash = 48, yPosLineUpPlacement = 36, yPosLineUpForFirstPlacementAfterTruss = 20, yPosPutSlidesBackDownBeforePlace = 36;
    public static double yPosPlacementBeacon1 = 42, yPosPlacementBeacon2 = 33, yPosPlacementBeacon3 = 45;
    //Headings

    public static double headingStartingPositionAndBeacon = Math.toRadians(90);
}
