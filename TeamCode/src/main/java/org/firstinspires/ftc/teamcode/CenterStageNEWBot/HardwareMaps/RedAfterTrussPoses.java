package org.firstinspires.ftc.teamcode.CenterStageNEWBot.HardwareMaps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RedAfterTrussPoses {
    //X Poses
    public static double xPosStartingPosition = 11;
    public static double xPosBeacon1Preload = -2, xPosBeacon2Preload = 9.5, xPosBeacon3Preload = 20, xPosBeacon1LineUpBeforeTruss = -37, xPosBeacon1KnockingLineUpBeforeTruss = -50, xPosBeacon3LineUpAfterTruss = 13;
    public static double xPosPickUpSpot = -64, xPosStackKnockerPos = -56, xPosPickUpPosAfterKnocked = -60, xPosBeforePickUpAfterKnocked = -50;
    public static double xPosPlacement = 47, xPosUnderTruss = 20, xPosSlidesDownAfterPlace = 30, xPosUnderTrussGoingBack = 0, xPosAfterPlacePosForNoCrash = 42;
    public static double xPosLineUpForTruss = -42, xPosAfterPickUpNoPixelCrash = -40, xPosLineUpPlacement = 32, xPosLineUpForFirstPlacementAfterTruss = 25, xPosPutSlidesBackDownBeforePlace = 34;
    public static double xPosPlacementBeacon1 = 47.5, xPosPlacementBeacon2 = 47.5, xPosPlacementBeacon3 = 47.5, xPosLineUpPlacementBeacon2 = 30;

    //Y Poses
    public static double yPosStartingPosition= - 63;
    public static double yPosBeacon1Preload = -38, yPosBeacon2Preload = -37, yPosBeacon3Preload = -45, yPosBeacon1LineUpBeforeTruss = -43, yPosBeacon1KnockingLineUpBeforeTruss = -40, yPosBeacon3LineUpAfterTruss = -38;
    public static double yPosPickUpSpot = -28, yPosStackKnockerPos = -38, yPosBeforePickUpAfterKnocked = -22;
    public static double yPosPlacement = -28, yPosUnderTruss = -53, yPosSlidesDownAfterPlace = -53, yPosUnderTrussGoingBack = -53, yPosAfterPlacePosForNoCrash = -53;
    public static double yPosLineUpForTruss = -53, yPosAfterPickUpNoPixelCrash = -48, yPosLineUpPlacement = -35, yPosLineUpForFirstPlacementAfterTruss = -35, yPosPutSlidesBackDownBeforePlace = -35;
    public static double yPosPlacementBeacon1 = -25, yPosPlacementBeacon2 = -33, yPosPlacementBeacon3 = -36.5, yPosLineUpPlacementBeacon2 = -35;
    //Headings

    public static double headingStartingPositionAndBeacon = -Math.toRadians(90);
}
