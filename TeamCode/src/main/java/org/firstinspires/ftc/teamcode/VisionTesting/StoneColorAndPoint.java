package org.firstinspires.ftc.teamcode.VisionTesting;

import org.opencv.core.Point;

public class StoneColorAndPoint {
    public Point stonePoint;
    public String stoneColor;
    public StoneColorAndPoint(Point p, String c){
        stonePoint = p;
        stoneColor = c.toLowerCase();
    }
    public Point getStonePoint(){
        return stonePoint;
    }
    public String getStoneColor(){
        return stoneColor;
    }
    public String toString(){
        return "Stone Point is " + stonePoint + " and Stone Color is " + stoneColor;
    }
}
