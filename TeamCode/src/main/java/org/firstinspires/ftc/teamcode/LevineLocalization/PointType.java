package org.firstinspires.ftc.teamcode.LevineLocalization;

public class PointType {
    public String type;
    public double followRadius;

    public PointType(String type){
        type = type.toLowerCase();
        this.type = type;
        switch (type) {
            case "mid":
                followRadius = LevineLocalizationMap.followRadius;
                break;
            case "inside":
                followRadius = LevineLocalizationMap.poseError*20;
                break;
            case "end":
                followRadius = LevineLocalizationMap.poseError*5;
                break;
            case "almostdone":
                followRadius = LevineLocalizationMap.poseError*10;
                break;
            default:
                followRadius = LevineLocalizationMap.poseError;
                break;
        }
    }
    public double getFollowRadius(){
        return followRadius;
    }
    public String toString(){
        return "Type: " + type + ", Follow Radius: " + followRadius;
    }
}
