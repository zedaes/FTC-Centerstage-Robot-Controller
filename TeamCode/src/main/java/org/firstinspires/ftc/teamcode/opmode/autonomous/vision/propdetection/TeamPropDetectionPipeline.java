package org.firstinspires.ftc.teamcode.opmode.autonomous.vision.propdetection;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class TeamPropDetectionPipeline extends OpenCvPipeline {

    public Telemetry telemetry;

    List<Integer> teamPropColor = Arrays.asList(0, 0, 0); // white

    public int teamPropZone = 2;

    Mat original;

    Mat zone1;
    Mat zone2;
    Mat zone3;

    Scalar avgColor1;
    Scalar avgColor2;
    Scalar avgColor3;


    static double max_distance = 0;


    @Override
    public Mat processFrame(Mat input) {
        original = input.clone();

        zone1 = input.submat(new Rect(270, 220, 220, 170));
        zone2 = input.submat(new Rect(670, 220, 220, 170));
        zone3 = input.submat(new Rect(1070, 220, 220, 170));

        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);

        double distance1 = getColorDistance(avgColor1, teamPropColor);
        double distance2 = getColorDistance(avgColor2, teamPropColor);
        double distance3 = getColorDistance(avgColor3, teamPropColor);

        max_distance = Math.min(distance3, Math.min(distance1, distance2));

        if (max_distance == distance1){
            telemetry.addLine("left");
            telemetry.update();
            teamPropZone = 1;

        }else if (max_distance == distance2){
            telemetry.addLine("mid");
            telemetry.update();
            teamPropZone = 2;
        } else {
            telemetry.addLine("right");
            telemetry.update();
            teamPropZone = 3;
        }
        return original;


    }

    public double getColorDistance(Scalar color1, List color2){
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        telemetry.addData("red value", r1);
        telemetry.addData("green value", g1);
        telemetry.addData("blue value", b1);
        telemetry.update();

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }


    public static double getMaxDistance(){
        return max_distance;
    }

    public int getTeamPropZone() {
        return teamPropZone;
    }
}