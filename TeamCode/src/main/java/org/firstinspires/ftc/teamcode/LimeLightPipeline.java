package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimeLightPipeline {

    private static Limelight3A litty = null;

    public static void makeLight(HardwareMap hw, String name) {
        litty = hw.get(Limelight3A.class, name);
        litty.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        litty.pipelineSwitch(2);
        litty.start();
    }

    public static double Align(int pipeline){
        double Angleish = -999;
        litty.pipelineSwitch(pipeline);
        LLResult seeingStuff = litty.getLatestResult();
        LLResultTypes.ColorResult colorResult = null;
        if (seeingStuff != null && seeingStuff.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();
            if (colorResults != null && !colorResults.isEmpty()) {
                colorResult = colorResults.get(0);
            }
        }
        if (colorResult != null) {
            List<List<Double>> targetCorners = colorResult.getTargetCorners();
            if (!targetCorners.isEmpty() && targetCorners.size() > 3) {
                List<Double> c1 = targetCorners.get(0);
                List<Double> c2 = targetCorners.get(1);
                List<Double> c3 = targetCorners.get(2);
                List<Double> c4 = targetCorners.get(3);

                double c1c2XOffset = c1.get(0) - c2.get(0);
                double c1c2YOffset = c1.get(1) - c2.get(1);
                double c3c2XOffset = c3.get(0) - c2.get(0);
                double c3c2YOffset = c3.get(1) - c2.get(1);
                double c4c3XOffset = c4.get(0) - c3.get(0);
                double c3c4YOffset = c3.get(1) - c4.get(1);
                double c1c2D = Math.sqrt(Math.pow(c1c2XOffset, 2) + Math.pow(c1c2YOffset, 2));
                double c2c3D = Math.sqrt(Math.pow(c3c2XOffset, 2) + Math.pow(c3c2YOffset, 2));
                boolean rightSideUp = true;

                if (c1c2D > c2c3D) {
                    rightSideUp = true;
                } else if (c1c2D <= c2c3D) {
                    rightSideUp = false;
                }
                if (rightSideUp) {
                    Angleish = Math.toDegrees(Math.atan2(c3c4YOffset, c4c3XOffset));
                }
                if (!rightSideUp) {
                    Angleish = Math.toDegrees(-Math.atan2(c3c2YOffset, c3c2XOffset));
                }
                Angleish += 90;
                if (Angleish > 90) {
                    Angleish = -180 + Angleish;
                } else if (Angleish < -90) {
                    Angleish = 180 + Angleish;
                }
            }
        }
        return Angleish;
    }
    public static double UpNDown(int pipeline){
        double UpNDown = -999;
        litty.pipelineSwitch(pipeline);
        LLResult seeingStuff = litty.getLatestResult();
        LLResultTypes.ColorResult colorResult = null;
        if (seeingStuff != null && seeingStuff.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();
            if (colorResults != null && !colorResults.isEmpty()) {
                colorResult = colorResults.get(0);
            }
        }
        if (colorResult != null) {
            UpNDown = colorResult.getTargetYDegrees();
        }
        return UpNDown;
    }
}
