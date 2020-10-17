package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Ring_Finder {
    public void runOpMode() {

        final float[] hsvValues = new float[3];

        int RingsFound = 0;

        final float[] rgbValues = new float[3];

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        float gain = 2;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gain", gain);

            colorSensor.setGain(gain);

            if ((((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM))>0) {
                RingsFound = 1;
            } else {

                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);

                rgbValues[0] = colors.red;
                rgbValues[1] = colors.green;
                rgbValues[2] = colors.blue;

                telemetry.addLine()
                        .addData("Red", "%.3f", colors.red)
                        .addData("Green", "%.3f", colors.green)
                        .addData("Blue", "%.3f", colors.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsvValues[0])
                        .addData("Saturation", "%.3f", hsvValues[1])
                        .addData("Value", "%.3f", hsvValues[2]);
                telemetry.addData("Alpha", "%.3f", colors.alpha);


                if (colors.red == 0 && colors.green == 0 && colors.blue == 0) {
                    RingsFound = 1;
                }
            }

            telemetry.addData("Rings Found:", RingsFound);
            telemetry.update();
        }
    }
}
