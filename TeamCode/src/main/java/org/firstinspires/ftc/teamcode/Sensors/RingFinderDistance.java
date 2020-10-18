package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

public class RingFinderDistance extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();

    public void runOpMode() {


        int RingsFound = 0;

        final float[] rgbValues = new float[3];


        float gain = 2;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gain", gain);

            robot.colorSensor1.setGain(gain);
            robot.colorSensor2.setGain(gain);

            if ((((DistanceSensor) robot.colorSensor1).getDistance(DistanceUnit.CM))>0) {
                RingsFound = 1;
            } else {

                NormalizedRGBA colors = robot.colorSensor1.getNormalizedColors();

                rgbValues[0] = colors.red;
                rgbValues[1] = colors.green;
                rgbValues[2] = colors.blue;


                if (colors.red == 0 && colors.green == 0 && colors.blue == 0) {
                    RingsFound = 1;
                }
            }

            telemetry.addData("Rings Found:", RingsFound);
            telemetry.update();
        }
    }
}
