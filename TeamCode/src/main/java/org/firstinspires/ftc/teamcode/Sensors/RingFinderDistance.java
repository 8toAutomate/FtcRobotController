package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="RingFinderDistance", group="Motion")
public class RingFinderDistance extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        String ringsFound;

        final float[] rgbValues = new float[3];

        double maxRingDistCM = 2.75;

        double sensor1ValueCM;
        double sensor2ValueCM;

        boolean sensor1Detected;
        boolean sensor2Detected;


        float gain = 1;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gain", gain);

            robot.bottomRing.setGain(gain);
            robot.topRing.setGain(gain);

            sensor1ValueCM = ((DistanceSensor) robot.bottomRing).getDistance(DistanceUnit.CM);
            sensor2ValueCM = ((DistanceSensor) robot.topRing).getDistance(DistanceUnit.CM);

            sensor1Detected = sensor1ValueCM < maxRingDistCM;
            sensor2Detected = sensor2ValueCM < maxRingDistCM;

            if (sensor1Detected && sensor2Detected) {
                ringsFound = "4 - C Path";
            } else if (sensor1Detected && !sensor2Detected) {
                ringsFound = "1 - B Path";
            } else if (!sensor1Detected && !sensor2Detected) {
                ringsFound = "0 - A Path";
            } else { // Means there was an error
                ringsFound = "error - sensor 2 detects but sensor 1 doesn't";
            }
            /*else {

                NormalizedRGBA colors = robot.colorSensor1.getNormalizedColors();

                rgbValues[0] = colors.red;
                rgbValues[1] = colors.green;
                rgbValues[2] = colors.blue;


                if (colors.red == 0 && colors.green == 0 && colors.blue == 0) {
                    RingsFound = 1;
                }
            }*/

            telemetry.addData("Sensor 1 Distance (CM): ", sensor1ValueCM);
            telemetry.addData("Sensor 2 Distance (CM): ", sensor2ValueCM);
            telemetry.addData("Maximum Ring Distance (CM): ", maxRingDistCM);
            telemetry.addData("Rings Found:", ringsFound);
            telemetry.update();
        }
    }
}
