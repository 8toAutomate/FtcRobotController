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
        String topRingConnect, botRingConnect, topRingName, botRingName, leftLineConnect, rightLineConnect, leftLineName, rightLineName;

       // final float[] rgbValues = new float[3];

        double maxRingDistCM = 20;

        double bottomRingValueCM;
        double topRingValueCM;
        double leftLineValueCM;
        double rightLineValueCM;

        boolean botRingDetected;
        boolean topRingDetected;

       // float gain = 1;

        waitForStart();
        while (opModeIsActive()) {
        //    telemetry.addData("Gain", gain);

       //     robot.bottomRing.setGain(gain);
      //      robot.topRing.setGain(gain);
            //bottomRingValueCM = robot.bottomRing.getDistance(DistanceUnit.CM);
            bottomRingValueCM = ((DistanceSensor) robot.bottomRing).getDistance(DistanceUnit.CM);
            botRingName = robot.bottomRing.getDeviceName();
            botRingConnect =  robot.bottomRing.getConnectionInfo();

            // topRingValueCM = robot.topRing.getDistance(DistanceUnit.CM);
            topRingValueCM = ((DistanceSensor) robot.topRing).getDistance(DistanceUnit.CM);
            topRingName = robot.topRing.getDeviceName();
            topRingConnect = robot.topRing.getConnectionInfo();
/*
            leftLineValueCM = ((DistanceSensor)robot.colorSensor1).getDistance(DistanceUnit.CM);
            leftLineName = robot.colorSensor1.getDeviceName();
            leftLineConnect = robot.colorSensor1.getConnectionInfo();
            rightLineValueCM = ((DistanceSensor)robot.colorSensor2).getDistance(DistanceUnit.CM);
            rightLineName = robot.colorSensor2.getDeviceName();
            rightLineConnect = robot.colorSensor2.getConnectionInfo();
*/
            botRingDetected = bottomRingValueCM < maxRingDistCM;
            topRingDetected = topRingValueCM < maxRingDistCM;

            // Wait and check again to keep more consistent, if either check gave a true sensor value, then the value is true, otherwise false.
            sleep(100);

            topRingValueCM = robot.topRing.getDistance(DistanceUnit.CM);
            bottomRingValueCM = robot.bottomRing.getDistance(DistanceUnit.CM);

            botRingDetected = bottomRingValueCM < maxRingDistCM || botRingDetected;
            topRingDetected = topRingValueCM < maxRingDistCM || topRingDetected;

            if (botRingDetected && topRingDetected) {
                ringsFound = "4 - C Path";
            } else if (botRingDetected&& !topRingDetected) {
                ringsFound = "1 - B Path";
            } else if (!botRingDetected && !topRingDetected) {
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

            telemetry.addData("Bot. sensor config: ", botRingName + "  Conn.: " + botRingConnect + "  Distance (CM): " + "%.3f%n",bottomRingValueCM);
            telemetry.addData("Top sensor config: ", topRingName + "  Conn.: " + topRingConnect + "  Distance (CM): " + "%.3f%n", topRingValueCM );
           // telemetry.addData("Left sensor config: ", leftLineName + "  Conn.: " + leftLineConnect + "  Distance (CM): " + "%.3f%n",leftLineValueCM);
           // telemetry.addData("Right sensor config: ", rightLineName + "  Conn.: " + rightLineConnect + "  Distance (CM): " + "%.3f%n", rightLineValueCM );
            telemetry.addData("Maximum Ring Distance (CM): ", maxRingDistCM);
            telemetry.addData("Rings Found:", ringsFound);
            telemetry.update();
        }
    }
}
