package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="Ring_Finder", group="Motion")
public class Ring_Finder extends LinearOpMode {

    ProgrammingFrame robot = new ProgrammingFrame();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        String ringsFound;

        boolean sensor1Detected;
        boolean sensor2Detected;


        float gain = 2;


        float redLowerVal = 1;
        float redUpperVal = 1;
        float greenLowerVal = 1;
        float greenUpperVal = 1;
        float blueLowerVal = 1;
        float blueUpperVal = 1;

        waitForStart();
        while (opModeIsActive()) {



            telemetry.addData("Gain", gain);

            robot.colorSensor1.setGain(gain);
            robot.colorSensor2.setGain(gain);


            NormalizedRGBA colors1 = robot.colorSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = robot.colorSensor2.getNormalizedColors();


            sensor1Detected = colors1.red >= redLowerVal && colors1.red <= redUpperVal && colors1.green >= greenLowerVal && colors1.green <= greenUpperVal && colors1.blue >= blueLowerVal && colors1.blue <= blueUpperVal;

            sensor2Detected = colors2.red >= redLowerVal && colors2.red <= redUpperVal && colors2.green >= greenLowerVal && colors2.green <= greenUpperVal && colors2.blue >= blueLowerVal && colors2.blue <= blueUpperVal;

            if (sensor1Detected && sensor2Detected) {
                ringsFound = "2";
            } else if (sensor1Detected && !sensor2Detected) {
                ringsFound = "1";
            } else if (!sensor1Detected && !sensor2Detected) {
                ringsFound = "0";
            } else {
                ringsFound = "error - sensor 2 detects but sensor 1 doesn't";
            }

            telemetry.addData("Rings Found:", ringsFound);
            telemetry.update();

        }

    }
}
