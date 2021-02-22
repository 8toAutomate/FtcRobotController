package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="WobbleFinderDistance test", group="Motion")
public class WobbleFinderDistance_test extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();

    public void runOpMode() {

        robot.init(hardwareMap, this);

       // final float[] rgbValues = new float[3];

        double wobbleDistValueCM;

        boolean wobbleDetected;

       // float gain = 1;

        waitForStart();
        while (opModeIsActive()) {
        //    telemetry.addData("Gain", gain);

       //     robot.bottomRing.setGain(gain);
      //      robot.topRing.setGain(gain);
            //bottomRingValueCM = robot.bottomRing.getDistance(DistanceUnit.CM);
            wobbleDistValueCM = ((DistanceSensor) robot.wobbleSensor).getDistance(DistanceUnit.CM);

            wobbleDetected = wobbleDistValueCM < 40;

            telemetry.addData("Wobble Distance: ", "%.3f%n",wobbleDistValueCM);

            telemetry.addData("Wobble Found:", wobbleDetected);
            telemetry.update();
        }
    }
}
