package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="RingTest", group="Motion")

public class RingTest extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 8.46;
    private ElapsedTime runtime = new ElapsedTime();
    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];
    int hBound1 = 0;
    int hBound2 = 0;


    public void findLine(){


        if (robot.topRing instanceof SwitchableLight)
        {
            ((SwitchableLight)robot.topRing).enableLight(true);
        }

        // reset the timeout time and start motion.
        runtime.reset();


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive()) {
            NormalizedRGBA colors1 = robot.topRing.getNormalizedColors();


            Color.colorToHSV(colors1.toColor(), hsvValues);



            telemetry.addData("Hval", hsvValues[0]);
            telemetry.addData("red", colors1.red);
            telemetry.addData("green", colors1.green);
            telemetry.addData("blue", colors1.blue);


            if (robot.topRing instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.topRing).getDistance(DistanceUnit.CM));
            }
            telemetry.update();

        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        findLine();

    }
}
