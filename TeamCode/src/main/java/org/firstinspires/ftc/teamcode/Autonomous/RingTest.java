package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Disabled
//@Autonomous(name="RingTest", group="Motion")

public class RingTest extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 8.46;
    private ElapsedTime runtime = new ElapsedTime();
    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];
    int hBound1 = 0;
    int hBound2 = 0;


    public void findLine(){

        if (robot.topRingColor instanceof SwitchableLight)
        {
            ((SwitchableLight)robot.topRingColor).enableLight(true);
        }

        // reset the timeout time and start motion.
        runtime.reset();


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

      //  while (opModeIsActive()) {
       //     telemetry.addData("rings detected", robot.ringFinder());
       //     telemetry.update();

     //   }


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
