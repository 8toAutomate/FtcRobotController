package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="FindLine", group="Motion")
@Disabled
public class FindLine extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 8.46;
    private ElapsedTime runtime = new ElapsedTime();
    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];
    int hBound1 = 0;
    int hBound2 = 0;


    public void findLine(double power){

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.resetDriveEncoders();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
        telemetry.update();


        NormalizedRGBA colors1 = robot.colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = robot.colorSensor2.getNormalizedColors();

        Color.colorToHSV(colors1.toColor(), hsvValues);
        Color.colorToHSV(colors2.toColor(), hsvValues2);


        if (robot.colorSensor1 instanceof SwitchableLight && robot.colorSensor2 instanceof SwitchableLight)
        {
            ((SwitchableLight)robot.colorSensor1).enableLight(true);
            ((SwitchableLight)robot.colorSensor2).enableLight(true);
        }

        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                hsvValues[0] == hBound1 && hsvValues2[0] == hBound2) {
        }

        robot.stopDriveMotors();

        robot.startDriveEncoders();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        findLine(0.5);

    }
}
