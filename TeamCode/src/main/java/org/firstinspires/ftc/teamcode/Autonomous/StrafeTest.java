package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="StrafeTest", group="Motion")

public class StrafeTest extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 31.3;
    private ElapsedTime runtime = new ElapsedTime();

    public void StrafeDistanceCM(int centimeters, double power){

        boolean left = centimeters < 0;
        int TICKS = (int) Math.abs(Math.round(centimeters * conversion_factor));
        int FLtarget = 0;
        int FRtarget = 0;
        int BLtarget = 0;
        int BRtarget = 0;

        power = Math.abs(power);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.resetDriveEncoders();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)

        if (left) {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() - TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() + TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;
        } else {
            FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
            FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
            BLtarget = robot.backLeftMotor.getCurrentPosition() - TICKS;
            BRtarget = robot.backRightMotor.getCurrentPosition() + TICKS;
        }
        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();

        if (left) {
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
        } else {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(power);
            robot.backLeftMotor.setPower(-power);
        }
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {
        }

        robot.stopDriveMotors();

        robot.startDriveEncoders();

        telemetry.addData("Path", "Complete");
        telemetry.addData("counts", TICKS);
        telemetry.update();
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        StrafeDistanceCM(-158, 0.5);

    }
}
