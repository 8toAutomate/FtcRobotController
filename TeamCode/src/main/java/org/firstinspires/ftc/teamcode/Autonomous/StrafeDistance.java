package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="StrafeDistance", group="Motion")

public class StrafeDistance extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();
    static final double conversion_factor = 8.46;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean strafingLeft;

    public void Strafe2M(int centimeters, double power){

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.resetEncoders();
//        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.startEncoders();

        double startDistance = robot.getDistanceCM();
        double targetDistance = Math.abs(startDistance - centimeters);
        strafingLeft = targetDistance > startDistance;

        // reset the timeout time and start motion.
        runtime.reset();
        if (!strafingLeft) {
            robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(-power);
            robot.backRightMotor.setPower(power);
        } else {
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.backLeftMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30)) {
            if (strafingLeft && robot.getDistanceCM() > targetDistance) {
                break;
            }
            if (!strafingLeft && robot.getDistanceCM() < targetDistance) {
                break;
            }
        }

        robot.stopDriveMotors();
//        robot.frontLeftMotor.setPower(0);
//        robot.frontRightMotor.setPower(0);
//        robot.backRightMotor.setPower(0);
//        robot.backLeftMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        Strafe2M(100, 0.5);

    }
}
