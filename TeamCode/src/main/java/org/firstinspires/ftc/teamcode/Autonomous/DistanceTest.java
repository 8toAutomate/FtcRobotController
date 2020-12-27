package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;
@Autonomous(name="DistanceTest", group="ProgrammingFrame")

    public class DistanceTest extends LinearOpMode {
        /* Updated telemetry statements so all lines are displayed on screen  8Toautomate 12-26-20
         */
        /* Declare OpMode members. */
        ProgrammingFrame robot   = new ProgrammingFrame();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();

        //static final int        TICKS                   = 1000;
        static final double     DRIVE_SPEED             = 0.5;
        static final double     TURN_SPEED              = 0.5;
        static final double     timeoutS                = 20;

        public void GoDistanceTICKS(int ticks, double power) {

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");
            //telemetry.update();

            robot.resetDriveEncoders();

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addLine().addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.frontLeftMotor.getCurrentPosition(),
                    robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
            //telemetry.update();

            // Wait for the game to start (driver presses PLAY)

            int FLtarget = robot.frontLeftMotor.getCurrentPosition() + ticks;
            int FRtarget = robot.frontRightMotor.getCurrentPosition() + ticks;
            int BLtarget = robot.backLeftMotor.getCurrentPosition() + ticks;
            int BRtarget = robot.backRightMotor.getCurrentPosition() + ticks;

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
            robot.frontLeftMotor.setPower(-power);
            robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 30) &&
                    (Math.abs(robot.frontLeftMotor.getCurrentPosition()) < ticks && Math.abs(robot.frontRightMotor.getCurrentPosition()) < ticks && Math.abs(robot.backLeftMotor.getCurrentPosition()) < ticks && Math.abs(robot.backRightMotor.getCurrentPosition()) < ticks)) {
            }

            robot.stopDriveMotors();

            robot.startDriveEncoders();


            telemetry.addLine().addData("Path", "Complete");
            telemetry.addLine().addData("counts", ticks);
            telemetry.addLine();
            telemetry.addData("Final pos.", "Starting at %7d :%7d :%7d :%7d",
                    robot.frontLeftMotor.getCurrentPosition(),
                    robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
            telemetry.update();
            while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode
        }
//*************************************************************************************************

        public void runOpMode() {
            robot.init(hardwareMap,this);
            waitForStart();
            GoDistanceTICKS(3338, 0.5);
            robot.stopDriveMotors();
        }

}
