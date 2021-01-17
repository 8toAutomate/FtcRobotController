package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="ArmTest", group="ProgrammingFrame")

    public class ArmTest extends LinearOpMode {
        /* Updated telemetry statements so all lines are displayed on screen  8Toautomate 12-26-20
         */
        /* Declare OpMode members. */
        ProgrammingFrame robot   = new ProgrammingFrame();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();

        //static final int        TICKS                   = 1000;
        static final double     DRIVE_SPEED             = 0.2;
        static final double     TURN_SPEED              = 0.5;
        static final double     timeoutS                = 20;

        public void ArmTICKS(int ticks, double power) {

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");
            //telemetry.update();

            robot.lifting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Wait for the game to start (driver presses PLAY)

            int target = robot.lifting.getCurrentPosition() + ticks;


            robot.lifting.setTargetPosition(target);


            robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);


         // reset the timeout time and start motion.
            runtime.reset();
            robot.lifting.setPower(power);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (robot.lifting.isBusy())) {}

            robot.lifting.setPower(0);

            robot.lifting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            telemetry.addLine().addData("Path", "Complete");
            telemetry.addLine().addData("counts", ticks);
            telemetry.addLine();
            telemetry.update();
            while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode
        }
//*************************************************************************************************

        public void runOpMode() {
            robot.init(hardwareMap,this);
            waitForStart();
            ArmTICKS(-100, 0.1);
            robot.stopDriveMotors();
        }

}
