/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name = "WobbleFinderDistance", group = "Sensor")
//@Disabled
public class WobbleFinderDistance extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();

    public void findWobble(int degrees, double power, double difference,int repetitions, LinearOpMode linearOpMode) {

        // conversion for degrees to ticks
        final double conversion_factor = 12.73;
        double distance2 = 0.0;

        // if degrees are negative, set the power negative
        if (degrees < 0 && power > 0) {
            power = power * -1;
        }

        int TICKS = (int) Math.round(degrees * conversion_factor);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
       //robot.systemTools.telemetry.addData("Status", "Resetting Encoders");
       //robot.systemTools.telemetry.update();

        robot.resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
       //robot.systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
       //        robot.frontLeftMotor.getCurrentPosition(),
       //        robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
       //robot.systemTools.telemetry.update();

        // set target position for all the motor encoders
        int FLcurrent = robot.frontLeftMotor.getCurrentPosition();
        int FRcurrent = robot.frontRightMotor.getCurrentPosition();
        int BLcurrent = robot.backLeftMotor.getCurrentPosition();
        int BRcurrent = robot.backRightMotor.getCurrentPosition();



        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        for (int i=0; i<repetitions; i++) {

            // set target position for all the motor encoders
            int FLtarget = FLcurrent + TICKS;
            int FRtarget = FRcurrent - TICKS;
            int BLtarget = BLcurrent + TICKS;
            int BRtarget = BRcurrent - TICKS;

            robot.frontLeftMotor.setTargetPosition(FLtarget);
            robot.frontRightMotor.setTargetPosition(FRtarget);
            robot.backLeftMotor.setTargetPosition(BLtarget);
            robot.backRightMotor.setTargetPosition(BRtarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (linearOpMode.opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {

                double distance = robot.wobbleSensor.getDistance(DistanceUnit.CM);
                double initial = linearOpMode.getRuntime();

                if (linearOpMode.getRuntime() - initial < 0.2) {
                    distance2 = robot.wobbleSensor.getDistance(DistanceUnit.CM);
                }
                // reset the timeout time and start motion.
                robot.frontLeftMotor.setPower(power);
                robot.frontRightMotor.setPower(-power);
                robot.backRightMotor.setPower(-power);
                robot.backLeftMotor.setPower(power);

                if (distance2 - distance < difference) {
                    break;
                }
            }

        robot.GoDistanceCM2(5, 0.2, false, this);


        }
        robot.stopDriveMotors();
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//        backLeftMotor.setPower(0);

        robot.startDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.systemTools.telemetry.addData("Path", "Complete");
        //robot.systemTools.telemetry.addData("counts", TICKS);
        //robot.systemTools.telemetry.update();
    }
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.wobbleSensor;

        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {


            findWobble(-50, 0.2, 55, 1, this);

            //telemetry.addData("range", String.format("%.01f cm", robot.wobbleFinder.getDistance(DistanceUnit.CM)));
//
            //telemetry.update();
        }
    }

}