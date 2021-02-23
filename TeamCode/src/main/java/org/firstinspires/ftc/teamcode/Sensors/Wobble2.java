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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name = "Wobble2", group = "Sensor")
//@Disabled
public class Wobble2 extends LinearOpMode {

    ProgrammingFrame robot   = new ProgrammingFrame();

    public boolean wobbleFind(int degrees, double power, double difference, LinearOpMode linearOpMode) {
        double distance = 100;
        // conversion for degrees to ticks
        final double conversion_factor = 12.73;

        int wStart=0, wEnd=0;

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
     //   robot.systemTools.telemetry.addData("Status", "Resetting Encoders");
     //   robot.systemTools.telemetry.update();

        robot.resetDriveEncoders();
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
     //   robot.systemTools.telemetry.addData("Path0", "Starting at %7d :%7d",
      //          robot.frontLeftMotor.getCurrentPosition(),
     //           robot.frontRightMotor.getCurrentPosition(), robot.backLeftMotor.getCurrentPosition(), robot.backRightMotor.getCurrentPosition());
     //   robot.systemTools.telemetry.update();

        int FLstart = robot.frontLeftMotor.getCurrentPosition();

        // set target position for all the motor encoders
        int FLtarget = robot.frontLeftMotor.getCurrentPosition() + TICKS;
        int FRtarget = robot.frontRightMotor.getCurrentPosition() - TICKS;
        int BLtarget = robot.backLeftMotor.getCurrentPosition() + TICKS;
        int BRtarget = robot.backRightMotor.getCurrentPosition() - TICKS;

        robot.frontLeftMotor.setTargetPosition(FLtarget);
        robot.frontRightMotor.setTargetPosition(FRtarget);
        robot.backLeftMotor.setTargetPosition(BLtarget);
        robot.backRightMotor.setTargetPosition(BRtarget);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftMotor.setPower(power);
        robot.frontRightMotor.setPower(-power);
        robot.backRightMotor.setPower(-power);
        robot.backLeftMotor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (linearOpMode.opModeIsActive() &&
                (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() && robot.backLeftMotor.isBusy() && robot.backRightMotor.isBusy())) {
            boolean frontEdgeFound = false;
            distance = robot.wobbleSensor.getDistance(DistanceUnit.CM);
       //     robot.systemTools.telemetry.addData("Distance= ", "%.3f%n", distance);
       //     telemetry.update();
            // reset the timeout time and start motion.
           /* robot.frontLeftMotor.setPower(power);
            robot.frontRightMotor.setPower(-power);
            robot.backRightMotor.setPower(-power);
            robot.backLeftMotor.setPower(power);
          */
            //int sum=0;
                if (distance < difference) {//  wobble found

                    wStart = robot.frontLeftMotor.getCurrentPosition() ;
                    frontEdgeFound = true;
                    // for ( int i = 1; i < 10000; ++i) {sum += i;}   // sum = sum + i.  waste some time to allow robot to turn more and align gripper with wobble.
                    //this saves over 0.5 seconds over adding another 3 degree turn
                }

            if (frontEdgeFound && distance > difference) {
                robot.stopDriveMotors();               //   break;
            }
        }

     //   robot.startDriveEncoders();
        wEnd = robot.frontLeftMotor.getCurrentPosition();

        int FLdelta2 = Math.abs(wEnd - wStart);
        telemetry.addData("wStart: ", wStart + "  wEnd: "+ wEnd + "  FLdelta2: " + FLdelta2);
        int rotateBackDeg2 = (int) (FLdelta2 / conversion_factor)+8;
        telemetry.addData("rotateBackDeg2: ", rotateBackDeg2);
        robot.RotateDEG(-rotateBackDeg2,0.5,this);
        distance = robot.wobbleSensor.getDistance(DistanceUnit.CM);
        return true;
    }

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)robot.wobbleSensor;

        waitForStart();

        while(opModeIsActive()) {
            double startTime = getRuntime();
        //    robot.RotateDEG(146, 0.7, this);        //this is video demo from target zone A
        //    robot.goDistanceAcceleration(103, 0.9, false, 5, 70, this);  //this is video demo from target zone A
            boolean wobbleStat =  wobbleFind(45,0.2,35,this);
                 if (wobbleStat) {
                  //   robot.RotateDEG(3, .2, this);
                     //robot.wait(500,this);
                     double startWobbleDist=robot.wobbleSensor.getDistance(DistanceUnit.CM);
                     int travelDist=8;
                    if (startWobbleDist > 22.5 ||startWobbleDist < 21.5 ){
                             travelDist = (int)(9 + (startWobbleDist-22.5));
                        }
                    if (travelDist >25) {
                        telemetry.addData("Error, travel distance exceeded. Travel distance:   ", travelDist);
                        telemetry.addData("Elapsed Time: ", getRuntime()-startTime);
                        telemetry.update();
                        break;
                    }
                    robot.gripperOpen();
                  robot.raiseGripper(750);
                   robot.GoDistanceCM2(travelDist, .2, false, this);
                   telemetry.addData("Elapsed Time: ", getRuntime()-startTime);
                    robot.wait(2000,this);
                    robot.lowerGripper(900);
                    robot.gripperClose();
                    //robot.wait(300,this);
                   //     robot.raiseGripper(400);
                  //   robot.gripperClose();
                     telemetry.addData("start wobble distance ", startWobbleDist);

                     telemetry.addData("travel distance ", travelDist);
                     telemetry.update();
                   break;
                 }  // end if
        } // end of While Opmodeisactive
        // telemetry.addData("Stopped at detect distance ", String.format("%.01f cm", wobbleDist));

        while(opModeIsActive()) {}
    }

}// end Wobble 2