/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ItsComplicated;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name="MecanumDriveIntake", group="Iterative Opmode")
//@Disabled
public class MecanumDriveIntake extends OpMode
{

    ProgrammingFrame robot   = new ProgrammingFrame();



    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    double strafingConstant = 1.5;
    boolean isIntakeOn = false;
    boolean isAPressed = false;
    boolean gripperClosed = false;
    boolean motorOff = false;
    boolean motorOn = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap, this);
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // controller variables
        double y = -gamepad1.left_stick_y; // y: inverse of left stick's y value
        double x = gamepad1.left_stick_x * strafingConstant; // x underscored: left stick's x value multiplied by the strafing coefficient in order to counteract imperfect strafing
        double rx = gamepad1.right_stick_x; // rx: right stick's x value

        if (Math.abs(x) <= .2) x = 0;

        // for the programming frame
        // frontLeftPower = y + x + rx;
        // frontRightPower = y - x - rx;
        // backLeftPower = -y - x + rx;
        // backRightPower = -y + x - rx;

        // for actual robot
        frontLeftPower = y + x + rx;
        frontRightPower = y - x - rx;
        backLeftPower = y - x + rx;
        backRightPower = y + x - rx;

        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower   = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower   = Range.clip(backRightPower, -1.0, 1.0);

        robot.frontLeftMotor.setPower(frontLeftPower);
        robot.frontRightMotor.setPower(frontRightPower);
        robot.backLeftMotor.setPower(backLeftPower);
        robot.backRightMotor.setPower(backRightPower);

        if (gamepad1.right_bumper) {
            robot.shooting.setPower(0);
            if (!motorOff) {
                motorOff = !motorOn;
                motorOn = true;
                robot.shooting.setPower(1);
            }
            else {
                motorOff = true;
            }

        }

        if (gamepad1.left_bumper) {
            robot.shooting.setPower(1);
            if (!motorOn) {
                motorOn = !motorOff;
                motorOff = true;
                robot.shooting.setPower(0);
            }
            else {
                motorOff = false;
            }

        }

        if (gamepad1.a) {
            if (!isAPressed) {
                isIntakeOn = !isIntakeOn;
                isAPressed = true;
            }
        } else {
            isAPressed = false;
        }

        if(isIntakeOn) {
            robot.intake.setPower(1);
        } else if (gamepad1.b) {
            robot.intake.setPower(-1);
        } else {
            robot.intake.setPower(0);
        }

        if (gamepad1.x && !gripperClosed) {
            robot.moveGripper(true);
            gripperClosed = true;
        }
        if (gamepad1.y && gripperClosed) {
            robot.moveGripper(false);
            gripperClosed = false;
        }

       // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Strafing constant", "Strafing Constant = " + strafingConstant);
        telemetry.addData("Motors", "front_left (%.2f), front_right (%.2f), back_left (%.2f), back_right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void stop() {
        robot.stopAllMotors();
    }

}
