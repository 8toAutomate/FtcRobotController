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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name="WobbleControl", group="Sensor")
//@Disabled
public class WobbleControl extends OpMode
{

    ProgrammingFrame robot   = new ProgrammingFrame();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;  //initial time for shotting button timer
    double initialST;  //initial time for storage button timer
    double initialFL;  //initial time for flywheel button timer
    double initialSR;  //initial time for right strafe 20
    //double initialIN;  //initial time for intake button timer
    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    // Setup a variable for strafing constant
    double strafingConstant = 1.5;
    // Setup boolean variables
    boolean isIntakeOn = false;
    boolean isAPressed = false;
    enum States {
        Forwards, Backwards, Off, On
    }
    States autoLifterState = States.Off;
    States ringPusher = States.Backwards;
    States flywheel = States.Off;
    States intakeState = States.Off;
    States intakeButtonState = States.Off;
    boolean gripperClosed = false;
    boolean gripperRaised = false;
    boolean xClick, yClick = false;
    boolean shooting = false;   // Flag  is true when shooting process is in progress
    boolean shootButton = false;   // shoot button status flag -  true  = button was pressed
    boolean shootingReset = true;  // shooting arm return flag - false = shooting arm reset in process
    boolean storageUp = false;
    boolean movingStorage = false;
    boolean storagePressed = false;
    boolean flyWheel, flyMotor, flyWheel2 = false;
    boolean strafe20,rtClick = false;
    boolean shootingReverse = false;
    boolean gripperMoving = false;
    boolean gripperPressed = false;
    double liftingPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void moveGripper(boolean close) {
        robot.gripperServo.setDirection(Servo.Direction.REVERSE);
        robot.gripperServo.scaleRange(0, 0.93);
        if (close) {
            robot.gripperServo.setPosition(1);
        }
        else {
            robot.gripperServo.setPosition(0);
        }
    }

    // exponential function for joystick inputs
    public double exponential(double value, int constant) {
        double cubed =  value*value*value;
        return cubed * constant;
    }

    public void raiseGripper() {
        autoLifterState = States.Forwards;
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() - 850);
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setPower(-0.8);
        // while (robot.lifting.isBusy() && robot.highSwitch1.isPressed() == false && robot.highSwitch2.isPressed() == false) {}
        // robot.lifting.setPower(0);
        // robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lowerGripper() {
        autoLifterState = States.Backwards;
        robot.lifting.setTargetPosition(robot.lifting.getCurrentPosition() + 850);
        robot.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lifting.setPower(.5);
        // while (robot.lifting.isBusy() && robot.lowSwitch1.isPressed() == false && robot.lowSwitch2.isPressed() == false) {}
        // robot.lifting.setPower(0);
        // robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
        robot.init(hardwareMap, this);
        robot.shooting.setPower(0);
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
        robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initialSH = 0.6;
    }

    @Override
    public void loop() {
//********************************* Robot movement ********************************************************

        // controller variables
        // y: inverse of left stick's y value
        double y = exponential(-gamepad1.left_stick_y, 1);
        // x underscored: left stick's x value multiplied by the strafing coefficient in order to counteract imperfect strafing
        double x = exponential(gamepad1.left_stick_x,1);
        // rx: right stick's x value
        double rx = exponential(gamepad1.right_stick_x,1);

        double y2 = gamepad2.right_stick_y;

        liftingPower = y2/2;

        if (autoLifterState == States.Off) { // don't do manual movements if moving automatically


//           if (!robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed() && !robot.highSwitch1.isPressed() && !robot.highSwitch2.isPressed()) {
//               liftingPower = y2/2;
//           }
            if (robot.lowSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
                robot.lifting.setPower(Range.clip(liftingPower, -.5,0));
                //robot.lifting.setPower(Range.clip(liftingPower, 0,0.5));
            }
            else if (robot.highSwitch1.isPressed() || robot.highSwitch2.isPressed()) {
                robot.lifting.setPower(Range.clip(liftingPower,0, 0.3));
            }
            else {robot.lifting.setPower(liftingPower);}

        } else {
            if (autoLifterState == States.Forwards) {
                // Don't break the robot check
                if (robot.highSwitch1.isPressed() || robot.highSwitch2.isPressed()) {
                    robot.lifting.setPower(0);
                    robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    autoLifterState = States.Off;
                }
            } else { // lifter is going backwards, aka down
                // Don't break the robot check
                if (robot.lowSwitch1.isPressed() || robot.lowSwitch2.isPressed()) {
                    robot.lifting.setPower(0);
                    robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    autoLifterState = States.Off;
                }
            }
            if (!robot.lifting.isBusy()) {
                robot.lifting.setPower(0);
                robot.lifting.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                autoLifterState = States.Off;
            }
        }

        // logic version
        /*
        if (gamepad2.dpad_up && !gripperRaised) {
            raiseGripper();
            gripperRaised = true;
        }

        if (gamepad2.dpad_down && gripperRaised) {
            lowerGripper();
            gripperRaised = false;
        }

        if (gamepad2.dpad_left && !gripperClosed) {
            moveGripper(true);
            gripperClosed = true;
        }
        if (gamepad2.dpad_right && gripperClosed) {
            moveGripper(false);
            gripperClosed = false;
        }
        */

        // no logic version
        if (gamepad2.dpad_up && !robot.highSwitch1.isPressed() && !robot.highSwitch2.isPressed() && autoLifterState == States.Off) {
            raiseGripper();
        }

        if (gamepad2.dpad_down && !robot.lowSwitch1.isPressed() && !robot.lowSwitch2.isPressed() && autoLifterState == States.Off) {
            lowerGripper();
        }

        if (!gripperMoving) { // checks if the gripper is not already moving
            if (gamepad2.b) { // checks if the b button is pressed
                gripperPressed = true; // raises storage pressed flag
                gripperMoving = true; // raises the moving storage flag
                initialST = getRuntime(); // gets current time
            }
        }
        if (gripperPressed && gripperMoving) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!gripperClosed) { moveGripper(true); } // if the storage is not up it moves it up
            else if (gripperClosed) { moveGripper(false); } // if the storage is up it moves it down
        }
        if (gripperMoving) {
            if (getRuntime() - initialST > .3) {
                gripperPressed = false; // storage pressed flag is lowered
                gripperClosed = !gripperClosed; // updates state
                gripperMoving = false;
            }
        }

        //*******************Flywheel motor (shooting) *************************************************
      /*
        // shooting motors turn on by pressing the X key
         Original code for the flywheel.  It worked sometimes and most others it did not.

        //if (gamepad1.right_bumper) {
         //   robot.shooting.setPower(1);
         //   motorOff = false;
        //}

        if (gamepad1.x) {
            if (XClick == false) {
                XClick = true;
                if (flywheel == States.On) {
                    flywheel = States.Off;
                }
                else {
                    flywheel = States.On;
                }
            }
            else{
                XClick = false;
            }
        }

        if (flywheel == States.On) {
            robot.shooting.setPower(0.7);
        }else{
            robot.shooting.setPower(0);
            flywheel = States.Off;
        }
*/
        //Update 1-1-2021
        // This code turns the flywheel motor on or off.
        // This is similar to the storage box program except adapted for motor instead of servo

        // FLAGS:
        // flyWheel - holds the state of the Flywheel operation
        // flyMotor- holds if the flywheel motor is running or off
        // xClick - checks if flywheel button is pressed

        if (!flyWheel) { // checks if the flywheel is not already moving
            if (gamepad2.x) { // checks if the bumper is pressed
                xClick = true; // set X-button flag - X-button was pressed
                flyWheel = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
                // telemetry.addData("Status", "xClick " + gamepad1.x);
            }
        }

        if (xClick && flyWheel) { // checks if the flywheel is moving and if the storage pressed flag is raised
            if (!flyMotor) {robot.shooting.setPower(0.8);} // if the flywheel is off , turn it on
            else if (flyMotor) {robot.shooting.setPower(0);} // if the flywheel is on , turn it off
        }
        if (flyWheel) {
            if (getRuntime() - initialFL > .3) {
                flyWheel = !flyWheel; // updates state
                flyMotor = !flyMotor;
                xClick = false; // Flywheel button flag is lowered
            }
        }
        //*******************Flywheel motor (Power shot shooting) *************************************************
        /*

         */
        //Update 1-3-2021
        // This code turns the flywheel motor on or off.
        // This is similar to the Flywheel program but runs motor at lower power for power-shot

        // FLAGS:
        // flyWheel2 - holds the state of the Flywheel operation
        // flyMotor- holds if the flywheel motor is running or off
        // xClick - checks if flywheel button is pressed

        if (!flyWheel2) { // checks if the flywheel is not already moving
            if (gamepad2.y) { // checks if the bumper is pressed
                yClick = true; // set X-button flag - X-button was pressed
                flyWheel2 = true; // Flywheel process has started
                //flyMotor = true;
                initialFL = getRuntime(); // gets current time
                //   telemetry.addData("Status", "yClick " + gamepad1.y);
            }
        }

        if (yClick && flyWheel2) { // checks if the storage is moving and if the storage pressed flag is raised
            if (!flyMotor) {robot.shooting.setPower(0.70);} // if the flywheel is off , turn it on
            else if (flyMotor) {robot.shooting.setPower(0);} // // if the flywheel is on , turn it off
        }
        if (flyWheel2) {
            if (getRuntime() - initialFL > .3) {
                flyWheel2 = !flyWheel2; // updates state
                flyMotor = !flyMotor;
                yClick = false; // Flywheel button is lowered
            }
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
       // telemetry.addData("Strafing constant", "Strafing Constant = " + strafingConstant);
        telemetry.addData("Lifting Motor ", "(%.2f)",liftingPower);
        telemetry.addData("Low Switch 1 Status", robot.lowSwitch1.isPressed());
        telemetry.addData("Hi Switch 1 Status", robot.highSwitch1.isPressed());
    }

    public void stop() {
        robot.stopAllMotors();
    }

}
