package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="AutonomousSecondTourney", group="Motion")
public class BlueAutoSecondTourney extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.gripperOpen();
        robot.gripperOpen();
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

    //    robot.wait(500L,this);
        robot.raiseGripper(750);

        robot.gripperClose();
        robot.wait(500L,this);
        robot.raiseGripper(350);

       // robot.GoDistanceCM2(70, .5, false,this);
        robot.goDistanceAcceleration(65,0.9,false,5,20,this);
        robot.wait(300,this);  // Wait for robot to stop before reading rings

            // Detect the rings here and return A, B, C, or E for Error
        ringAt = robot.ringFinderDistance(this);
        if (ringAt == 'E') {  // Top saw a ring but bottom didn't somehow, try one more time
            char tryAgain = robot.ringFinderDistance(this);  // If this fails it will take C path
            if (tryAgain == 'E') {
                ringAt = 'C';
            }
            else {
                ringAt = tryAgain;
            }
        }
       // ringAt = robot.ringFinderDistance();
        telemetry.addData("Target zone", ringAt);
        telemetry.update();

        // Gets us to the target zone
        //robot.GoDistanceCM2(60, .5, false, this);
        robot.flywheel(true, 0.85);
        robot.goDistanceAcceleration(62,0.8,false,5,25,this);
        robot.storage(true, this);
       // robot.flywheel(true, 0.8);
      //  robot.wait(2500,this);

        for (int i = 0; i<4; i++) {                             // added extra shot in case last ring is stuck.
            robot.pushRing(0.4, this);
            robot.wait(400, this); // timeout was 1200 fir first tourney
        }
        robot.flywheel(false, 0.0);

        if (ringAt == 'A') {
            robot.GoDistanceCM2(29, .7, false, this);
            robot.strafeDistanceCM2(-32, .7,false, this);
            // drop wobble stick here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(300L,this);
            robot.lowerGripper(900);
            robot.RotateDEG(146, 0.7, this);        // raotion was 147 at 0.5 power
            robot.goDistanceAcceleration(103, 0.9, false, 5, 70, this);
            robot.wobbleFind(35, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            robot.gripperOpen();
           // robot.RotateDEG(3, .2, this);
            // robot.wait(1000L,this);
            robot.raiseGripper(750);
            robot.GoDistanceCM2(8, .2, false, this);
            robot.gripperClose();
            robot.wait(700, this);
            //robot.gripperOpen();
            //robot.wait(1000, this);
            //robot.GoDistanceCM2(-20, 30, false, this);
            //robot.gripperClose();
            //robot.lowerGripper(800);
            robot.raiseGripper(400);
            robot.RotateDEG((180 - wobbleDegrees) + 12, .7, this);
            robot.goDistanceAcceleration(90, .9, false, 5, 75, this);
            robot.lowerGripper(100);
            robot.gripperOpen();
          //  robot.GoDistanceCM2(-20, -.7,false, this);
         //   robot.strafeDistanceCM2(55, .7,false, this);  // go sidways and forward  to launch line without
         //   robot.GoDistanceCM2(35, .7, false, this);   // disturbing wobble (strafe needed for Target zone A only)
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(95, .7, false,this);   // added extra 20 cm to get ring out of the way.  Wobble was landing on top of ring 1-27-2021
            robot.strafeDistanceCM2(35, .7,false, this);
            robot.GoDistanceCM2(-20, -.7, false,this);  //  back up and drop wobble away from ring
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(900);
            robot.GoDistanceCM2(-30, -.7, false,this);
        }
        else {   // ring at C
            robot.GoDistanceCM2(147, .7, false,this);
            robot.strafeDistanceCM2(-30, .7,false, this);
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(950);
            robot.GoDistanceCM2(-80, -.7, false,this);
        }
        robot.wait(300, this);
        //robot.GoDistanceCM2(-20, 30, false, this);
        robot.lowerGripper(900);
      //  robot.gripperClose();
        robot.RotateDEG(45,0.7,this);
        robot.GoDistanceCM2(15, .8,false, this);  // navigate to white line
        robot.gripperClose();
        robot.storage(false,this);

        robot.wait(1500L,this);  //allow time for servo to finish closing grip before terminating
        // /*  Debug: comment out rest of method  MAx M. 12-24-2020
        // Add function that drops a wobble goal
        // Move to the launch line
      /*  if (ringAt == 'A') {
            robot.StrafeCM2(-22, .7, this);
            robot.GoDistanceCM2(27, .7, this);
        }
        else if (ringAt == 'B') {
            robot.GoDistanceCM2(-27, .7, this);
        }
        else {
            robot.GoDistanceCM2(-86, .7, this);
            robot.StrafeCM2(-59, .7, this);
        }

*/
       // while (opModeIsActive()) {}  //  Empty while loop - program waits until user terminates op-mode

    }

}
