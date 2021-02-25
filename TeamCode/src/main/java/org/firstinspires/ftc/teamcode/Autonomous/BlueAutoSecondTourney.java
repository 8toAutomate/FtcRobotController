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
        robot.lowerGripper(100);
        robot.gripperOpen();
        robot.gripperOpen();
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

    //    robot.wait(500L,this);
        robot.raiseGripper(730);

        robot.gripperClose();
        robot.wait(500L,this);
        robot.raiseGripper(350);

       // robot.GoDistanceCM2(70, .5, false,this);
        robot.goDistanceAcceleration(65,0.8,false,5,60,this);// 0.9 5 20
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
   //     telemetry.addData("Target zone", ringAt);
     //   telemetry.update();

        // Gets us to the target zone
        //robot.GoDistanceCM2(60, .5, false, this);
        robot.flywheel(true, 0.80);
        robot.storage(true, this);
        robot.goDistanceAcceleration(62,0.8,false,5,50,this); // back ramp was 25 2-20-21

       // robot.flywheel(true, 0.8);
      //  robot.wait(2500,this);

        for (int i = 0; i<3; i++) {                             // added extra shot in case last ring is stuck.
            robot.pushRing(0.4, this);
            robot.flywheel(true, 0.85); // increase power for second and third shot
            robot.wait(400, this); // timeout was 1200 fir first tourney
        }
        robot.flywheel(false, 0.0);

        if (ringAt == 'A') {
            robot.GoDistanceCM2(32, .7, false, this); //distance was 29 2-22-21 added extra
            robot.strafeDistanceCM2(-26, .7,false, this);
            // drop wobble stick here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(400L,this);
            robot.GoDistanceCM2(-7,0.3,false,this);
            robot.lowerGripper(950);
            robot.RotateDEG(148, 0.7, this);        // rotation was 146 at 0.7 power is 148  2/22/21
            robot.goDistanceAcceleration(97, 0.9, false, 5, 70, this);// distance was 103 is 101 2-22-21
           // robot.wobbleFind(35, 0.2, 40, this);
            robot.wobbleFind2(45, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            int travelDist = ProgrammingFrame.wobble.travelDist;
            if (success) {
                robot.gripperOpen();
                robot.raiseGripper(750);
                robot.GoDistanceCM2(travelDist, .2, false, this);
                robot.gripperClose();
                robot.wait(400, this);
                robot.raiseGripper(400);
            }
            robot.RotateDEG((180 - wobbleDegrees) + 13, .7, this);
            robot.goDistanceAcceleration(90, .9, false, 5, 75, this);  // distance was 90 2-22/21
            robot.lowerGripper(100);
            robot.gripperOpen();
            robot.wait(400, this);
            //robot.GoDistanceCM2(-20, 30, false, this);
            robot.lowerGripper(200);
            //  robot.gripperClose();
            robot.GoDistanceCM2(-10,0.5,false,this);
            robot.RotateDEG(50,0.7,this);
            robot.GoDistanceCM2(35, .8,false, this);  // navigate to white line
        }   // end drop zone A

        else if (ringAt == 'B') {
            robot.goDistanceAcceleration(125, 0.8, false,8, 75, this);   // was 127 2-24-21, added extra 20 cm to get ring out of the way.  Wobble was landing on top of ring 1-27-2021
            robot.GoDistanceCM2(-29, -0.4, false, this);  //  back up and drop wobble away from ring
            robot.strafeDistanceCM2(29, 0.5,false, this);
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(500L,this);
            robot.lowerGripper(950);
            robot.GoDistanceCM2(-10, .3, false, this);
            robot.RotateDEG(174, 0.6, this); // was 171
            robot.goDistanceAcceleration(133, 0.9, false, 5, 75, this);  //distance was 135 2-22-21
            // robot.wobbleFind(35, 0.2, 40, this);
            robot.wobbleFind2(45, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            int travelDist = ProgrammingFrame.wobble.travelDist;
            if (success) {
                robot.gripperOpen();
                robot.raiseGripper(750);
                robot.GoDistanceCM2(travelDist, .2, false, this);
                robot.gripperClose();
                robot.wait(500, this);
                robot.raiseGripper(400);
            }
           // robot.gripperOpen();
           // robot.raiseGripper(750);
            //robot.GoDistanceCM2(9, .2, false, this);
            //robot.gripperClose();
           // robot.wait(500, this);
           // robot.raiseGripper(400);
            //robot.GoDistanceCM2(-30, -.7, false,this);
            robot.RotateDEG((180 - wobbleDegrees) + 5 , 0.6, this);  // degrees 4.5 would be best  2-22-21
            robot.goDistanceAcceleration(130, .9, false, 5, 75, this);

            robot.lowerGripper(100);
            robot.gripperOpen();    //drop wobble
            robot.wait(400,this);
           // robot.lowerGripper(1000);
                   }// end drop zone B

        else {   // ring at C
            robot.goDistanceAcceleration(149, 0.8, false,8, 80, this);
            robot.strafeDistanceCM2(-11, 0.6,false, this);
            //robot.GoDistanceCM2(-28, -0.4, false, this);  //  back up and drop wobble away from ring

            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(300L,this);
            robot.lowerGripper(950);
            //robot.GoDistanceCM2(-10, .3, false, this);
            robot.RotateDEG(168, 0.6, this); // was 171

            robot.goDistanceAcceleration(203, 0.9, false, 5, 85, this);     //distance was 205 2-22-21
            // robot.wobbleFind(35, 0.2, 40, this);
            robot.wobbleFind2(45, 0.2, 40, this);
            boolean success = ProgrammingFrame.wobble.success;
            int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
            int travelDist = ProgrammingFrame.wobble.travelDist;
            if (success) {
                robot.gripperOpen();
                robot.raiseGripper(750);
                robot.GoDistanceCM2(travelDist, .2, false, this);
                robot.gripperClose();
                robot.wait(500, this);
                robot.raiseGripper(400);
            }
            //robot.gripperOpen();
           // robot.raiseGripper(750);
           // robot.GoDistanceCM2(9, .2, false, this);
            //robot.gripperClose();
            //robot.wait(400, this);
            //robot.raiseGripper(400);

            robot.RotateDEG((180 - wobbleDegrees)  , 0.6, this);  // degrees was -5
            robot.goDistanceAcceleration(207, .9, false, 5, 85, this);

            robot.lowerGripper(100);
            robot.gripperOpen();
            robot.wait(300,this);
            robot.goDistanceAcceleration(-60, 1, false, 5, 95, this);

        }// end drop zone c
        robot.lowerGripper(300);
         robot.storage(false,this);

        robot.gripperClose();
        robot.wait(200L,this);  //allow time for servo to finish closing grip before terminating
        // /*  Debug: comment out rest of method  MAx M. 2-15-2021
        // Add function hat navigates to launch line if second wobble misses
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

    }       // end runOpmode

}
