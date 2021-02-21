package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="AutonomousTestSecondTournament", group="Motion")
public class SecondTournamentAutoTest extends LinearOpMode {
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
        robot.goDistanceAcceleration(65,0.9,false,5,50,this);
        robot.wait(300,this);  // Wait for robot to stop before reading rings
        robot.goDistanceAcceleration(62,0.8,false,8,50,this);

            robot.GoDistanceCM2(130, .7, false,this);   // added extra 20 cm to get ring out of the way.  Wobble was landing on top of ring 1-27-2021
            robot.strafeDistanceCM2(30, .7,false, this);
            robot.GoDistanceCM2(-28, -.7, false,this);  //  back up and drop wobble away from ring
            // drop wobble goal here
            robot.lowerGripper(250);
            robot.gripperOpen();
            robot.wait(1000L,this);
            robot.lowerGripper(900);
            robot.GoDistanceCM2(-10, .2, false, this);
            robot.RotateDEG(171, .7, this);
            robot.goDistanceAcceleration(138, 0.9, false, 5, 60, this);
        robot.wobbleFind(35, 0.2, 40, this);
        boolean success = ProgrammingFrame.wobble.success;
        int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
        robot.gripperOpen();
        // robot.RotateDEG(3, .2, this);
        // robot.wait(1000L,this);
        robot.raiseGripper(750);
        robot.GoDistanceCM2(9, .2, false, this);
        robot.gripperClose();
        robot.wait(600, this);
        //robot.gripperOpen();
        //robot.wait(1000, this);
        //robot.GoDistanceCM2(-20, 30, false, this);
        //robot.gripperClose();
        //robot.lowerGripper(800);
        robot.raiseGripper(400);
            //robot.GoDistanceCM2(-30, -.7, false,this);
        robot.RotateDEG((180 - wobbleDegrees) - 2 , .7, this);  // degrees was -5
        robot.goDistanceAcceleration(130, .9, false, 5, 75, this);
        robot.lowerGripper(100);
        robot.gripperOpen();
        //  robot.GoDistanceCM2(-20, -.7,false, this);
        //   robot.strafeDistanceCM2(55, .7,false, this);  // go sidways and forward  to launch line without
        //   robot.GoDistanceCM2(35, .7, false, this);   // disturbing wobble (strafe needed for Target zone A only)

    }

}
