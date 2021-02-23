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
        //robot.gripperOpen();
        //robot.gripperOpen();
        robot.lowerGripper(300);
        waitForStart();

        // Have method(s) that shoot 3 rings here, likely in the high goal
        // Driving to the starter stack

        robot.wobbleFind2(40, 0.2, 40, this);
        boolean success = ProgrammingFrame.wobble.success;
        int wobbleDegrees = ProgrammingFrame.wobble.rotateBack;
        int travelDist = ProgrammingFrame.wobble.travelDist;
        if (success) {
           // robot.gripperOpen();
           // robot.raiseGripper(750);
           // robot.GoDistanceCM2(travelDist, .2, false, this);
           // robot.gripperClose();
           // robot.wait(400, this);
           // robot.raiseGripper(400);
           // robot.gripperOpen();
           // robot.wait(300,this);
           // robot.lowerGripper(900);
            robot.RotateDEG((180 - wobbleDegrees) , .7, this);
        }
        //robot.RotateDEG((180 - wobbleDegrees) + 12, .7, this);
        //  robot.GoDistanceCM2(-20, -.7,false, this);
        //   robot.strafeDistanceCM2(55, .7,false, this);  // go sidways and forward  to launch line without
        //   robot.GoDistanceCM2(35, .7, false, this);   // disturbing wobble (strafe needed for Target zone A only)

    }

}
