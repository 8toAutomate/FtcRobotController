package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="ScrimmageAutoPowerShot", group="Motion")
public class ScrimmageAutoPowerShot extends LinearOpMode {
    // This program starts on the left red line, shoots at the high goal, drops off a wobble goal in
    // it's target, than drives to center shooting spot to park at the end.
    char ringAt;
    ProgrammingFrame robot   = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        waitForStart();
        robot.flywheel(true, 0.75);
        robot.GoDistanceCM2(120, 0.7, true, this);
        robot.GoDistanceCM2(25, 0.3, false, this);
        robot.flywheel(true, 0.65);
        robot.storage(true,this);
        double initialSH = getRuntime();
        while (getRuntime() - initialSH < 0.5) {}

        //robot.strafeDistanceCM2(45, 0.5, this);
       // robot.ringPusher.scaleRange(0, 1.0);
        robot.pushRing(0.5,this);
        robot.strafeDistanceCM2(20, 0.2, false,this);
        initialSH = getRuntime();
        while (getRuntime() - initialSH < 0.5) {}
        robot.pushRing(0.5,this);
        robot.strafeDistanceCM2(20, 0.2, false, this);
        initialSH = getRuntime();
        while (getRuntime() - initialSH < 0.5) {}
        robot.pushRing(0.5,this);
        initialSH = getRuntime();
        while (getRuntime() - initialSH < 0.5) {}
        robot.pushRing(0.5,this); // push ring again - sometimes the last ring gets stuck
        robot.storage(false,this);
        robot.flywheel(false, 0);
        robot.GoDistanceCM2(20, 0.5, false,this);

        /*
        robot.flywheel(true, 0.8);

        robot.storage(true);

        for (int i = 0; i < 3; i++) {
            robot.pushRing(0.5, this);
        }

        robot.flywheel(false, 0.8);

        robot.GoDistanceCM(160, .7, this);

         */
    }
}
