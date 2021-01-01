package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;
import org.firstinspires.ftc.teamcode.TeleOp.MecanumDriveIntake;

@Autonomous(name="ScrimmageAutonomous", group="Motion")
public class ScrimmageAutonomous extends LinearOpMode {
    // This program starts on the left blue line, shoots at the high goal, drops off a wobble goal
    // in it's target, than drives to center shooting spot to park at the end.

    ProgrammingFrame robot = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;
    enum States {
        Forwards, Backwards, Off, On
    }

    ScrimmageAutonomous.States ringPusher = ScrimmageAutonomous.States.Backwards;
    ScrimmageAutonomous.States flywheel = ScrimmageAutonomous.States.Off;
    ScrimmageAutonomous.States lifting = ScrimmageAutonomous.States.Backwards;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        // Have method(s) that shoot 3 rings here, likely in the high goal
        robot.shooting.setPower(1);
        robot.ringPusher.scaleRange(0, 1.0);
        initialSH = getRuntime();
        robot.ringPusher.setPosition(1.0);

        while (getRuntime() - initialSH < .5) {
    }
        robot.ringPusher.setPosition(0);
        /* for (int i=1; i<=3; i++)
        {
            robot.ringPusher.setPosition(1.0);
            robot.ringPusher.setPosition(0);
        }

         */
    }
}