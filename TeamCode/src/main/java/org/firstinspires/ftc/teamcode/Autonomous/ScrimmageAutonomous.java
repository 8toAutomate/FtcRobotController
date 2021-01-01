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
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.GoDistanceCM();
        robot.storageServo.setPosition(0);
        robot.shooting.setPower(.8);
        robot.ringPusher.scaleRange(0, 1.0);
        initialSH = getRuntime();
        while (getRuntime() - initialSH < 4.0) {}

         for (int i=1; i<=3; i++) {
             initialSH = getRuntime();
             robot.ringPusher.setPosition(1.0);
            while (getRuntime() - initialSH < 2) {}
            robot.ringPusher.setPosition(0);
            initialSH = getRuntime();
            while (getRuntime() - initialSH < 2) {}
        }
         robot.shooting.setPower(0);
         robot.GoDistanceCM(160, .7, this);
    }

}