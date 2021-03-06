package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="4RingsTest", group="Motion")
public class FourRingsTest extends LinearOpMode {
    ProgrammingFrame robot = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.goDistanceAcceleration(65, 0.8, false, 5, 50, this);
        robot.strafeDistanceCM2(23, 0.3, false, this);
        robot.intake(ProgrammingFrame.States.Forwards);
        robot.GoDistanceCM2(38, 0.3, false, this);
        sleep(2000L);
    }
}