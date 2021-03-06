package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingFrame;

@Autonomous(name="4RingsTest2", group="Motion")
public class FourRingsTest2 extends LinearOpMode {
    ProgrammingFrame robot = new ProgrammingFrame();
    private ElapsedTime runtime = new ElapsedTime();
    double initialSH;
    ProgrammingFrame.wobble wobble = new ProgrammingFrame.wobble();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.storageServo.setPosition(0);
        robot.shooting.setPower(.78);
        robot.goDistanceAcceleration(65,0.8,false,5,50,this);

        for (int i = 1; i<=3; i++) {
            robot.pushRing(0.4, this);
            robot.flywheel(true, 0.8); // increase power for second and third shot
            robot.wait(400, this); // timeout was 1200 fir first tourney
        }
        robot.storageServo.setPosition(1);
        robot.strafeDistanceCM2(23, 0.3, false, this);
        robot.intake(ProgrammingFrame.States.Forwards);
        robot.GoDistanceCM2(17, 0.3, false, this);
        sleep(2000L);
        robot.intake(ProgrammingFrame.States.Off);
        robot.shooting.setPower(.8);
        robot.storageServo.setPosition(0);
        robot.strafeDistanceCM2(-30, .3, false, this);

        for (int i = 1; i<=2; i++) {
            robot.pushRing(0.4, this);
            robot.flywheel(true, 0.8); // increase power for second and third shot
            robot.wait(400, this); // timeout was 1200 fir first tourney
        }





    }
}
