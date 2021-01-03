/* Copyright (c) 2017-2020 FIRST. All rights reserved.
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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ProgrammingFrame;


@TeleOp(name = "ColorTestThanish2Sensors", group = "Sensor")
//@Disabled
public class ColorTestThanish2Sensors extends OpMode {

  /** The colorSensor field will contain a reference to our color sensor hardware object */
  ProgrammingFrame robot   = new ProgrammingFrame();

  float gain = 2;

  @Override public void init() {

    robot.init(hardwareMap, this);
    if (robot.colorSensor1 instanceof SwitchableLight)
    {
      ((SwitchableLight)robot.colorSensor1).enableLight(true);
    }
    if (robot.colorSensor2 instanceof SwitchableLight)
    {
      ((SwitchableLight)robot.colorSensor1).enableLight(true);
    }
    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");

  }


  public void loop() {

    telemetry.addData("Gain", gain);

    robot.colorSensor1.setGain(gain);
    robot.colorSensor2.setGain(gain);

    NormalizedRGBA colors1 = robot.colorSensor1.getNormalizedColors();
    NormalizedRGBA colors2 = robot.colorSensor2.getNormalizedColors();

    float red1 = colors1.red;
    float red2 = colors2.red;
    float green1 = colors1.green;
    float green2 = colors2.green;
    float blue1 = colors1.blue;
    float blue2 = colors2.blue;

    telemetry.addLine();
    telemetry.addData("red1", red1);
    telemetry.addData("green1", green1);
    telemetry.addData("blue1", blue1);
    telemetry.addLine();
    telemetry.addData("red2", red2);
    telemetry.addData("green2", green2);
    telemetry.addData("blue2", blue2);


    telemetry.update();

  }
}


