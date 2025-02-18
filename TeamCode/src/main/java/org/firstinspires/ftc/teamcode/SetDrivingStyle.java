/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.utilities.ToggleButton;


/**
 * I should probably document this...
 */

@TeleOp(name="Set Driving Style", group="Set Driving Style")
public class SetDrivingStyle extends CommandbasedOpmode {

    ToggleButton xToggle;
    ToggleButton yToggle;
    ToggleButton bToggle;

    ToggleButton rToggle;

    public static boolean isFieldCentric         = false;
    public static boolean isBlue                 = false;
    public static boolean memorizedSlidePosition = false;
    public static int autoSecondsDelay = 0;
    public static boolean shortAuto              = false;

    private boolean dpadDownWasDown = false;
    private boolean dpadUpWasDown = false;

    @Override
    public void init() {
        xToggle = new ToggleButton(pad1::getX);
        yToggle = new ToggleButton(pad1::getY);
        bToggle = new ToggleButton(pad1::getB);
        rToggle = new ToggleButton(pad1::getRightBumper);

        robot.init(hardwareMap, true, telemetry);
    }

    @Override
    public void init_loop() {
        isFieldCentric         = xToggle.get();
        isBlue                 = yToggle.get();
        memorizedSlidePosition = rToggle.get();
        shortAuto              = bToggle.get();

        if (!pad1.getDpadDown() && dpadDownWasDown) {
            autoSecondsDelay = Math.max(autoSecondsDelay-1, 0);
        } else if (!pad1.getDpadUp() && dpadUpWasDown) {
            autoSecondsDelay += 1;
        }

        telemetry.addData("Field Centric?: (toggle with x)", isFieldCentric);
        telemetry.addData("We are on the (toggle with y)", isBlue ? "Blue Alliance" : "Red Alliance");
        telemetry.addData("Slide height is (toggle with r bumper)", memorizedSlidePosition ? "Memorized & Adjustable (WIP)" : "Setpoints");
        telemetry.addData("Auto type: (toggle with b)", shortAuto ? "No cycles (worse)" : "Cycles (longer)");
        telemetry.addData("Auto start delay (seconds) (adjust with dpad up/down)", autoSecondsDelay);
    }

    @Override
    public void start() {
        requestOpModeStop();
    }

    @Override
    public void loop() {}
}