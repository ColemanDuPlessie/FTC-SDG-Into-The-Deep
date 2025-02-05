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

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ControlHorizArmFromSlides;
import org.firstinspires.ftc.teamcode.backend.commands.ControlWrist;
import org.firstinspires.ftc.teamcode.backend.commands.DriveFromGamepad;
import org.firstinspires.ftc.teamcode.backend.commands.RetractHang;


/**
 * I should probably document this...
 */

@TeleOp(name="Teleop (THIS ONE)")
public class Teleop extends CommandbasedOpmode {

    public int armState = 2; // , 0 = horiz., 1 = diag., 2 = vert. all the way, 3 = reverse diag. for hang

    public int getArmState() {return armState;}

    private ControlHorizArmFromSlides makeArmHoriz = null;

    public void incrementArmState() {
        if (armState < 3) {
            armState++;
            if (armState == 1) {
                robot.arm.setTargetPosition(0.5);
                robot.slides.setTargetPosition(0.3);
                if (makeArmHoriz != null) {
                    scheduler.cancel(makeArmHoriz);
                }
                scheduler.schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new InstantCommand(() -> robot.wrist.setTargetPosition(0.25, robot.wrist.getRollPosition()))
                ));
            } else if (armState == 2) {
                robot.arm.vert();
                scheduler.schedule(new SequentialCommandGroup(
                    new WaitCommand(500), new InstantCommand(() -> {
                        if (robot.arm.getAngleFromVert() < 0.2 && robot.arm.getAngleFromVert() > -0.1) {
                            robot.slides.setTargetPosition(1.0);
                        }
                    }),
                    new WaitCommand(300),
                    new InstantCommand(() -> robot.wrist.setTargetPosition(0.35, robot.wrist.getRollPosition()))
                ));
            } else if (armState == 3) {
                robot.slides.setTargetPosition(0.65);
                scheduler.schedule(new SequentialCommandGroup(
                        new WaitCommand(500), new InstantCommand(() -> {
                    if (robot.slides.getTargetPosition() < 0.7 && robot.slides.getTargetPosition() > 0.6) {
                        robot.arm.setTargetPosition(0.15);
                    }
                })
                ));
            }
        } else {
            robot.arm.setTargetPosition(0.15);
            robot.slides.setTargetPosition(0.65);
        }
    }

    public void decrementArmState() {
        if (armState > 0) {
            armState--;
            if (armState == 0) {
                if (makeArmHoriz == null) {
                    makeArmHoriz = new ControlHorizArmFromSlides(robot.arm, robot.slides, timer);
                }
                scheduler.schedule(new SequentialCommandGroup(
                        new WaitCommand(Math.max(50, (int) robot.slides.getPosition()*1500-150)),
                        new InstantCommand(() -> scheduler.schedule(makeArmHoriz)),
                        new WaitCommand(300),
                        new InstantCommand(() -> robot.wrist.setTargetPosition(0.6, robot.wrist.getRollPosition()))
                ));
                robot.slides.setTargetPosition(0.1);
            } else if (armState == 1) {
                scheduler.schedule(new SequentialCommandGroup(
                    new WaitCommand(Math.max(50, (int) robot.slides.getPosition()*1000-300)),
                    new InstantCommand(() -> robot.arm.setTargetPosition(0.5)),
                    new WaitCommand(300),
                    new InstantCommand(() -> robot.wrist.setTargetPosition(0.25, robot.wrist.getRollPosition()))
                ));
                robot.slides.setTargetPosition(0.35);
            } else if (armState == 2) {
                scheduler.schedule(new RetractHang(robot.slides, timer)); // Lower, not raise, the slides, because we must be hanging!
                scheduler.schedule(new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(robot.arm::vert)
                ));
            }
        } else {
            scheduler.schedule(makeArmHoriz);
            robot.slides.setTargetPosition(0.1);
            if (robot.wrist.getPitchPosition() > 0.5) {
                robot.wrist.setTargetPosition(0.25, robot.wrist.getRollPosition());
            } else {
                robot.wrist.setTargetPosition(0.6, robot.wrist.getRollPosition());
            }
        }
    }

    @Override
    public void init() {
        robot.init(hardwareMap, true, telemetry);
        robot.slides.setTargetPosition(0.4);
    }

    @Override
    public void start() {
        scheduler.setDefaultCommand(robot.drivetrain, new DriveFromGamepad(robot.drivetrain, pad1, SetDrivingStyle.isFieldCentric));

        GamepadEx gamepad = new GamepadEx(gamepad1);

        scheduler.setDefaultCommand(robot.wrist, new ControlWrist(robot.wrist, gamepad));

        /*gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenReleased(robot.slides::hang);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenReleased(new RetractHang(robot.slides, timer)); // RetractHang only does things if we're already in hanging position
        */
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(this::decrementArmState);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(this::incrementArmState);
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenReleased(() -> robot.slides.incrementTargetPosition(0.1));
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenReleased(() -> robot.slides.incrementTargetPosition(-0.1));

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(robot.claw::cycle);


        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenReleased(robot.arm::cheatIncrementStartPos);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenReleased(robot.arm::cheatDecrementStartPos);

    }

    @Override
    public void loop() {
        telemetry.addData("Slides target position", robot.slides.getTargetPosition());
        telemetry.addData("Slides actual position", robot.slides.getPosition());
        telemetry.addData("Arm target position", robot.arm.getTargetPosition());
        telemetry.addData("Arm actual position", robot.arm.getPosition());
        telemetry.addData("Wrist target pitch", robot.wrist.getRollPosition());
        telemetry.addData("Wrist target roll", robot.wrist.getPitchPosition());
    }
}