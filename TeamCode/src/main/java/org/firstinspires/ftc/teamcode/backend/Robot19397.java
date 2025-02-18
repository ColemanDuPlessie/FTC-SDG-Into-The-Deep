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

package org.firstinspires.ftc.teamcode.backend;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DifferentialWristSubsystem;

/**
 * I should probably write this documentation...
 */
public class Robot19397 extends Robot {

    public static Telemetry tele;

    public final ArmSubsystem arm;
    public final ClawSubsystem claw;
    public final SlidesSubsystem slides;
    public final DrivetrainSubsystem drivetrain;
    public final DifferentialWristSubsystem wrist;
    // public final CameraSubsystem camera;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime timer;

    /* Constructor */
    public Robot19397(ElapsedTime timer){
        this.timer = timer;
        this.drivetrain = new DrivetrainSubsystem();
        this.arm = new ArmSubsystem();
        this.claw = new ClawSubsystem();
        this.slides = new SlidesSubsystem();
        this.wrist = new DifferentialWristSubsystem();
        // this.camera = new CameraSubsystem();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap, Telemetry t) {
        this.init(hwMap, false, t);
    }

    public void init(HardwareMap hwMap, boolean isTeleop, Telemetry t) {
        // Save reference to Hardware map
        this.hwMap = hwMap;
        tele = t;
        drivetrain.init(hwMap, isTeleop);
        CommandScheduler.getInstance().registerSubsystem(this.drivetrain);
        arm.init(timer, hwMap, isTeleop);
        CommandScheduler.getInstance().registerSubsystem(this.arm);
        claw.init(timer, hwMap, isTeleop);
        CommandScheduler.getInstance().registerSubsystem(this.claw);
        slides.init(timer, hwMap, isTeleop, arm);
        CommandScheduler.getInstance().registerSubsystem(this.slides);
        wrist.init(timer, hwMap, isTeleop);
        CommandScheduler.getInstance().registerSubsystem(this.wrist);
        // camera.init(hwMap, isTeleop);
        // CommandScheduler.getInstance().registerSubsystem(this.camera);
    }

 }

