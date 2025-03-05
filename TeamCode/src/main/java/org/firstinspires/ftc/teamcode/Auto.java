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

import static org.firstinspires.ftc.teamcode.SetDrivingStyle.autoSecondsDelay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ControlHorizArmFromSlides;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.backend.subsystems.DifferentialWristSubsystem;

import java.util.ArrayList;


/**
 * I should probably document this...
 */

@Autonomous(name="Auto (Yellow-side) (THIS ONE)")
public class Auto extends CommandbasedOpmode {

    SampleMecanumDrive drive;

    TrajectorySequence startDepositTraj;

    private static final double REVERSE = Math.toRadians(180);
    private static final double CLOCKWISE90 = Math.toRadians(-90);

    public static double STARTX = -11;
    public static double STARTY = -64.5;
    public static double STARTTHETA = CLOCKWISE90;
    public static double DEPOSITY = -37.75;
    public static double SAMPLEINTAKEX = -54;
    public static double SAMPLEINTAKEY = -36;
    public static double PARKX = -20;
    public static double PARKY = -12;

    private ControlHorizArmFromSlides makeArmHoriz = null;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false, telemetry);

        makeArmHoriz = new ControlHorizArmFromSlides(robot.arm, robot.slides, timer, 0.05);

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, STARTTHETA);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        startDepositTraj = drive.trajectorySequenceBuilder(startPose)

                // Deposit specimen
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.setAngleFromVert(-0.08);
                    robot.slides.setTargetPosition(0.35);
                    robot.wrist.setTargetPosition(0.2, DifferentialWristSubsystem.rollCenterPosition);
                })
                .splineToConstantHeading(new Vector2d(STARTX, DEPOSITY), STARTTHETA+REVERSE)
                .setReversed(false)
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> robot.wrist.setTargetPosition(0.0, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.slides.setTargetPosition(0.0))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> robot.slides.setTargetPosition(0.2))

                // Intake first sample
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    scheduler.schedule(makeArmHoriz);
                    robot.claw.open();
                    robot.wrist.setTargetPosition(0.6, DifferentialWristSubsystem.rollCenterPosition);
                })
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX, SAMPLEINTAKEY+3.0, -CLOCKWISE90), REVERSE)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, robot.claw::close)

                // Deposit first sample
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    scheduler.cancel(makeArmHoriz);
                    robot.arm.vert();
                    robot.wrist.setTargetPosition(0.25, DifferentialWristSubsystem.rollCenterPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.slides.setTargetPosition(1.0))
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-5, SAMPLEINTAKEY-16.5, REVERSE-CLOCKWISE90/2), REVERSE-CLOCKWISE90/2)
                .waitSeconds(2.75)
                .UNSTABLE_addTemporalMarkerOffset(-2.25, () -> robot.wrist.setTargetPosition(0.4, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, robot.claw::open)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.wrist.setTargetPosition(0.15, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.2))
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> scheduler.schedule(makeArmHoriz))

                // Intake second sample
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-7.25, SAMPLEINTAKEY-6, 0), -CLOCKWISE90*5.0/4.0)
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-10, SAMPLEINTAKEY+1.5, -CLOCKWISE90), -CLOCKWISE90)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> robot.wrist.setTargetPosition(0.6, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(0.5, robot.claw::close)

                // Deposit second sample
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> {
                    scheduler.cancel(makeArmHoriz);
                    robot.arm.vert();
                    robot.wrist.setTargetPosition(0.25, DifferentialWristSubsystem.rollCenterPosition);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.0, () -> robot.slides.setTargetPosition(1.0))
                .waitSeconds(2.8)
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-8.25, SAMPLEINTAKEY-4, 0), CLOCKWISE90*3.0/4.0)
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-5.5, SAMPLEINTAKEY-10.5, REVERSE-CLOCKWISE90/2), REVERSE-CLOCKWISE90/2)
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-6.5, SAMPLEINTAKEY-19, REVERSE-CLOCKWISE90/2), CLOCKWISE90)
                .waitSeconds(2.75)
                .UNSTABLE_addTemporalMarkerOffset(-2.25, () -> robot.wrist.setTargetPosition(0.4, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, robot.claw::open)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> robot.wrist.setTargetPosition(0.15, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.4))
                // Park
                .splineToSplineHeading(new Pose2d(SAMPLEINTAKEX-5.5, (SAMPLEINTAKEY-17.5)*0.75+PARKY*0.25, CLOCKWISE90*5/6), -CLOCKWISE90)
                .splineToSplineHeading(new Pose2d(PARKX, PARKY, 0), 0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.arm.setTargetPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wrist.setTargetPosition(0.4, DifferentialWristSubsystem.rollCenterPosition))
                .waitSeconds(2)
                .build();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        ArrayList<Command> auto = new ArrayList<>();
        if (autoSecondsDelay != 0) {
            auto.add(new WaitCommand(autoSecondsDelay * 1000));
        }
        auto.add(new FollowRRTraj(robot.drivetrain, drive, startDepositTraj));
        scheduler.schedule(false, new SequentialCommandGroup(auto.toArray(new Command[0])));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void end() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.drivetrain.getHeading()+Math.toRadians(180));
    }
}