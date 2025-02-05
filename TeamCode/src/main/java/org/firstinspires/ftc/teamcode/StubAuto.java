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

@Autonomous(name="Stub, 1 specimen only auto (specimen side) (THIS ONE)")
public class StubAuto extends CommandbasedOpmode {

    SampleMecanumDrive drive;

    TrajectorySequence startDepositTraj;

    private static final double REVERSE = Math.toRadians(180);
    private static final double CLOCKWISE90 = Math.toRadians(-90);

    public static double STARTX = 11;
    public static double STARTY = -65.5;
    public static double STARTTHETA = CLOCKWISE90;
    public static double DEPOSITY = -37.75;
    public static double PARKX = 50;
    public static double PARKY = -58;


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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.setAngleFromVert(-0.08);
                    robot.slides.setTargetPosition(0.35);
                    robot.wrist.setTargetPosition(0.2, DifferentialWristSubsystem.rollCenterPosition);
                })
                .splineToConstantHeading(new Vector2d(STARTX, DEPOSITY), STARTTHETA+REVERSE)
                .waitSeconds(1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> robot.wrist.setTargetPosition(0.0, DifferentialWristSubsystem.rollCenterPosition))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> robot.slides.setTargetPosition(0.0))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> robot.slides.setTargetPosition(0.2))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.wrist.center())
                .splineToSplineHeading(new Pose2d(STARTX, STARTY*0.3+DEPOSITY*0.7, 0), 0)
                .waitSeconds(.1)
                .splineToSplineHeading(new Pose2d(36, -38, 0), 0)// moving to the side
                .splineToConstantHeading(new Vector2d(32, -18), -CLOCKWISE90*0.45)//getting into position to push
                .splineToConstantHeading(new Vector2d(36, -10), -90)//in position to push
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(48, -55, 0))// push to obs
                .lineToLinearHeading(new Pose2d(48,-45, 90))// back up and turn to collect
                .waitSeconds(3)
                //lineToLinearHeading(new Pose2d(48,-56,90))
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