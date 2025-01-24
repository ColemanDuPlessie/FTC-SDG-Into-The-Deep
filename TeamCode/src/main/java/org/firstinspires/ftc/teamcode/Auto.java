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
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


/**
 * I should probably document this...
 */

@Autonomous(name="Auto (Yellow-side) (THIS ONE)")
public class Auto extends CommandbasedOpmode {

    SampleMecanumDrive drive;

    TrajectorySequence startDepositTraj;

    TrajectorySequence depositParkTraj;

    TrajectorySequence depositIntakeTraj;

    TrajectorySequence intakeDepositTraj;

    private static final double REVERSE = Math.toRadians(180);
    private static final double CLOCKWISE90 = Math.toRadians(-90);

    public static double STARTX = -11;
    public static double STARTY = -64.5;
    public static double STARTTHETA = CLOCKWISE90;
    public static double DEPOSITY = -48;
    public static double PIXELINTAKEX = -57;
    public static double PIXELINTAKEY = -24;
    public static double TRAVERSESTARTX = -56;
    public static double TRAVERSEENDX = 48;
    public static double TRAVERSEY = -12;


    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false, telemetry);

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, STARTTHETA);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        startDepositTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.setTargetPosition(0.3);
                    robot.wrist.setTargetPosition(0.5, 0.6);
                })
                .splineToConstantHeading(new Vector2d(STARTX, DEPOSITY), STARTTHETA+REVERSE)
                .setReversed(false)
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.wrist.setTargetPosition(0.5, 0.8))
                .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY), REVERSE)
                .build();

        // TODO NOT VETTED BELOW THIS LINE

        depositParkTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                    .lineTo(new Vector2d(PIXELINTAKEX - 1, PIXELINTAKEY))
                    .waitSeconds(1.25)
                    .lineTo(new Vector2d(PIXELINTAKEX - 3, PIXELINTAKEY))
                    .waitSeconds(1.5)
                    .lineTo(new Vector2d(PIXELINTAKEX, PIXELINTAKEY))
                    .build();

        depositIntakeTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 35)
                    .addTemporalMarker(5.5, () -> robot.slides.setTargetPosition(0.0))
                    .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY * 0.75 + TRAVERSEY * 0.25), -CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(TRAVERSESTARTX, TRAVERSEY), 0)
                    .splineToConstantHeading(new Vector2d(TRAVERSEENDX, TRAVERSEY), 0)
                    .build();



        intakeDepositTraj = drive.trajectorySequenceBuilder(new Pose2d(TRAVERSEENDX, TRAVERSEY, REVERSE))
                    .splineToConstantHeading(new Vector2d(STARTX - 4, DEPOSITY * 0.5 + TRAVERSEY * 0.5), CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(STARTX, DEPOSITY), 0)
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                    .splineToConstantHeading(new Vector2d(STARTX - 4, DEPOSITY), REVERSE)
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
        /*
        if (!SetDrivingStyle.shortAuto) {
            auto.add(new FollowRRTraj(robot.drivetrain, drive, depositIntakeTraj));
            auto.add(new FollowRRTraj(robot.drivetrain, drive, intakeDepositTraj));
        }
        auto.add(new FollowRRTraj(robot.drivetrain, drive, depositParkTraj));
        */
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