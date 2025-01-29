package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.backend.subsystems.DifferentialWristSubsystem;

@Config
public class ControlWrist extends CommandBase {

    public double targetPitchPosition;
    public double targetRollPosition;
    private double maxMoveSpeed = 0.01;

    private DifferentialWristSubsystem wrist;
    private final GamepadEx triggers;


    public ControlWrist(DifferentialWristSubsystem w, GamepadEx triggers) {
        wrist = w;
        addRequirements(w);
        this.triggers = triggers;
        targetPitchPosition = DifferentialWristSubsystem.pitchCenterPosition;
        targetRollPosition = DifferentialWristSubsystem.rollCenterPosition;
    }

    @Override
    public void initialize() {
        wrist.center();
    }

    @Override
    public void execute() {
        double pitchMoveMagnitude = (triggers.getButton(GamepadKeys.Button.X) ? 1.0 : 0.0) - (triggers.getButton(GamepadKeys.Button.Y) ? 1.0 : 0.0);
        double rollMoveMagnitude = (triggers.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1.0 : 0.0) - (triggers.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1.0 : 0.0);
        targetPitchPosition = Math.max(Math.min(1.0, targetPitchPosition +rollMoveMagnitude*maxMoveSpeed), 0.0); // Roll overrides roll
        targetRollPosition = Math.max(Math.min(Math.min(targetPitchPosition, 1.0- targetPitchPosition), targetRollPosition +pitchMoveMagnitude*maxMoveSpeed), Math.max(-targetPitchPosition, -1.0+ targetPitchPosition));
        wrist.setTargetPosition(targetPitchPosition, targetRollPosition);
    }

}
