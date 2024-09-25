package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class ControlWrist extends CommandBase {

    public double targetPosition;
    private double maxMoveSpeed = 0.002;

    private WristSubsystem wrist;
    private final GamepadEx triggers;


    public ControlWrist(WristSubsystem w, GamepadEx triggers, GamepadButton resetButton) {
        wrist = w;
        addRequirements(w);
        this.triggers = triggers;
        resetButton.whenReleased(() -> this.resetPosition());
        targetPosition = WristSubsystem.centerPosition;
    }

    public void resetPosition() {
        wrist.center();
        targetPosition = WristSubsystem.centerPosition;
    }

    @Override
    public void initialize() {
        wrist.center();
    }

    @Override
    public void execute() {
        double move_magnitude = triggers.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - triggers.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        targetPosition = Math.max(Math.min(1.0, targetPosition+move_magnitude*maxMoveSpeed), -1.0);
        wrist.setTargetPosition(targetPosition);
    }

}
