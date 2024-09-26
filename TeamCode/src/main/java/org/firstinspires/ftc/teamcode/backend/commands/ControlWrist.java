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
    private double maxMoveSpeed = 0.01;

    private WristSubsystem wrist;
    private final GamepadEx triggers;


    public ControlWrist(WristSubsystem w, GamepadEx triggers) {
        wrist = w;
        addRequirements(w);
        this.triggers = triggers;
        targetPosition = WristSubsystem.centerPosition;
    }

    @Override
    public void initialize() {
        wrist.center();
    }

    @Override
    public void execute() {
        double move_magnitude = (triggers.getButton(GamepadKeys.Button.X) ? 1.0 : 0.0) - (triggers.getButton(GamepadKeys.Button.Y) ? 1.0 : 0.0);
        targetPosition = Math.max(Math.min(1.0, targetPosition+move_magnitude*maxMoveSpeed), 0.0);
        wrist.setTargetPosition(targetPosition);
    }

}
