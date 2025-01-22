package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DifferentialWristSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class ControlHorizArmFromSlides extends CommandBase {

    private double targetVertDist = 4.0; // Distance above horizontal, in inches.

    private ArmSubsystem arm;
    private SlidesSubsystem slides;


    public ControlHorizArmFromSlides(ArmSubsystem a, SlidesSubsystem s) {
        arm = a;
        addRequirements(a);
        slides = s; // We never use the slides, only read from them.
    }

    @Override
    public void initialize() {
        execute();
    }

    @Override
    public void execute() {
        double slidesLength = slides.getEffectiveLength();
        double targetAngle = Math.asin(targetVertDist/slidesLength);
        arm.setAngleFromVert(Math.PI/2-targetAngle);
    }

}
