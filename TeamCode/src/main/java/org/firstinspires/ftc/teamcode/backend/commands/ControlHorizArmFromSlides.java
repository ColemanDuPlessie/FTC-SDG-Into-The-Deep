package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DifferentialWristSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class ControlHorizArmFromSlides extends CommandBase {

    public static double targetVertDist = 2.5; // Distance above horizontal, in inches.
    public static double maxRadsPerSec = 2.0;

    private ArmSubsystem arm;
    private SlidesSubsystem slides;
    private ElapsedTime timer;

    private double prevTime;

    public ControlHorizArmFromSlides(ArmSubsystem a, SlidesSubsystem s, ElapsedTime t) {
        arm = a;
        addRequirements(a);
        slides = s; // We never use the slides, only read from them.
        timer = t;
    }

    @Override
    public void initialize() {
        prevTime = timer.milliseconds();
        execute();
    }

    @Override
    public void execute() {
        double slidesLength = slides.getEffectiveLength();
        double targetAngle = Math.PI/2-Math.asin(targetVertDist/slidesLength);
        double prevAngle = arm.getAngleFromVert();
        double time = timer.milliseconds();
        double delta = time-prevTime;
        double angle = Math.min(Math.max(arm.getAngleFromVert()-delta*maxRadsPerSec/1000, targetAngle), arm.getAngleFromVert()+delta*maxRadsPerSec/1000);
        arm.setAngleFromVert(angle);
        prevTime = time;
    }

}
