package org.firstinspires.ftc.teamcode.backend.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

@Config
public class RetractHang extends CommandBase {

    public static double finalPos = 0.3;
    public static long travelDuration = 2000;

    private SlidesSubsystem slides;
    private ElapsedTime timer;

    private boolean isRunning = false;
    private long startMillis;
    private double startPos;

    public RetractHang(SlidesSubsystem s, ElapsedTime timer) {
        slides = s;
        addRequirements(s);
        this.timer = timer;
    }

    @Override
    public void initialize() {
        if (slides.getTargetPosition() > finalPos) {
            isRunning = true;
            startMillis = (long) timer.milliseconds();
            startPos = slides.getTargetPosition();
        } else {
            isRunning = false;
        }
    }

    @Override
    public void execute() {
        long totalElapsedTime = (long)timer.milliseconds()-startMillis;
        if (!isRunning) {
            return;
        }
        double currentTarget = startPos-((double)totalElapsedTime)/((double)travelDuration)*(startPos-finalPos);
        if (currentTarget < finalPos) {
            slides.setTargetPosition(finalPos);
            isRunning = false;
        } else {
            slides.setTargetPosition(currentTarget);
        }
    }

    @Override
    public boolean isFinished() { return !isRunning; }
}
