package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;

@Config
public class ClawSubsystem extends SubsystemBase implements PositionControlled {

    public ServoImpl servo;

    public static double openPos = 0.65;
    public static double closedPos = 0.90;
    public static double waitingPos = 0.85;

    private double targetPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "ClawServo");
        setTargetPosition(closedPos);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        this.init(aTimer, ahwMap);
    }

    public double getPosition() {return servo.getPosition();}
    public double getTargetPosition() {return targetPosition;}
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        servo.setPosition(targetPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += increment;
        servo.setPosition(targetPosition);
    }

    public void close() {setTargetPosition(closedPos);}
    public void open() {setTargetPosition(openPos);}
    public void waiting() {setTargetPosition(waitingPos);}

    public void cycle() {
        if (targetPosition == closedPos) {
            open();
        } else if (targetPosition == openPos) {
            waiting();
        } else {
            close();
        }
    }

}
