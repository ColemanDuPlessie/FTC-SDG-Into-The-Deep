package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;

@Config
public class WristSubsystem extends SubsystemBase implements PositionControlled {

    public ServoImpl servo;

    public static double centerPosition = 0.50;

    private double targetPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        center();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        servo = ahwMap.get(ServoImpl.class, "WristServo");
        center();
    }

    public double getTargetPosition() {return targetPosition;}

    public double getPosition() {return servo.getPosition();}

    public void setTargetPosition(double target) {
        targetPosition = target;
        servo.setPosition(targetPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += increment;
        servo.setPosition(targetPosition);
    }

    public void center() {setTargetPosition(centerPosition);}


}
