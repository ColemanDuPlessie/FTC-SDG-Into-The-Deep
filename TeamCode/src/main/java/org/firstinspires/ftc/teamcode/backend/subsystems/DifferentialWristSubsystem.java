package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;

@Config
public class DifferentialWristSubsystem extends SubsystemBase {

    public ServoImpl leftServo;
    public ServoImpl rightServo;

    public static double rollCenterPosition = 0.50;
    public static double pitchCenterPosition = 0.50;

    private double targetRollPosition;
    private double targetPitchPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        leftServo = ahwMap.get(ServoImpl.class, "LeftWristServo");
        rightServo = ahwMap.get(ServoImpl.class, "RightWristServo");
        center();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        leftServo = ahwMap.get(ServoImpl.class, "LeftWristServo");
        rightServo = ahwMap.get(ServoImpl.class, "RightWristServo");
        center();
    }

    public double getRollPosition() {return targetRollPosition;}
    public double getPitchPosition() {return targetPitchPosition;}

    public void setTargetPosition(double roll, double pitch) {
        targetRollPosition = roll;
        targetPitchPosition = pitch;
        leftServo.setPosition(targetPitchPosition-targetRollPosition);
        rightServo.setPosition(1-(targetPitchPosition+targetRollPosition));
    }

    public void center() {setTargetPosition(rollCenterPosition, pitchCenterPosition);}


}
