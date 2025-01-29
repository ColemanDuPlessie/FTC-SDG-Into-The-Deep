package org.firstinspires.ftc.teamcode.backend.subsystems;

import static org.firstinspires.ftc.teamcode.backend.Robot19397.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DifferentialWristSubsystem extends SubsystemBase {

    public ServoImpl leftServo;
    public ServoImpl rightServo;

    public static double pitchCenterPosition = 0.50;
    public static double rollCenterPosition = 0.0;

    private double targetPitchPosition;
    private double targetRollPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        leftServo = ahwMap.get(ServoImpl.class, "LeftWristServo");
        rightServo = ahwMap.get(ServoImpl.class, "RightWristServo");
        initPos();
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        leftServo = ahwMap.get(ServoImpl.class, "LeftWristServo");
        rightServo = ahwMap.get(ServoImpl.class, "RightWristServo");
        initPos();
    }

    public double getPitchPosition() {return targetPitchPosition;}
    public double getRollPosition() {return targetRollPosition;}

    public void setTargetPosition(double pitch, double roll) {
        targetPitchPosition = pitch;
        targetRollPosition = roll;
        if (tele != null) {
            tele.addData("Left wrist servo pos", targetPitchPosition - targetRollPosition);
            tele.addData("Right wrist servo pos", 1.0-(targetPitchPosition + targetRollPosition));
        }
        leftServo.setPosition(targetPitchPosition - targetRollPosition);
        rightServo.setPosition(1.0-(targetPitchPosition + targetRollPosition));
    }

    public void center() {setTargetPosition(pitchCenterPosition, rollCenterPosition);}

    public void initPos() {setTargetPosition(pitchCenterPosition + 0.2, rollCenterPosition);}

}
