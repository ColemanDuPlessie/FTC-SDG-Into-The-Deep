package org.firstinspires.ftc.teamcode.backend.subsystems;

import static org.firstinspires.ftc.teamcode.backend.Robot19397.tele;

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
    public static double pitchCenterPosition = 0.0;

    private double targetRollPosition;
    private double targetPitchPosition;

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

    public double getRollPosition() {return targetRollPosition;}
    public double getPitchPosition() {return targetPitchPosition;}

    public void setTargetPosition(double roll, double pitch) {
        targetRollPosition = roll;
        targetPitchPosition = pitch;
        if (tele != null) {
            tele.addData("Left wrist servo pos", targetRollPosition-targetPitchPosition);
            tele.addData("Right wrist servo pos", 1.0-(targetRollPosition+targetPitchPosition));
        }
        leftServo.setPosition(targetRollPosition-targetPitchPosition);
        rightServo.setPosition(1.0-(targetRollPosition+targetPitchPosition));
    }

    public void center() {setTargetPosition(rollCenterPosition, pitchCenterPosition);}

    public void initPos() {setTargetPosition(rollCenterPosition, pitchCenterPosition-0.2);}

}
