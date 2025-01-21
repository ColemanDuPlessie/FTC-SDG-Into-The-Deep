package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class SlidesSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    private PIDController PIDF;

    public static int minPosition = -60; // Added to ensure complete retraction
    public static int maxPosition = 3750; // TODO this isn't all the way up for a hang
    public static int hangPosition = 2800; // TODO

    public static double kP = 0.007; // TODO tune this
    public static double kI = 0.0000;
    public static double kD = 0.000065;
    public static double kG = 0.25;
    public static double maxPower = 0.6; // TODO this is extra gentle

    private int targetPosition;

    private int startPosition;

    private ArmSubsystem linkedArm;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, ArmSubsystem a) {
        leftMotor = ahwMap.get(DcMotor.class, "LeftSlidesMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = ahwMap.get(DcMotor.class, "RightSlidesMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = leftMotor.getCurrentPosition();
        targetPosition = 0;
        linkedArm = a;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop, ArmSubsystem a) {
        leftMotor = ahwMap.get(DcMotor.class, "LeftSlidesMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = ahwMap.get(DcMotor.class, "RightSlidesMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop) {
            Integer position = AutoToTeleopContainer.getInstance().getSlidesPosition();
            if (position == null) {
                startPosition = leftMotor.getCurrentPosition();
            } else { startPosition = position;}
        } else {
            startPosition = leftMotor.getCurrentPosition();
            AutoToTeleopContainer.getInstance().setSlidesPosition(startPosition);
        }
        targetPosition = 0;
        linkedArm = a;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public double getPosition() {return ((double)(leftMotor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getTargetPosition() {return ((double)(targetPosition-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition));
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    public void hang() {targetPosition = hangPosition;} // This is the only way to go above maxPosition.

    @Override
    public void periodic() {
        double actualPower = Math.min(maxPower, Math.max(PIDF.update(leftMotor.getCurrentPosition()-startPosition, targetPosition) * maxPower, -maxPower)) + kG; // * Math.cos(linkedArm.getAngleFromVert());
        leftMotor.setPower(actualPower);
        rightMotor.setPower(actualPower);
    }

}
