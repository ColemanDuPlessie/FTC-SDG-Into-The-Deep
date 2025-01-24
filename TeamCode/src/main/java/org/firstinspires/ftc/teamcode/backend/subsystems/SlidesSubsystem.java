package org.firstinspires.ftc.teamcode.backend.subsystems;

import static org.firstinspires.ftc.teamcode.backend.Robot19397.tele;

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
    public static int maxPosition = 3750; // TODO
    public static int hangPosition = 2800;

    public static double minLength = 13.75;
    public static double maxLength = 39.5;
    public static double endEffectorInset = 4.0;
    public static double maxRearExtension = 12.0;
    public static double maxFrontExtension = 23.0; // These two values are in inches must sum to <42. 4 in of wiggle room should be enough to account for width and imperfect control systems

    public static double kP = 0.005; // TODO tune this
    public static double kI = 0.0000;
    public static double kD = 0.00008;
    public static double kG = 0.25;
    public static double maxPower = 1.0;

    private int targetPosition;

    private int startPosition;

    private ArmSubsystem linkedArm;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, ArmSubsystem a) {
        leftMotor = ahwMap.get(DcMotor.class, "LeftSlidesMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = ahwMap.get(DcMotor.class, "RightSlidesMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = leftMotor.getCurrentPosition();
        targetPosition = minPosition;
        linkedArm = a;
        PIDF = new PIDController(kP, kI, kD, aTimer);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop, ArmSubsystem a) {
        leftMotor = ahwMap.get(DcMotor.class, "LeftSlidesMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        targetPosition = minPosition;
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

    public double getEffectiveLength() { // Only considers end effector position
        return minLength-endEffectorInset+(maxLength-minLength)*getTargetPosition();
    }

    @Override
    public void periodic() {
        double maxTargetExtension = 9999;
        if (linkedArm.getAngleFromVert() > 0.0) { // Slides are facing forward
            maxTargetExtension = 1.0/Math.sin(linkedArm.getAngleFromVert())*maxFrontExtension;
        } else if (linkedArm.getAngleFromVert() < 0.0) { // Slides are facing backwards
            maxTargetExtension = 1.0/Math.sin(-linkedArm.getAngleFromVert())*maxRearExtension;
        }
        double maxPos = (maxTargetExtension-minLength) / (maxLength-minLength);
        tele.addData("Max possible extension", maxTargetExtension);
        tele.addData("(Arm degrees from vertical)", linkedArm.getAngleFromVert()/Math.PI*180);
        tele.addData("Max slide position", maxPos);
        tele.addData("Current slide position", getTargetPosition());
        if (getTargetPosition() > maxPos) {
            setTargetPosition(maxPos);
        }
        double actualPower = Math.min(maxPower, Math.max(PIDF.update(leftMotor.getCurrentPosition()-startPosition, targetPosition) * maxPower, -maxPower)) + kG * Math.cos(linkedArm.getAngleFromVert());
        leftMotor.setPower(actualPower);
        rightMotor.setPower(actualPower);
    }

}
