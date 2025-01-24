package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoToTeleopContainer;
import org.firstinspires.ftc.teamcode.backend.utilities.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.ArmPIDFController;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

@Config
public class ArmSubsystem extends SubsystemBase implements PositionControlled {

    public DcMotor motor;

    private HardwareMap hwMap;

    private PIDController PIDF;

    public static int minPosition = 0;
    public static int maxPosition = 2000;
    public static int horizPos = -600;
    public static int vertPos = 550;

    public static double kP = 0.003;
    public static double kI = 0.00005;
    public static double kD = 0.0002;
    public static double kG = 0.05; // TODO link to calculated MoI from slides
    public static double powerMultThrottle = 1.0;

    private int targetPosition;

    private int startPosition;

    public void init(ElapsedTime aTimer, HardwareMap ahwMap) {
        hwMap = ahwMap;
        motor = ahwMap.get(DcMotor.class, "ArmMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = motor.getCurrentPosition();
        AutoToTeleopContainer.getInstance().setArmPosition(startPosition);
        targetPosition = vertPos;
        PIDF = new ArmPIDFController(kP, kI, kD, aTimer, kG, horizPos, vertPos);
    }

    public void init(ElapsedTime aTimer, HardwareMap ahwMap, boolean isTeleop) {
        hwMap = ahwMap;
        motor = ahwMap.get(DcMotor.class, "ArmMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (isTeleop && AutoToTeleopContainer.getInstance().getArmPosition() != null) {
            startPosition = AutoToTeleopContainer.getInstance().getArmPosition();
        } else {
            startPosition = motor.getCurrentPosition();
            AutoToTeleopContainer.getInstance().setArmPosition(startPosition);
        }
        targetPosition = vertPos;
        PIDF = new ArmPIDFController(kP, kI, kD, aTimer, kG, horizPos, vertPos);
    }

    public double getTargetPosition() {return (targetPosition-minPosition)/(double)(maxPosition-minPosition);}

    public double getPosition() {return ((double)(motor.getCurrentPosition()-startPosition)-minPosition)/(double)(maxPosition-minPosition);}

    public double getAngleFromVert() {return ((double)(targetPosition-vertPos))/((double)(vertPos-horizPos))*Math.PI/2;}

    public void cheatIncrementStartPos() {
        startPosition += 20;
        AutoToTeleopContainer.getInstance().setArmPosition(startPosition);
    }

    public void cheatDecrementStartPos() {
        startPosition -= 20;
        AutoToTeleopContainer.getInstance().setArmPosition(startPosition);
    }

    public void setAngleFromVert(double angleRadians) {
        targetPosition = (int)(angleRadians/(Math.PI*2)*4*(vertPos-horizPos)+vertPos);
    }

    public void setTargetPosition(double target) {
        targetPosition = (int)(target * (maxPosition-minPosition) + minPosition);
    }

    public void incrementTargetPosition(double increment) {
        targetPosition += (int)(increment * (maxPosition-minPosition) + minPosition);
        targetPosition = Math.min(Math.max(targetPosition, minPosition), maxPosition);
    }

    public void vert() {
        setAngleFromVert(0.12); // Yes, this is cheating, but it makes things work out nice.
    }

    @Override
    public void periodic() {
        if (motor != null) {
            motor.setPower(Math.min(powerMultThrottle, Math.max(PIDF.update(motor.getCurrentPosition() - startPosition, targetPosition) * powerMultThrottle, -powerMultThrottle)));
        } else {
            motor = hwMap.get(DcMotor.class, "ArmMotor");
        }
    }

}