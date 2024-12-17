package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.arcrobotics.ftclib.util.MathUtils;

public class Arm extends SubsystemBase {

    private DcMotorEx motor;
    private ProfiledPIDController armPID;

    public static final double COUNTS_PER_REV = 8192;

    private static final double OFFSET = 45;

    public Arm(HardwareMap hardwareMap) {
        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_Motor");

        armPID = new ProfiledPIDController(0.11, 0, 0.0, new TrapezoidProfile.Constraints(30.0, 20.0));

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPID.reset(getPosition());
        armPID.setGoal(getPosition());
    }

    private double armFeedForward(double angle){
        return ((Math.cos(Math.toRadians(angle))) * 0.373);
    }

    public double getPosition() {
        double currentTicks = motor.getCurrentPosition();
        return ((currentTicks / COUNTS_PER_REV) * 360 - OFFSET);
    }

    public void setTarget(double targetHeight) {
        if (armPID.getGoal().position != targetHeight) {
            armPID.reset(getPosition());
            armPID.setGoal(targetHeight);
        }
    }
    @Override
    public void periodic() {
        double motorOutput = armPID.calculate(getPosition());
        motor.setPower(motorOutput + armFeedForward(getPosition()));

    }
}