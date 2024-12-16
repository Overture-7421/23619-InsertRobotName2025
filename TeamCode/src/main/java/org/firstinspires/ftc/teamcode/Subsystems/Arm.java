package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.arcrobotics.ftclib.controller.PIDFController;

public class Arm extends SubsystemBase {

    private DcMotorEx motor;
    private ProfiledPIDController armPID;

    public static final double COUNTS_PER_REV = 28.0;

    public static final double MOTOR_GEAR_RATIO = 0.128;

    private double motorOffset = 48.0;

    public Arm(HardwareMap hardwareMap) {

        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_Motor");

        armPID = new ProfiledPIDController(0.01, 0, 0.0, new TrapezoidProfile.Constraints(3, 2));

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //resetZero();

        armPID.reset(getPosition());
        armPID.setGoal(getPosition());
    }

    public void resetZero() {
        motorOffset = motor.getCurrentPosition();
    }

    public double getPosition() {
        double currentTicks = motor.getCurrentPosition();
        double currentPosition = (currentTicks / (COUNTS_PER_REV * MOTOR_GEAR_RATIO))  - (motorOffset/360.0);
        return currentPosition;
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
        motor.setPower(motorOutput);

    }
}