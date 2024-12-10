package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private final DcMotor armMotor;
    private final AnalogInput potentiometer;

    // Predefined positions for the arm (in potentiometer voltage or mapped values)
    private final double GROUND_POSITION = 0.5;  // Example voltage for ground level
    private final double MID_POSITION = 1.5;     // Example voltage for mid level
    private final double HIGH_POSITION = 2.5;    // Example voltage for high level

    // PID constants (tune these for your specific setup)
    private final double kP = 0.5;
    private final double kI = 0.0;
    private final double kD = 0.1;

    private double integral = 0.0;
    private double previousError = 0.0;

    public Arm (HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToPosition(double targetVoltage) {
        double currentVoltage = potentiometer.getVoltage();
        double error = targetVoltage - currentVoltage;

        integral += error;
        double derivative = error - previousError;

        double power = kP * error + kI * integral + kD * derivative;

        armMotor.setPower(power);

        previousError = error;
    }

    public void stop() {
        armMotor.setPower(0);
    }

    public void moveToGround() {
        moveToPosition(GROUND_POSITION);
    }

    public void moveToMid() {
        moveToPosition(MID_POSITION);
    }

    public void moveToHigh() {
        moveToPosition(HIGH_POSITION);
    }

    public double getPotentiometerVoltage() {
        return potentiometer.getVoltage();
    }

}
