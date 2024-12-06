package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator extends SubsystemBase {
    private final Motor viperSlideMotor;
    private static final int SLIDE_MIN_POSITION = 0;        // Fully retracted
    private static final int SLIDE_MAX_POSITION = 100;     // Fully extended
    private static final double SLIDE_POWER = 0.5;

    // Constructor
    public Elevator(HardwareMap hardwareMap) {
        // Initialize the viperSlideMotor using the HardwareMap
        this.viperSlideMotor = new Motor(hardwareMap, "elevator_Motor");

        // Reset encoder and configure motor
        this.viperSlideMotor.resetEncoder();
        this.viperSlideMotor.setRunMode(Motor.RunMode.PositionControl);
    }

    public void moveToPosition(int targetPosition) {
        // Clamp the target position to valid range
        targetPosition = Math.max(SLIDE_MIN_POSITION, Math.min(SLIDE_MAX_POSITION, targetPosition));

        // Set the target position
        viperSlideMotor.setTargetPosition(targetPosition);
        viperSlideMotor.set(SLIDE_POWER);
    }

    public void stop() {
        viperSlideMotor.stopMotor();
    }

    public boolean isAtTarget() {
        return viperSlideMotor.atTargetPosition();
    }
}
