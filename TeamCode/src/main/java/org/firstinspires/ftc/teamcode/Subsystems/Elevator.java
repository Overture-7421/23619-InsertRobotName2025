package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator extends SubsystemBase {
    private DcMotorEx elevatorMotor;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator_Motor");
        invertMotor();
    }

    // Motor Inversion (if needed)
    public void invertMotor() {
        elevatorMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setSpeed(double speed) {
        elevatorMotor.setPower(speed);
    }


    public String getPosition() {
        return "Elevator Position: " + elevatorMotor.getCurrentPosition();
    }
}
