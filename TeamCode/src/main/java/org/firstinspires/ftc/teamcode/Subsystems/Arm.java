package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;

public class Arm extends SubsystemBase {
    private final DcMotor armMotor;
    private final AnalogInput potentiometer;

    private double lastTargetAngle;
    private double lastMotorPower;

    // PID and trapezoid profile constraints
    private final ProfiledPIDController profiledPIDController;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        // Profiled PID Controller
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(200.0, 150.0);
        profiledPIDController = new ProfiledPIDController(0.065, 0.0, 0.0, constraints);

        // Tolerances
        profiledPIDController.setTolerance(0.05);
        profiledPIDController.reset(voltageToAngle(potentiometer.getVoltage()));
        profiledPIDController.enableContinuousInput(-180, 180);


    }

    //Proportional feedforward to angle
    private double feedForwardV(double neededAngle){
        return Math.cos(neededAngle*(Math.PI/180))*(8/*feedforwarth value*/) ;
    }
    private double voltageToAngle(double voltage) {
        return (15.291 * Math.pow(this.getVoltage(), 2) + 27.952 * this.getVoltage() + 0.0907) - 45;
    }

    public void moveToPosition(double targetAngle) {
        double currentAngle = voltageToAngle(potentiometer.getVoltage());
        profiledPIDController.setGoal(targetAngle);
        double power = profiledPIDController.calculate(currentAngle) + feedForwardV(currentAngle);
        armMotor.setPower(Math.max(-1.0, Math.min(1.0, power)));

        lastTargetAngle = targetAngle;
        lastMotorPower = power;
    }

    public boolean isAtTarget() {
        return profiledPIDController.atGoal();
    }

    public double getVoltage(){
        return -potentiometer.getVoltage() + 3.3;
    }

    public void stop() {
        armMotor.setPower(0);
    }

    public double getTargetAngle() {
        return lastTargetAngle;
    }

    public double getCurrentAngle() {
        return voltageToAngle(potentiometer.getVoltage());
    }

    public double getMotorPower() {
        return lastMotorPower;
    }
}