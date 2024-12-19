package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.overture.ftc.overftclib.Contollers.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm extends SubsystemBase {

    private final DcMotorEx motor;
    private final PIDController armPID;
    private final Telemetry telemetry;

    public static final double COUNTS_PER_REV = 8192;
    private static final double OFFSET = 47;
    private double target = -47;
    private static final double ff = 0.175;
    //public static double p = 0.0;

    public Arm(HardwareMap hardwareMap) {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, "arm_Motor");

        armPID = new PIDController(0.045, 0, 0.0);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double armFeedForward(double angle){
        return ((Math.cos(Math.toRadians(angle))) * ff);

    }

    public double getPosition() {
        double currentTicks = motor.getCurrentPosition();
        return ((currentTicks / COUNTS_PER_REV) * 360 - OFFSET);
    }


    public void setTarget(double targetPos) {
        target = targetPos;

    }
    @Override
    public void periodic() {
        double motorOutput = armPID.calculate(getPosition(), target);
        motor.setPower(motorOutput + armFeedForward(getPosition()));
        telemetry.addData("Arm Output", motorOutput);
        telemetry.addData("Arm Position", getPosition());
        telemetry.addData("Arm Target", target);


    }
}