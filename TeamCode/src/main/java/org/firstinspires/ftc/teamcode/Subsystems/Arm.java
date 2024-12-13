package org.firstinspires.ftc.teamcode.Subsystems;
import com.overture.ftc.overftclib.Devices.IOverDcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import java.lang.Math;


public class Arm extends SubsystemBase{
    private final DcMotor armMotor;
    private final AnalogInput potentiometer;
    private final ProfiledPIDController profiledPIDController;
    public Arm ( HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5, 2.5);
        profiledPIDController = new ProfiledPIDController(0.01,0.0,0.0, constraints);

        //Tolerances
        profiledPIDController.setTolerance(1.0);
        profiledPIDController.reset(voltageToAngle(potentiometer.getVoltage()));
        profiledPIDController.enableContinuousInput(-180,180);

    }
    private double voltageToAngle(double voltage) {
        //return ((-voltage + 3.3) * 62.15) - 45;
        return (15.291 *Math.pow(this.getVoltage(), 2 )+27.952*this.getVoltage()+0.0907)-45;
    }
    public void moveToPosition(double targetAngle){
        double currentAngle= voltageToAngle(potentiometer.getVoltage());

        profiledPIDController.setGoal(targetAngle);
        double power= profiledPIDController.calculate(currentAngle);

        armMotor.setPower(power);
    }

    public boolean isAtTarget(){
        return profiledPIDController.atGoal();

    }

    public double getVoltage(){
        return -potentiometer.getVoltage() + 3.3;
    }

    public void stop(){
        armMotor.setPower(0);
    }

    public double getCurrentAngle(){
        return voltageToAngle(this.getVoltage());
    }
}
