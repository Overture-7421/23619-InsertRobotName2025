package org.firstinspires.ftc.teamcode.Subsystems;
import com.overture.ftc.overftclib.Devices.IOverDcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
public class Arm extends SubsystemBase {
    private DcMotorEx arm_Motor;
    private ProfiledPIDController armPID;
    public static final double COUNTS_PER_REV = 384.5;
    public static final double GEAR_RATIO= 13.7;
    private double ARM_OFFSET = 46;

    public Arm (HardwareMap hardwareMap){
       arm_Motor = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "arm_Motor");
       armPID = new ProfiledPIDController(0.0,0.0,0.0,
               new TrapezoidProfile.Constraints (0.0,0.0));

       arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       armPID.reset(getPosition());
       armPID.setGoal(getPosition());

    }

    public void resetZero(){
        ARM_OFFSET= arm_Motor.getCurrentPosition();
    }

    public double getPosition(){
        double currentTicks = arm_Motor.getCurrentPosition();
        double currentPosition = (currentTicks/COUNTS_PER_REV*GEAR_RATIO)-(ARM_OFFSET/360);
        return currentPosition;
    }

    public void setTarget(double targetPos){
        if (armPID.getGoal().position != targetPos){
            armPID.reset(getPosition());
            armPID.setGoal(targetPos);
        }
    }

    @Override
    public void periodic(){
        double motorOutput = armPID.calculate(getPosition());
        arm_Motor.setPower(motorOutput);
    }


}
