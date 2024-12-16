package org.firstinspires.ftc.teamcode.Subsystems;
import com.overture.ftc.overftclib.Contollers.ProfiledPIDController;
import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final DcMotorEx elevatorMotor;
    private ProfiledPIDController elevatorMotorPID;
    public static final double TICKS_PER_REVOLUTION = 753.2;
    public static final double ELEVATOR_WINCH_CIRCUMFERENCE = 12.0008738;
    // In Meters diameter: 3.82 cm
    public static final double GEAR_REDUCTION = 26.9;

    private double motorOffset = 0.0;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevator_Motor");

        elevatorMotorPID = new ProfiledPIDController(1.0,0,0, new TrapezoidProfile.Constraints(100.0,80.0));

        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetZero();
        elevatorMotorPID.reset(getHeight());
        elevatorMotorPID.setGoal(getHeight());

    }

    public void resetZero() {
    motorOffset = elevatorMotor.getCurrentPosition();
    }

    public double getHeight() {
        double elevatorMotorTicks = elevatorMotor.getCurrentPosition();
        double elevatorMotorCurrentHeight = elevatorMotorTicks * ((ELEVATOR_WINCH_CIRCUMFERENCE/TICKS_PER_REVOLUTION));
        return elevatorMotorCurrentHeight;
    }

    public void setGoal(double goalHeight) {
        if(elevatorMotorPID.getGoal().position != goalHeight) {
            elevatorMotorPID.reset(getHeight());
            elevatorMotorPID.setGoal(goalHeight);
        }
    }


    //Periodic actions used for positional Elevator
    @Override
    public void periodic() {
        double outputMotor = elevatorMotorPID.calculate(getHeight());
        elevatorMotor.setPower(outputMotor);

        if(getHeight() > 97){
            elevatorMotor.setPower(0.0);
        }

        if(getHeight() < -0.1){
            elevatorMotor.setPower(0.0);
        }
    }
}
