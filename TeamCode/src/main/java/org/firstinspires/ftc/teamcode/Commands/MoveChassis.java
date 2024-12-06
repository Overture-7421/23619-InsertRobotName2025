package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.overture.ftc.overftclib.Contollers.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import com.overture.ftc.overftclib.Utils.OverJoystickHandler;

public class MoveChassis extends CommandBase {
    private final Chassis chassis;
    private final Gamepad driverGamepad;


    private final TrapezoidProfile leftProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.6, 999));
    private final TrapezoidProfile rightProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.6, 999));

    private TrapezoidProfile.State leftGoal = new TrapezoidProfile.State();
    private TrapezoidProfile.State rightGoal = new TrapezoidProfile.State();

    public MoveChassis(Chassis subsystem, Gamepad driverGamepad) {
        this.driverGamepad = driverGamepad;
        chassis = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        double right = -driverGamepad.right_stick_x;
        double left = -driverGamepad.left_stick_y;

        right = OverJoystickHandler.handleJoystickInput(right);
        left = OverJoystickHandler.handleJoystickInput(left);

        rightGoal = rightProfile.calculate(0.5, rightGoal, new TrapezoidProfile.State(right, 0.0));
        leftGoal = leftProfile.calculate(0.5, leftGoal, new TrapezoidProfile.State(left, 0.0));

        chassis.setSpeed(leftGoal.position, rightGoal.position);
    }
}