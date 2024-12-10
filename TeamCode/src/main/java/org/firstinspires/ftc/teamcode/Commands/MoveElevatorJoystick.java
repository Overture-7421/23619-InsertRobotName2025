package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Utils.JoystickHandler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class MoveElevatorJoystick extends CommandBase {
    private final Elevator elevator;
    private final Gamepad operatorGamepad;

    public MoveElevatorJoystick(Elevator subsystem, Gamepad driverGamepad) {
        this.operatorGamepad = driverGamepad;
        elevator = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double elevatorSpeed = operatorGamepad.left_stick_y;

        elevatorSpeed = JoystickHandler.handleJoystickInput(elevatorSpeed);

        elevator.setSpeed(elevatorSpeed);
    }
}