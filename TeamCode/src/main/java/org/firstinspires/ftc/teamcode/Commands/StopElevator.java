package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class StopElevator extends CommandBase {

    private final Elevator elevatorSubsystem; // Reference to Elevator subsystem

    public StopElevator(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Calls the stop method in the Elevator subsystem
        elevatorSubsystem.stop();
    }
}
