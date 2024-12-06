package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class MoveElevator extends CommandBase {

    private final Elevator elevatorSubsystem;
    private final int targetPosition;

    public MoveElevator(Elevator elevatorSubsystem, int targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;

        // Declare subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        // Move the elevator to the target position
        elevatorSubsystem.moveToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the elevator reaches the target position
        return elevatorSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator when the command ends
        elevatorSubsystem.stop();
    }
}
