package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MoveArm extends CommandBase {
    private final Arm armSubsystem;
    private final double targetPosition;

    public MoveArm(Arm armSubsystem, double targetPosition) {
        this.armSubsystem = armSubsystem;
        this.targetPosition = targetPosition;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // No initialization needed
    }

    @Override
    public void execute() {
        armSubsystem.moveToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Stop when the arm is close enough to the target position
        double currentVoltage = armSubsystem.getPotentiometerVoltage();
        return Math.abs(targetPosition - currentVoltage) < 0.05; // Adjust tolerance as needed
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}
