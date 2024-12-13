package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MoveArm extends CommandBase {
    private final Arm armSubsystem;
    private final double targetAngle;

    public MoveArm(Arm armSubsystem, double targetAngle) {
        this.armSubsystem = armSubsystem;
        this.targetAngle = targetAngle;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.moveToPosition(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}
