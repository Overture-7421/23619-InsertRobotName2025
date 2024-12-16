package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MoveArm extends CommandBase {

    private final Arm arm;

    private final double targetPosition;

    public MoveArm(Arm arm, double targetPosition) {
        this.arm = arm;
        this.targetPosition = targetPosition / 360;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTarget(targetPosition);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = arm.getPosition();
        return Math.abs(targetPosition - currentPosition) < 0.05;
    }
}