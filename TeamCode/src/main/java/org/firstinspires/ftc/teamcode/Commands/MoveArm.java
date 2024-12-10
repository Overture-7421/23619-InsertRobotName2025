package org.firstinspires.ftc.teamcode.Commands;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

public class MoveArm extends CommandBase {
    private final Arm arm;
    private final double targetPos;

    public MoveArm(Arm arm, double targetPos){
        this.arm=arm;
        this.targetPos=targetPos/360;
        addRequirements(arm);
    }
    @Override
    public void initialize(){
        arm.setTarget(targetPos);
    }

    @Override
    public boolean isFinished(){
        double currentPosition = arm.getPosition() ;
        return Math.abs(targetPos-currentPosition)<0.05;
    }

}
