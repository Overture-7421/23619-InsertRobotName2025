package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;



public class BasketPos extends SequentialCommandGroup{
    public BasketPos(Arm arm, Elevator elevator, double armAngle , double elevatorDistance){

                addCommands(
                        new MoveArm(arm, armAngle),
                        new WaitCommand(500),
                        new ElevatorPositions(elevator, elevatorDistance)

                );

            }
        }



