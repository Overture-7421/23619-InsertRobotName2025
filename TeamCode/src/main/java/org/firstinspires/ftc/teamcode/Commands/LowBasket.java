package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;


package org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

public class LowBasket extends SequentialCommandGroup {

    public void LowBasket(Arm arm, Elevator elevator){
        addCommands(

                new MoveArm(arm, 1.0),
                new WaitCommand(500),
                new MoveElevator(elevator, 1.0)

        );


    }

}
