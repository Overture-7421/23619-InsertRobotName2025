package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Commands Import
import org.firstinspires.ftc.teamcode.Commands.MoveElevator;
import org.firstinspires.ftc.teamcode.Commands.StopElevator;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;

// Subsystems Import
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;

@TeleOp
public class MainSystem extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);

        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        GamepadButton moveElevatorUp = new GamepadButton(driverGamepad, GamepadKeys.Button.X);
        //GamepadButton moveElevatorDown = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        GamepadButton stopElevator = new GamepadButton(driverGamepad, GamepadKeys.Button.B);

        moveElevatorUp.whenPressed(new MoveElevator(elevator, 100));
        //moveElevatorDown.whenPressed(new MoveElevator(elevator, 0));
        stopElevator.whenPressed(new StopElevator(elevator));

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
