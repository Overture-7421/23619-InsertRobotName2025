package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Commands Import
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.Commands.MoveElevatorJoystick;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;

// Subsystems Import

import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;

@TeleOp
public class MainSystem extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Reset and prepare CommandScheduler
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        // Initialize subsystems
        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Gamepad bindings
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        // Set default commands
        chassis.setDefaultCommand(new MoveChassis(chassis, gamepad1));
        elevator.setDefaultCommand(new MoveElevatorJoystick(elevator, gamepad2));

        // Define arm positions (update these with actual voltage values for your setup)
        double groundPosition = 0.5; // Example voltage
        double midPosition = 1.5;    // Example voltage
        double highPosition = 2.5;   // Example voltage

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Telemetry for subsystems
            telemetry.addData("Chassis Pose", chassis.getPose());
            telemetry.addData("Elevator Position", elevator.getPosition());
            telemetry.addData("Arm Potentiometer Voltage", arm.getPotentiometerVoltage());
            telemetry.update();

            // Control the arm with gamepad buttons
            if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                CommandScheduler.getInstance().schedule(new MoveArm(arm, groundPosition));
            } else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                CommandScheduler.getInstance().schedule(new MoveArm(arm, midPosition));
            } else if (operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
                CommandScheduler.getInstance().schedule(new MoveArm(arm, highPosition));
            }

            Button operatorButtonY= operatorGamepad.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenHeld(new MoveIntake(intake,1.0));
            operatorButtonY.whenReleased(new MoveIntake(intake,0.0));

            Button operatorButtonB= operatorGamepad.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenHeld(new MoveIntake(intake,-1.0));
            operatorButtonB.whenReleased(new MoveIntake(intake,0.0));

            // Run CommandScheduler to execute commands
            CommandScheduler.getInstance().run();
        }
    }
}
