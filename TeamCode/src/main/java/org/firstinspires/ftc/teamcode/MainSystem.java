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


        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            double targetAngle = arm.getTargetAngle();
            double currentAngle = arm.getCurrentAngle();
            double motorPower = arm.getMotorPower();

            // Telemetry for subsystems
            telemetry.addData("Chassis Pose", chassis.getPose());
            telemetry.addData("Elevator Position", elevator.getPosition());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();


            /* ARM OPERATION */
            Button operatorButtonDPAD_UP = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorButtonDPAD_UP.whenPressed(new MoveArm(arm, 45.0));

            Button operatorButtonDPAD_DOWN = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            operatorButtonDPAD_DOWN.whenPressed(new MoveArm(arm, -45.0));

            Button operatorButtonDPAD_LEFT = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            operatorButtonDPAD_LEFT.whenPressed(new MoveArm(arm, 0.0));


            /* INTAKE OPERATION */
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
