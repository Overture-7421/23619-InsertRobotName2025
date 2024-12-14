package org.firstinspires.ftc.teamcode;

/* GENERAL IMPORTED LIBRARIES */
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* COMMANDS IMPORT */
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Commands.MoveChassis;
import org.firstinspires.ftc.teamcode.Commands.MoveElevatorJoystick;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;

/* SUBSYSTEMS IMPORT */
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
        /* RESET AND CANCEL STORED COMMANDS */
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        /* INITIALIZE SUBSYSTEMS */
        Chassis chassis = new Chassis(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        /* GAMEPAD DECLARATIONS */
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        GamepadEx operatorGamepad = new GamepadEx(gamepad2);

        /* CHASSIS COMMAND OPERATION */
        chassis.setDefaultCommand(new MoveChassis(chassis, gamepad1));

        /* WAIT FOR START*/
        waitForStart();

        /* MAIN LOOP */
        while (opModeIsActive()) {

            /* TELEMETRY */
            telemetry.addData("Chassis Pose", chassis.getPose());
            telemetry.addData("Target Angle", arm.getTargetAngle());
            telemetry.addData("Current Angle", arm.getCurrentAngle());
            telemetry.addData("Motor Power", arm.getMotorPower());
            telemetry.update();


            /* ARM OPERATION */
            Button operatorButtonDPAD_UP = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
            operatorButtonDPAD_UP.whenPressed(new MoveArm(arm, 20.0));

            Button operatorButtonDPAD_DOWN = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
            operatorButtonDPAD_DOWN.whenPressed(new MoveArm(arm, -40.0));

            Button operatorButtonDPAD_LEFT = operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
            operatorButtonDPAD_LEFT.whenPressed(new MoveArm(arm, 0.0));

            /* ELEVATOR OPERATION */
            Button operatorButtonY= operatorGamepad.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new ElevatorPositions(elevator,20.0));

            Button operatorButtonX= operatorGamepad.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenPressed(new ElevatorPositions(elevator,0.0));

            /* INTAKE OPERATION */
            Button operatorButtonA= operatorGamepad.getGamepadButton(GamepadKeys.Button.A);
            operatorButtonA.whenHeld(new MoveIntake(intake,1.0));
            operatorButtonA.whenReleased(new MoveIntake(intake,0.0));

            Button operatorButtonB= operatorGamepad.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenHeld(new MoveIntake(intake,-1.0));
            operatorButtonB.whenReleased(new MoveIntake(intake,0.0));

            /* COMMAND SCHEDULER TO RUN COMMANDS */
            CommandScheduler.getInstance().run();
        }
    }
}
