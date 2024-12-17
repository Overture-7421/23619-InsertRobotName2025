package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Commands.BasketPos;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;
import org.firstinspires.ftc.teamcode.Commands.MoveArm;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import com.acmerobotics.ftclib.command.ftcDaschboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@TeleOp
public class MainSystem extends LinearOpMode {
    @Override
        public void runOpMode(){
            Chassis chassis = new Chassis(hardwareMap); //Here you can add every element of the robot
            Intake intake = new Intake(hardwareMap);
            Arm arm = new Arm(hardwareMap);
            GamepadEx driver = new GamepadEx(gamepad1);
            GamepadEx operator = new GamepadEx(gamepad2);
            Elevator elevator = new Elevator(hardwareMap);

        /*------------------------------------------------------------------*/
        /*------------------------------------------------------------------*/

           /* chassis.setDefaultCommand(new Drive(chassis,gamepad1)); */

           /* Button operatorButtonX= operator.getGamepadButton(GamepadKeys.Button.X);
            operatorButtonX.whenHeld(new MoveIntake(intake,1.0));
            operatorButtonX.whenReleased(new MoveIntake(intake,0.0));

            Button operatorButtonB= operator.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenHeld(new MoveIntake(intake,-1.0));
            operatorButtonB.whenReleased(new MoveIntake(intake,0.0));*/

            /*Button operatorButtonA= operator.getGamepadButton(GamepadKeys.Button.A);
            operatorButtonA.whenHeld(new MoveArm(arm,30));
            operatorButtonA.whenReleased(new MoveArm(arm,30));*/

            Button operatorButtonY= operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenPressed(new ElevatorPositions(elevator,20.0));

        Button operatorButtonX= operator.getGamepadButton(GamepadKeys.Button.X);
        operatorButtonX.whenPressed(new ElevatorPositions(elevator,0));

        /*Button operatorButtonA=operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new BasketPos(arm, elevator, 0, 0));*/

        Button operatorButtonA=operator.getGamepadButton(GamepadKeys.Button.A);
        operatorButtonA.whenPressed(new BasketPos(arm, elevator, Constants.Arm.ARMHIGHCHAMBER, Constants.Elevator.ELEVATORHIGHCHAMBER));


        waitForStart();
            chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0))); /*When the Op mode starts,
                                                                every value will return to be zero*/

            while (opModeIsActive()) { //This will occur whenever the op mode is active
                CommandScheduler.getInstance().run();
                Pose2d pose = chassis.getPose();

                // -- ODOMETRY TELEMETRY -- //
                /*telemetry.addData("X", pose.getX()); //This will display the telemetry on the DriverHub
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", pose.getRotation().getDegrees());
                telemetry.addData("RightDistance", chassis.rightDistance());
                telemetry.addData("LeftDistance", chassis.leftDistance());
                telemetry.addData("Potentiometer voltage", arm.getVoltage());*/
                telemetry.addData("Elevator_Distance", elevator.getHeight());

                // -- UPDATE TELEMETRY -- //
                telemetry.update();
            }
    }
}
