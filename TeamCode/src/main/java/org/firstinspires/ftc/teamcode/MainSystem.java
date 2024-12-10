package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.Commands.MoveIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Button;


@TeleOp
public class MainSystem extends LinearOpMode {
    @Override
        public void runOpMode(){
            //Chassis chassis = new Chassis(hardwareMap); //Here you can add every element of the robot
            Intake intake = new Intake(hardwareMap);
            GamepadEx driver = new GamepadEx(gamepad1);
            GamepadEx operator = new GamepadEx(gamepad2);

            /*------------------------------------------------------------------*/
            /*------------------------------------------------------------------*/

           // chassis.setDefaultCommand(new Drive(chassis,gamepad1));

            Button operatorButtonY= operator.getGamepadButton(GamepadKeys.Button.Y);
            operatorButtonY.whenHeld(new MoveIntake(intake,1.0));
            operatorButtonY.whenReleased(new MoveIntake(intake,0.0));

            Button operatorButtonB= operator.getGamepadButton(GamepadKeys.Button.B);
            operatorButtonB.whenHeld(new MoveIntake(intake,-1.0));
            operatorButtonB.whenReleased(new MoveIntake(intake,0.0));

            waitForStart();
            /*chassis.reset(new Pose2d(0,0, Rotation2d.fromDegrees(0))); /*When the Op mode starts,
                                                                every value will return to be zero*/

            while (opModeIsActive()) { //This will occur whenever the op mode is active
                CommandScheduler.getInstance().run();
                //Pose2d pose = chassis.getPose();

                // -- ODOMETRY TELEMETRY -- //
               /* telemetry.addData("X", pose.getX()); //This will display the telemetry on the DriverHub
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading", pose.getRotation().getDegrees());
                telemetry.addData("RightDistance", chassis.rightDistance());
                telemetry.addData("LeftDistance", chassis.leftDistance());

                // -- UPDATE TELEMETRY -- //
                telemetry.update();*/
            }
    }
}
