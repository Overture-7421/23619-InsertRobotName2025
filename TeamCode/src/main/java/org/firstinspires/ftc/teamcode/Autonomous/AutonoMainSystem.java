package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Commands.ElevatorPositions;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.Arrays;

@Autonomous
public class AutonoMainSystem extends LinearOpMode {


    Elevator elevator;

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();


        elevator = new Elevator(hardwareMap);

        Trajectory basicCenter = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                        new Pose2d(0.7, 0, Rotation2d.fromDegrees(0))),
                new TrajectoryConfig(1, 0.8));

        Trajectory returnTrajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(
                        new Pose2d(0.6, 0, Rotation2d.fromDegrees(180)),
                        new Pose2d(0,0, Rotation2d.fromDegrees(180))),
                new TrajectoryConfig(1,0.8));

        SequentialCommandGroup testCommandGroup = new SequentialCommandGroup(
                new WaitCommand(2000),
                new ElevatorPositions(elevator, 10)
                );
        //new RamseteCommand(chassis, returnTrajectory));
        waitForStart();



        CommandScheduler.getInstance().schedule(testCommandGroup);

        while (opModeIsActive ()){
            CommandScheduler.getInstance().run();


        }
    }
}
