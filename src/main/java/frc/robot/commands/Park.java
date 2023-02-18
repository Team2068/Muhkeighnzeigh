package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

public class Park extends SequentialCommandGroup{
    public Park(DriveSubsystem driveSubsystem){
        addCommands(
            new Paths(Trajectories.Step1_Park, driveSubsystem)
        );
    }
}