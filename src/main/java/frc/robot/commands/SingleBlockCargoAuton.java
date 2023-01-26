package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

public class SingleBlockCargoAuton extends SequentialCommandGroup{
    public SingleBlockCargoAuton( DriveSubsystem driveSubsystem){
        addCommands(
            new Paths(Trajectories.Step1_1BlockCargo, driveSubsystem),
            new Paths(Trajectories.Step2_1BlockCargo, driveSubsystem)
        );
    }
}
