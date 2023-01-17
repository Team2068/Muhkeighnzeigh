package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

public class Auto1 extends SequentialCommandGroup {
    public Auto1(DriveSubsystem driveSubsystem) {
        addCommands(
            new Paths(TrajectoryPaths.Step1_4Cargo, driveSubsystem), 
            new Paths(Trajectories.Step2_4Cargo, driveSubsystem),
            new Paths(TrajectoryPaths.Step3_4Cargo, driveSubsystem),
            new Paths(TrajectoryPaths.Step4_4Cargo, driveSubsystem),
            new Paths(TrajectoryPaths.Step5_4Cargo, driveSubsystem),
            new Paths(TrajectoryPaths.Step5_4Cargo, driveSubsystem),
            new Paths(TrajectoryPaths.Step6_4Cargo, driveSubsystem)
    );
}
}
