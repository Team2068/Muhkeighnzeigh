// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonBalance extends SequentialCommandGroup {
  private final double kP = 0.3;

  public AutonBalance(DriveSubsystem driveSubsystem) {
    addCommands(
      new DefaultDriveCommand(driveSubsystem, new ChassisSpeeds(1, 0, 0)).withTimeout(1),
      new PIDCommand(
        new PIDController(kP, 0, 0),
        driveSubsystem.pigeon2::getPitch, 0, output -> {
          driveSubsystem.drive(new ChassisSpeeds(output * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0, 0));
        },
        driveSubsystem).until(() -> Math.abs(driveSubsystem.pigeon2.getPitch()) < 1),
      new DefaultDriveCommand(driveSubsystem, new ChassisSpeeds()),
      new InstantCommand(() -> System.out.println("Robot Balanced"))
    );
  }
}
