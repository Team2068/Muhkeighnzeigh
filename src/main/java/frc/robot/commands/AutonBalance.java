// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonBalance extends SequentialCommandGroup {
  private final double kP = 0.3;

  private class Balance extends PIDCommand {
    DriveSubsystem driveSubsystem;

    public Balance(DriveSubsystem driveSubsystem, double setpoint) {
      super(new PIDController(kP, 0, 0), driveSubsystem.pigeon2::getRoll, setpoint, output -> {
        driveSubsystem.drive(new ChassisSpeeds(-output * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0,0));
      }, driveSubsystem);
      this.driveSubsystem = driveSubsystem;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_controller.getPositionError()) < 1;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("Robot Balanced!");
        driveSubsystem.drive(new ChassisSpeeds());
    }
  }

  public AutonBalance(DriveSubsystem driveSubsystem) {
    // Since we cannot zero the ROLL of the gyro, take the initial roll
    // Before we balance (the roll when the robot is flat) and make this our setpoint
    final double initialRoll = driveSubsystem.pigeon2.getRoll();
    addCommands(
      new DefaultDriveCommand(driveSubsystem, new ChassisSpeeds(2, 0, 0)).withTimeout(3),
      new Balance(driveSubsystem, initialRoll)
    );
  }
}
