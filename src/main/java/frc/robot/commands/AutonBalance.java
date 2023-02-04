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

  private class Balance extends PIDCommand {
    DriveSubsystem driveSubsystem;

    public Balance(DriveSubsystem driveSubsystem) {
      super(new PIDController(0.3, 0, 0.01), driveSubsystem.pigeon2::getPitch, 0, output -> {
        driveSubsystem.drive(new ChassisSpeeds(output * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0,0));
      }, driveSubsystem);
      this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        super.execute();
        System.out.println("RUNNING PIDBALANCE!!!");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_controller.getPositionError()) < 1;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        driveSubsystem.drive(new ChassisSpeeds());
    }
  }

  /** Creates a new AutonBalance. */
  public AutonBalance(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DefaultDriveCommand(driveSubsystem, ()->1, ()->0, ()->0).withTimeout(1),
      new Balance(driveSubsystem)
    );
  }
}
