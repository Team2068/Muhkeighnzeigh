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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonBalance extends SequentialCommandGroup {

  private class Balance extends PIDCommand {
    DriveSubsystem driveSubsystem;

    public Balance(DriveSubsystem driveSubsystem) {
      super(new PIDController(0.3, 0, 0), driveSubsystem.pigeon2::getRoll, 0, output -> {
        driveSubsystem.drive(new ChassisSpeeds(output * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0,0));
      }, driveSubsystem);
      this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_controller.getPositionError()) < 1;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("Ending Autonbalance");
        driveSubsystem.drive(new ChassisSpeeds());
    }
  }

  public AutonBalance(DriveSubsystem driveSubsystem) {
    addCommands(
      new DefaultDriveCommand(driveSubsystem, ()->1, ()->0, ()->0).withTimeout(1),
      new Balance(driveSubsystem)
    );
  }
}
