// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Paths;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.Aimbot;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Photonvision photonvision = new Photonvision(RobotConstants.camName1);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
            () ->
                -modifyAxis(driverController.getLeftY())
                    * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(driverController.getLeftX())
                    * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () ->
                -modifyAxis(driverController.getRightX())
                    * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    PathPlannerServer.startServer(5811);
  }

  private void configureBindings() {
    driverController
        .a()
        .whileTrue(new InstantCommand(() -> driveSubsystem.drive(new ChassisSpeeds())));
    driverController.y().whileTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    driverController.x().whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry()));
    driverController.b().whileTrue(new InstantCommand(() -> driveSubsystem.toggleFieldOriented()));
    driverController
        .leftTrigger()
        .toggleOnTrue(new InstantCommand(() -> photonvision.togglePipeline()));
    driverController.rightBumper().whileTrue(new Aimbot(photonvision, driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        driveSubsystem.followPath(Paths.loop), new AutonBalance(driveSubsystem));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) return (value - deadband) / (1.0 - deadband);
      return (value + deadband) / (1.0 - deadband);
    }
    return 0.0;
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.05); // Deadband
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }
}
