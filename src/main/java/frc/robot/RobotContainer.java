// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Paths;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetClawPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // private final Photonvision photonvision = new Photonvision(RobotConstants.camName1);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();

  private final CommandXboxController mechController = new CommandXboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,
        () -> -modifyAxis(driverController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    PathPlannerServer.startServer(5811);
  }

  private void configureBindings() {
    mechController.a().whileTrue(new InstantCommand(()->clawSubsystem.setWristSpeed(-.5)));
    mechController.b().whileTrue(new SetArmPosition(armSubsystem, 180));
    mechController.a().onFalse(new InstantCommand(()->clawSubsystem.setWristSpeed(0)));
    mechController.y().onTrue(new SetArmPosition(armSubsystem, 360));
    mechController.leftTrigger().onTrue(new SetClawPosition(clawSubsystem, 90));
    mechController.leftBumper().onTrue(new SetClawPosition(clawSubsystem, 180));
    driverController.a().whileTrue(new InstantCommand(() -> driveSubsystem.drive(new ChassisSpeeds())));
    driverController.y().whileTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    driverController.x().whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry()));
    driverController.b().whileTrue(new InstantCommand(() -> driveSubsystem.toggleFieldOriented()));
    // driverController.leftTrigger().toggleOnTrue(new InstantCommand(() -> photonvision.togglePipeline()));
    // driverController.rightBumper().whileTrue(new Aimbot(photonvision, driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      driveSubsystem.followPath(Paths.loop),
      new AutonBalance(driveSubsystem));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) <= deadband) return 0.0;
    deadband *= (value > 0.0) ? 1 : -1;
    return (value + deadband) / (1.0 + deadband);
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.05); // Deadband
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }
}
