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
import frc.robot.subsystems.TelescopeSubsystem;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // private final Photonvision photonvision = new Photonvision(RobotConstants.camName1);
  final DriveSubsystem driveSubsystem = new DriveSubsystem();
  final ArmSubsystem armSubsystem = new ArmSubsystem();
  final ClawSubsystem clawSubsystem = new ClawSubsystem();
  final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
  
  final CommandXboxController mechController = new CommandXboxController(1);
  final CommandXboxController driverController = new CommandXboxController(0);
 
  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,
        () -> -modifyAxis(driverController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    PathPlannerServer.startServer(5811);
  } 
 
  private void configureBindings() {
    // mechController.a().onTrue(new InstantCommand(()->clawSubsystem.setWristSpeed(-.5)));
    mechController.a().onTrue(new SetArmPosition(armSubsystem, 275));
    mechController.b().onTrue(new SetArmPosition(armSubsystem, 180));
    mechController.x().onTrue(new InstantCommand(()->armSubsystem.set(1)));
    mechController.y().onTrue(new InstantCommand(telescopeSubsystem::stopTelescope));
    mechController.leftTrigger().onTrue(new SetClawPosition(clawSubsystem, 175));
    mechController.leftBumper().onTrue(new SetClawPosition(clawSubsystem, 245));
    mechController.rightTrigger().onTrue(new InstantCommand(telescopeSubsystem::extendTelescope));
    mechController.rightBumper().onTrue(new InstantCommand(telescopeSubsystem::retractTelescope));
    
    driverController.x().whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry()));
    driverController.y().whileTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    driverController.a().whileTrue(new InstantCommand(() -> driveSubsystem.drive(new ChassisSpeeds())));
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
