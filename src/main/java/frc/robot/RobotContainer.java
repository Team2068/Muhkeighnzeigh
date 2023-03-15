// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Paths;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.commands.Aimbot;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Pickup2;
import frc.robot.commands.Score;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SetTelescopePosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.TelescopeSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final Photonvision photonvision = new Photonvision(PhotonConstants.CAM_NAME);
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
      SmartDashboard.putData(new InstantCommand(photonvision::rotateMount));
  } 
 
  private void configureBindings() {
   SetArmPosition armCommand = new SetArmPosition(armSubsystem, 75);

    mechController.x().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.FLAT_POSITION));
    mechController.a().onTrue(new InstantCommand(armCommand::cancel));
    mechController.b().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION));
    mechController.y().onTrue(armCommand);

    mechController.rightBumper().onTrue(new InstantCommand(clawSubsystem::openClaw));
    mechController.rightTrigger().onTrue(new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.HIGH_POSITION));

    mechController.leftBumper().onTrue(new InstantCommand(clawSubsystem::closeClaw));
    mechController.leftTrigger().onTrue(new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.LOW_POSITION));

    mechController.povUp().onTrue(new InstantCommand(telescopeSubsystem::resetPosition));
    mechController.povDown().onTrue(new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0));
    mechController.povLeft().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.INTAKE_POSITION));
    mechController.rightStick().onTrue(new InstantCommand(armCommand::flipPosition));
    
    driverController.x().whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry()));
    driverController.y().whileTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    driverController.b().whileTrue(new InstantCommand(() -> driveSubsystem.toggleFieldOriented()));
    driverController.leftTrigger().toggleOnTrue(new InstantCommand(() -> photonvision.togglePipeline()));
    driverController.rightBumper().whileTrue(new Aimbot(photonvision, driveSubsystem));
    driverController.rightTrigger().whileTrue(new InstantCommand(() -> photonvision.rotateMount()));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      driveSubsystem.followPath(Paths.park),
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
