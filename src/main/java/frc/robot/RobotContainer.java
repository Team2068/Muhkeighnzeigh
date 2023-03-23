// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Paths;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SetTelescopePosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // private final Photonvision photonvision = new Photonvision(PhotonConstants.CAM_NAME);
  final DriveSubsystem driveSubsystem = new DriveSubsystem();
  final ArmSubsystem armSubsystem = new ArmSubsystem();
  final ClawSubsystem clawSubsystem = new ClawSubsystem();
  final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
  
  final CommandXboxController mechController = new CommandXboxController(1);
  final CommandXboxController driverController = new CommandXboxController(0);

 
  SendableChooser<Command> autonomousSelector = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    initEventMap();
    configureAutonomous();
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem,
        () -> -modifyAxis(driverController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
      // SmartDashboard.putData(new InstantCommand(photonvision::rotateMount));
      SmartDashboard.putData("Auto Selector", autonomousSelector);
      PathPlannerServer.startServer(5811);
  } 

  private void configureAutonomous() {
    autonomousSelector.setDefaultOption("Drive to side + Park", new SequentialCommandGroup(
      driveSubsystem.followPath(Paths.park),
      new AutonBalance(driveSubsystem, false)));
    autonomousSelector.addOption("Only Park", new AutonBalance(driveSubsystem, false));
    autonomousSelector.addOption("Leave Community", driveSubsystem.followPath(Paths.leaveCommunity));
    autonomousSelector.addOption("Score Mid + Leave Community", new SequentialCommandGroup(
      new ScoreLow(telescopeSubsystem, armSubsystem, clawSubsystem),
      new WaitCommand(1),
      new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0),
      driveSubsystem.followPath(Paths.leaveCommunity)));
      // driveSubsystem.followPathWithEvents(Paths.picking, Paths.eventMap)
    autonomousSelector.addOption("Door Dash", driveSubsystem.followPathGroupWithEvents(Paths.picking).andThen(new AutonBalance(driveSubsystem, true)));
    autonomousSelector.addOption("Leave Community + Park", new SequentialCommandGroup(
      new InstantCommand(clawSubsystem::closeClaw),
      new ScoreHigh(armSubsystem, telescopeSubsystem, clawSubsystem),
      new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0),
      new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION),
      driveSubsystem.followPath(Paths.leaveCommunityPark),
      new AutonBalance(driveSubsystem, true)
    ));
  }

  public void initEventMap(){
    Paths.eventMap.put("pickup", new SequentialCommandGroup(
      new InstantCommand(clawSubsystem::openClaw),
      new SetClawPosition(clawSubsystem, ClawConstants.INTAKE_POSITION),
      new InstantCommand(clawSubsystem::closeClaw)
    ));
    Paths.eventMap.put("print", new PrintCommand("PRINTINTINTINTTINTIN"));
    Paths.eventMap.put("liftarm", new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION).withTimeout(1));
    Paths.eventMap.put("scorehigh", new ScoreHigh(armSubsystem, telescopeSubsystem, clawSubsystem).andThen(new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0)));
    Paths.eventMap.put("scorelow", new ScoreLow(telescopeSubsystem, armSubsystem, clawSubsystem).andThen(new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0)));
  }
 
  private void configureBindings() {
    SetArmPosition armCommand = new SetArmPosition(armSubsystem, 75);

    // mechController.x().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.FLAT_POSITION));
    mechController.a().onTrue(new InstantCommand(armCommand::cancel));
    // mechController.b().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION));
    mechController.y().onTrue(armCommand);
   
    mechController.rightBumper().onTrue(new InstantCommand(clawSubsystem::openClaw));
  //  mechController.rightTrigger().onTrue(new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.HIGH_POSITION));
    // mechController.rightTrigger().whileTrue(new InstantCommand(telescopeSubsystem::extendTelescope)).whileFalse(new InstantCommand(telescopeSubsystem::stopTelescope));

    mechController.leftBumper().onTrue(new InstantCommand(clawSubsystem::closeClaw));
    // mechController.leftTrigger().onTrue(new ScoreLow(telescopeSubsystem, armSubsystem, clawSubsystem));
    mechController.leftTrigger().whileTrue(new InstantCommand(telescopeSubsystem::extendTelescope)).whileFalse(new InstantCommand(telescopeSubsystem::stopTelescope));


    mechController.povUp().onTrue(new InstantCommand(telescopeSubsystem::resetPosition));
    mechController.povDown().whileTrue(new InstantCommand(telescopeSubsystem::retractTelescope)).whileFalse(new InstantCommand(telescopeSubsystem::stopTelescope));
    // mechController.povLeft().onTrue(new SetClawPosition(clawSubsystem, ClawConstants.INTAKE_POSITION));    
    // mechController.povRight().onTrue(new InstantCommand(armCommand::flipPosition));
    clawSubsystem.setDefaultCommand(new InstantCommand(() -> clawSubsystem.setWristVoltage(MathUtil.clamp(mechController.getLeftY() * ClawConstants.WRIST_VOLTAGE, -ClawConstants.WRIST_VOLTAGE, ClawConstants.WRIST_VOLTAGE)), clawSubsystem));

    // mechController.povRight().onTrue(new ScoreHigh(armSubsystem, telescopeSubsystem, clawSubsystem));

    driverController.x().whileTrue(new InstantCommand(() -> driveSubsystem.resetOdometry()));
    driverController.y().whileTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    driverController.b().whileTrue(new InstantCommand(() -> driveSubsystem.toggleFieldOriented()));
    driverController.rightTrigger().onTrue(new InstantCommand(driveSubsystem::toggleSlowMode));
    driverController.a().onTrue(new InstantCommand(driveSubsystem::syncEncoders));
    // driverController.leftTrigger().toggleOnTrue(new InstantCommand(() -> photonvision.togglePipeline()));
    // driverController.rightBumper().whileTrue(new Aimbot(photonvision, driveSubsystem));
  } 

  public Command getAutonomousCommand() {
    return autonomousSelector.getSelected();
  }

  private static double deadband(double value, double deadband) {

    if (Math.abs(value) <= deadband) return 0.0;
    deadband *= (value > 0.0) ? 1 : -1;
    return (value + deadband) / (1.0 + deadband);
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.1); // Deadband
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }

  public void syncEncodersDisabled() {
    driveSubsystem.syncEncoders();
  }
}
