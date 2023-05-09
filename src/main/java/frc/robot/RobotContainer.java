// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Paths;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreMid;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SetTelescopePosition;
import frc.robot.commands.SetArmProfiled;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Photonvision;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  final Photonvision photonvision = new Photonvision(PhotonConstants.CAM_NAME);
  final DriveSubsystem driveSubsystem = new DriveSubsystem();
  final LEDSubsystem ledSubsystem = new LEDSubsystem();
  final ArmSubsystem armSubsystem = new ArmSubsystem();
  final ClawSubsystem clawSubsystem = new ClawSubsystem();
  final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();

  final SendableChooser<Command> autonomousSelector = new SendableChooser<>();
  public final CommandXboxController mechController = new CommandXboxController(0);
  public final CommandXboxController driveController = new CommandXboxController(3);
 
  public RobotContainer() {
    configureBindings();
    initEventMap();
    configureAutonomous();
    photonvision.camera.setPipelineIndex(1);
    photonvision.mount.setAngle(PhotonConstants.FORWARD_ANGLE);
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand(driveSubsystem, 
    () -> -modifyAxis(driveController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    () -> -modifyAxis(driveController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
    () -> -modifyAxis(driveController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    SmartDashboard.putData("Auto Selector", autonomousSelector);
    CameraServer.startAutomaticCapture();
    // SmartDashboard.putData("Kill LEDs", new InstantCommand(ledSubsystem::killLeds, ledSubsystem));
    // PathPlannerServer.startServer(5811); // DEBUGGING
  }

  private void configureAutonomous() {
    autonomousSelector.setDefaultOption("Drive to side + Park", new SequentialCommandGroup(
        driveSubsystem.followPath(Paths.park),
        new AutonBalance(driveSubsystem, false)));
    autonomousSelector.addOption("Only Park", new SequentialCommandGroup(new InstantCommand(() -> armCommand.setAngle(75)), new AutonBalance(driveSubsystem, false)));
    autonomousSelector.addOption("Only Park Reversed", new AutonBalance(driveSubsystem, true));
    autonomousSelector.addOption("Leave Community", driveSubsystem.followPath(Paths.leaveCommunity));
    autonomousSelector.addOption("Score Mid + Leave Community", new SequentialCommandGroup(
        new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
        new WaitCommand(1),
        new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION),
        driveSubsystem.followPath(Paths.leaveCommunity)));
    autonomousSelector.addOption("frfr", new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision));
    autonomousSelector.addOption("Door Dash",
        driveSubsystem.followPathGroupWithEvents(Paths.picking).andThen(new AutonBalance(driveSubsystem, true)));
    autonomousSelector.addOption("Leave Community + Park", new SequentialCommandGroup(
        new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
        new InstantCommand(clawSubsystem::closeClaw),
        new ScoreHigh(armSubsystem, telescopeSubsystem, clawSubsystem, photonvision),
        new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION),
        driveSubsystem.followPath(Paths.leaveCommunityPark),
        new AutonBalance(driveSubsystem, true)));
    autonomousSelector.addOption("Score Mid + Park", new SequentialCommandGroup(
      new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
      new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION),
      new AutonBalance(driveSubsystem, true)));
    autonomousSelector.addOption("Score Mid", new SequentialCommandGroup(
        new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
        new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION).withTimeout(0.5)));
    autonomousSelector.addOption("Score Mider", new SequentialCommandGroup(
      new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
      new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION).withTimeout(0.5),
      new InstantCommand(clawSubsystem::openClaw),
      driveSubsystem.followPathGroupWithEvents(Paths.cooking)
    ));
    autonomousSelector.addOption("We BALL auto", new SequentialCommandGroup(
        new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision),
        driveSubsystem.followPathGroupWithEvents(Paths.weBall)));
  }

  public void initEventMap() {
    Paths.eventMap.put("pickup", new SequentialCommandGroup(
        new InstantCommand(clawSubsystem::openClaw),
        new SetClawPosition(clawSubsystem, ClawConstants.INTAKE_POSITION).withTimeout(1),
        new InstantCommand(clawSubsystem::intake)));
    Paths.eventMap.put("closeclaw", new InstantCommand(clawSubsystem::closeClaw));
    Paths.eventMap.put("openclaw", new InstantCommand(clawSubsystem::openClaw));
    Paths.eventMap.put("stopintake", new InstantCommand(clawSubsystem::stopClaw));
    Paths.eventMap.put("intakeposition",
        new SetClawPosition(clawSubsystem, ClawConstants.INTAKE_POSITION).withTimeout(1));
    Paths.eventMap.put("print", new PrintCommand("PRINTINTINTINTTINTIN"));
    Paths.eventMap.put("liftarm", new SetClawPosition(clawSubsystem, ClawConstants.CARRY_POSITION).withTimeout(1));
    Paths.eventMap.put("scorehigh", new ScoreHigh(armSubsystem, telescopeSubsystem, clawSubsystem, photonvision)
        .andThen(new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0)));
    Paths.eventMap.put("scorelow", new ScoreMid(telescopeSubsystem, armSubsystem, clawSubsystem, photonvision)
        .andThen(new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0)));
  }

  SetArmProfiled armCommand = new SetArmProfiled(73, armSubsystem, telescopeSubsystem, photonvision::rotateMount, true);

  private void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);

    armSubsystem.setDefaultCommand(armCommand);

    mechController.a().onTrue(new InstantCommand(armCommand::stop));
    mechController.b().onTrue(new InstantCommand(()->armCommand.setAngle(0)));
    mechController.x().onTrue(new InstantCommand(()->armCommand.setAngle(-75)));
    mechController.y().onTrue(new InstantCommand(()->armCommand.setAngle(60)));

    mechController.rightBumper().onTrue(new InstantCommand(clawSubsystem::openClaw).andThen(new InstantCommand(() -> ledSubsystem.setAllLeds(new Color(0.2, 0.15, 0)))));
    mechController.leftBumper().onTrue(new InstantCommand(clawSubsystem::closeClaw).andThen(new InstantCommand(() -> ledSubsystem.setAllLeds(new Color(0 ,0, 0.25)))));
    
    mechController.leftTrigger().whileTrue(new InstantCommand(telescopeSubsystem::extendTelescope))
      .whileFalse(new InstantCommand(telescopeSubsystem::stopTelescope));
    mechController.povDown().whileTrue(new InstantCommand(telescopeSubsystem::retractTelescope))
      .whileFalse(new InstantCommand(telescopeSubsystem::stopTelescope));
    mechController.povUp().onTrue(new InstantCommand(telescopeSubsystem::resetPosition));

    clawSubsystem.setDefaultCommand(new InstantCommand(
      () -> clawSubsystem.setWristVoltage(MathUtil.clamp(modifyAxis(mechController.getLeftY()) * ClawConstants.WRIST_VOLTAGE,
          -ClawConstants.WRIST_VOLTAGE, ClawConstants.WRIST_VOLTAGE)),
      clawSubsystem).alongWith(new InstantCommand(() -> clawSubsystem.setIntakeSpeed(mechController.getRightY() * 2))));

    driveController.b().onTrue(new InstantCommand(driveSubsystem::toggleFieldOriented));
    driveController.y().onTrue(new InstantCommand(driveSubsystem::resetOdometry));
    driveController.a().onTrue(new InstantCommand(driveSubsystem::syncEncoders));
    driveController.x().onTrue(new InstantCommand(driveSubsystem::zeroGyro));
  }

  public Command getAutonomousCommand() {
    InstantCommand postAutonomous = new InstantCommand(() -> {
      if (!driveSubsystem.isFieldOriented())
        driveSubsystem.toggleFieldOriented();

      // Since we start the robot with "forward" facing towards the drivers,
      // Add 180 to the gyro's rotation so that "forward" is facing downfield
      // This makes forward (field oriented) away from the driver, as intended.
      driveSubsystem.pigeon2.addYaw(180);
    }, driveSubsystem);
    return autonomousSelector.getSelected().andThen(postAutonomous);
  }

  public Command runSubsystemTests(){
    driveSubsystem.syncEncoders();
    driveSubsystem.resetOdometry();
    System.out.println("Init...");
    return new SequentialCommandGroup(
      new PrintCommand("Starting..."),
      new InstantCommand(()->driveSubsystem.drive(new ChassisSpeeds(10,0,0))),
      new InstantCommand(()->armCommand.setAngle(60)),
      new WaitCommand(0.5),
      new InstantCommand(clawSubsystem::intake),
      new InstantCommand(clawSubsystem::openClaw),
      new InstantCommand(telescopeSubsystem::extendTelescope),
      new WaitCommand(0.5),
      new InstantCommand(clawSubsystem::output),
      new InstantCommand(clawSubsystem::closeClaw),
      new InstantCommand(telescopeSubsystem::retractTelescope),
      new WaitCommand(0.5),
      new InstantCommand(armCommand::stop).alongWith(new InstantCommand(clawSubsystem::stopClaw)),
      new InstantCommand(telescopeSubsystem::stopTelescope),
      new InstantCommand(()->driveSubsystem.drive(new ChassisSpeeds()))
    );
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

  public void syncEncodersDisabled() {
    driveSubsystem.syncEncoders();
  }
}
