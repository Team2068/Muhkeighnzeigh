// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.Paths;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.Score;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SetTelescopePosition;
import frc.robot.utilities.General;
import frc.robot.utilities.IO;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  final SendableChooser<Command> autonomousSelector = new SendableChooser<>();
  final SendableChooser<Runnable> bindingsSelector = new  SendableChooser<Runnable>();
 
  public IO io = new IO(bindingsSelector);

  public RobotContainer() {
    initEventMap();
    configureAutonomous();
    io.configGlobal();
    
    SmartDashboard.putData("Auto Selector", autonomousSelector);
    CameraServer.startAutomaticCapture();
  }

  private void configureAutonomous() {
    autonomousSelector.setDefaultOption("Drive to side + Park", new SequentialCommandGroup(
        io.driveSubsystem.followPath(Paths.park),
        new AutonBalance(io.driveSubsystem, false)));
    autonomousSelector.addOption("Only Park", new SequentialCommandGroup(General.Instant(() -> io.armCommand.setAngle(75)), new AutonBalance(io.driveSubsystem, false)));
    autonomousSelector.addOption("Only Park Reversed", new AutonBalance(io.driveSubsystem, true));
    autonomousSelector.addOption("Leave Community", io.driveSubsystem.followPath(Paths.leaveCommunity));
    autonomousSelector.addOption("Score Mid + Leave Community", new SequentialCommandGroup(
        new Score(io, false),
        new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION),
        io.driveSubsystem.followPath(Paths.leaveCommunity)));
    autonomousSelector.addOption("frfr", new Score(io, false));
    autonomousSelector.addOption("Door Dash",
        io.driveSubsystem.followPathGroupWithEvents(Paths.picking).andThen(new AutonBalance(io.driveSubsystem, true)));
    autonomousSelector.addOption("Leave Community + Park", new SequentialCommandGroup(
      new Score(io, false),
        General.Instant(io.claw::closeClaw),
        new Score(io, true),
        new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION),
        io.driveSubsystem.followPath(Paths.leaveCommunityPark),
        new AutonBalance(io.driveSubsystem, true)));
    autonomousSelector.addOption("Score Mid + Park", new SequentialCommandGroup(
      new Score(io, false),
      new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION),
      new AutonBalance(io.driveSubsystem, true)));
    autonomousSelector.addOption("Score Mid", new SequentialCommandGroup(
      new Score(io, false),
      new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION).withTimeout(0.5)));
    autonomousSelector.addOption("Score Mider", new SequentialCommandGroup(
      new Score(io, false),
      new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION).withTimeout(0.5),
      General.Instant(io.claw::openClaw),
      io.driveSubsystem.followPathGroupWithEvents(Paths.cooking)
    ));
    autonomousSelector.addOption("We BALL auto", new SequentialCommandGroup(
      new Score(io, false),
      io.driveSubsystem.followPathGroupWithEvents(Paths.weBall)));
    autonomousSelector.addOption("Loop", io.driveSubsystem.followPath(Paths.loop));
  }

  private void initEventMap() {
    Paths.eventMap.put("pickup", new SequentialCommandGroup(
        General.Instant(io.claw::openClaw),
        new SetClawPosition(io.claw, ClawConstants.INTAKE_POSITION).withTimeout(1),
        General.Instant(io.claw::intake)));
    Paths.eventMap.put("closeclaw", General.Instant(io.claw::closeClaw));
    Paths.eventMap.put("openclaw", General.Instant(io.claw::openClaw));
    Paths.eventMap.put("stopintake", General.Instant(io.claw::stopClaw));
    Paths.eventMap.put("intakeposition",
        new SetClawPosition(io.claw, ClawConstants.INTAKE_POSITION).withTimeout(1));
    Paths.eventMap.put("liftarm", new SetClawPosition(io.claw, ClawConstants.CARRY_POSITION).withTimeout(1));
    Paths.eventMap.put("scorehigh", new Score(io, true)
        .andThen(new SetTelescopePosition(io.telescope, io.arm, 0)));
    Paths.eventMap.put("scorelow", new Score(io, false)
        .andThen(new SetTelescopePosition(io.telescope,io.arm, 0)));
  }


  public Command getAutonomousCommand() {
    InstantCommand postAutonomous = new InstantCommand(() -> {
      if (!io.driveSubsystem.isFieldOriented())
        io.driveSubsystem.toggleFieldOriented();

      // Since we start the robot with "forward" facing towards the drivers,
      // Add 180 to the gyro's rotation so that "forward" is facing downfield
      // This makes forward (field oriented) away from the driver, as intended.
      io.driveSubsystem.pigeon2.addYaw(180);
    }, io.driveSubsystem);
    return autonomousSelector.getSelected().andThen(postAutonomous);
  }

}
