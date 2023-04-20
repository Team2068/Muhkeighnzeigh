// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.TelescopeSubsystem;

public class ScoreMid extends SequentialCommandGroup {
  public ScoreMid(TelescopeSubsystem telescopeSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, Photonvision vision) {
    SetArmProfiled armCommand = new SetArmProfiled(75,armSubsystem, telescopeSubsystem, vision::rotateMount, false);
    addCommands(
      new ParallelCommandGroup(
        armCommand,
        new PrintCommand("Starting Low..."),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.LOW_POSITION),
          new SetClawPosition(clawSubsystem, -50).withTimeout(0.5),
          new InstantCommand(clawSubsystem::openClaw),
          new InstantCommand(() -> clawSubsystem.setIntakeSpeed(-0.5)),
          new SetTelescopePosition(telescopeSubsystem, armSubsystem, 0)
        )
      ).withTimeout(3),
      new InstantCommand(clawSubsystem::stopClaw),
      new InstantCommand(() -> clawSubsystem.setWristVoltage(0))
    );
  }
}