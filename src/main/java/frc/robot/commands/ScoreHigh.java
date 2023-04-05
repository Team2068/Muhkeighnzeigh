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

public class ScoreHigh extends SequentialCommandGroup {
  public ScoreHigh(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem, Photonvision vision) {
    SetArmProfiled armCommand = new SetArmProfiled(70, armSubsystem, telescopeSubsystem, vision::rotateMount);
    addCommands(
      new ParallelCommandGroup(
        armCommand,
        new PrintCommand("Starting High..."),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.HIGH_POSITION),
          new SetClawPosition(clawSubsystem, 0).withTimeout(0.5),
          new InstantCommand(clawSubsystem::openClaw)
        )
      ).withTimeout(4)
    );
  }
}
