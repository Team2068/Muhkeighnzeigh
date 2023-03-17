// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ScoreLow extends SequentialCommandGroup {
  public ScoreLow(TelescopeSubsystem telescopeSubsystem, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
    SetArmPosition armCommand = new SetArmPosition(armSubsystem, 75
    );
    addCommands(
      new ParallelCommandGroup(
        armCommand,
        new SequentialCommandGroup(
          new WaitCommand(2),
          new SetTelescopePosition(telescopeSubsystem, armSubsystem, TelescopeConstants.LOW_POSITION),
          new SetClawPosition(clawSubsystem, ClawConstants.FLAT_POSITION),
          new WaitCommand(1),
          new InstantCommand(clawSubsystem::openClaw)
        )
      ).deadlineWith(new WaitCommand(6))
    );
  }
}
