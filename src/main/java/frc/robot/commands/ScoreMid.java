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
import frc.robot.utilities.IO;

public class ScoreMid extends SequentialCommandGroup {
  public ScoreMid(IO io) {
    SetArmProfiled armCommand = new SetArmProfiled(75,io.arm, io.telescope, io.photon::rotateMount, false);
    addCommands(
      new ParallelCommandGroup(
        armCommand,
        new PrintCommand("Starting Low..."),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new SetTelescopePosition(io.telescope, io.arm, TelescopeConstants.LOW_POSITION),
          new SetClawPosition(io.claw, -50).withTimeout(0.5),
          new InstantCommand(io.claw::openClaw),
          new InstantCommand(() -> io.claw.setIntakeSpeed(-0.5)),
          new SetTelescopePosition(io.telescope, io.arm, 0)
        )
      ).withTimeout(3),
      new InstantCommand(io.claw::stopClaw),
      new InstantCommand(() -> io.claw.setWristVoltage(0))
    );
  }
}
