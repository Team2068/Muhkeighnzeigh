// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.TelescopeSubsystem;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pickup2 extends SequentialCommandGroup {
  public Pickup2(Boolean pickingCone, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, Photonvision vision, TelescopeSubsystem telescopeSubsystem) {
    addCommands(
      new SetArmProfiled(18, armSubsystem, telescopeSubsystem, vision::rotateMount), 
      new InstantCommand(armSubsystem::stop),
      new InstantCommand(clawSubsystem::openClaw),
      new InstantCommand(clawSubsystem::intake),
      new SetClawPosition(clawSubsystem, 175),
      new InstantCommand(() -> { if (pickingCone) clawSubsystem.closeClaw();}),
      new WaitCommand(0.5),
      new InstantCommand(clawSubsystem::stopClaw)
    );
  }
}
