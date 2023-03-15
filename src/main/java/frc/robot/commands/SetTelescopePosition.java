// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TelescopeSubsystem;

public class SetTelescopePosition extends PIDCommand {
  TelescopeSubsystem telescopeSubsystem;

  public SetTelescopePosition(TelescopeSubsystem telescopeSubsystem, double position) {
    super(
        new PIDController(0.3, 0, 0),
        telescopeSubsystem::getPosition,
        position,
        output -> {
          telescopeSubsystem.setVoltage(MathUtil.clamp(output, -12, 12));
        });
    addRequirements(telescopeSubsystem);
    this.telescopeSubsystem = telescopeSubsystem;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < 3; 
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TELESCOPE HAS REACHED POSITION :DDDD");
    telescopeSubsystem.stopTelescope();
  }
}
