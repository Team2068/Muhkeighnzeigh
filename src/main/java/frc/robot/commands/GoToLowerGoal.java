// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToLowerGoal extends ProfiledPIDCommand {
  /** Creates a new GoToLowerGoal. */
  public GoToLowerGoal(ArmSubsystem armSubsystem) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            15,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.1, 0.1)),
        // This should return the measurement
          armSubsystem::getArmPosition,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(90, 0.25),
        // This uses the output
        (output, setpoint) -> {
          armSubsystem.setVoltage(output*3);
          SmartDashboard.putNumber("PID Output", output);
        });
        addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
