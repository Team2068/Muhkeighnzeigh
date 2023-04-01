// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.utilities.DebugTable;

public class SetArmProfiled extends CommandBase {

  PIDController controller = new PIDController(0.07, 0.06, 0);
  Timer timer = new Timer();
  TrapezoidProfile profile;
  ArmSubsystem arm;
  TelescopeSubsystem telescope;

  double targetAngle;

  public SetArmProfiled(double angle, ArmSubsystem arm, TelescopeSubsystem telescope) {
    targetAngle = angle;
    this.arm = arm;
    this.telescope = telescope;
    addRequirements(arm);
    addRequirements(telescope);
  }

  @Override
  public void initialize() {
    double maxVelocity = 360;
    // double maxAcceleration = arm.feedforward.maxAchievableAcceleration(12, Math.toRadians(targetAngle), Math.toRadians(maxVelocity));
    double maxAcceleration = arm.getMaxAcceleration(targetAngle);
    DebugTable.set("Max Vel", maxVelocity);
    DebugTable.set("Max Accel",maxAcceleration);
    profile = new TrapezoidProfile(
      new Constraints(360, 12),
      // new Constraints(maxVelocity, maxAcceleration),
      new TrapezoidProfile.State(targetAngle, 0),
      new TrapezoidProfile.State(arm.getArmPosition(), 0));

    controller.reset();
    timer.restart();
  }

  @Override
  public void execute() {
    State expected = profile.calculate(timer.get());
    
    double currentPos = arm.getArmPosition();
    double output = controller.calculate(currentPos, expected.position); // + arm.calculateFeedforward(Units.degreesToRadians(expected.position));
    
    DebugTable.set("Current Position", expected.position);
    DebugTable.set("Output", output);
    DebugTable.set("Elapsed Profile Time", timer.get());
    DebugTable.set("Leftover Profile Time", profile.timeLeftUntil(targetAngle));
    DebugTable.set("Total Time", profile.totalTime());

    arm.setVoltage(MathUtil.clamp(output, -12, 12));
    // arm.setReference(current.position); // NOTE: May use our own if this doesn't go too well
  }

  @Override
  public void end(boolean interrupted) {
    // arm.stop();
    arm.setVoltage(-0.7); // telescope = 0
    // double armExtensionMultiplier = MathUtil.interpolate(1, 2.325, telescope.getPosition() / 90);
    // arm.setVoltage(-0.7 * armExtensionMultiplier); // telescope = 1
  }

  @Override
  public boolean isFinished() {
    double currentTime = timer.get();
    return profile.isFinished(currentTime) || Math.abs(targetAngle - arm.getArmPosition()) < 1 ;
  }
}
