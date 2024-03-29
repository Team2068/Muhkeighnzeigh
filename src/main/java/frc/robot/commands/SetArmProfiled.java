// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;

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

  Constraints constraints = new Constraints(360, 100);
  PIDController controller = new PIDController(0.07, 0.06, 0);
  Timer timer = new Timer();
  TrapezoidProfile profile;
  ArmSubsystem arm;
  TelescopeSubsystem telescope;

  Consumer<Double> flip;
  double targetAngle;
  boolean stopped;

  public SetArmProfiled(double angle, ArmSubsystem arm, TelescopeSubsystem telescope, Consumer<Double> flip, boolean stop) {
    targetAngle = angle;
    this.arm = arm;
    this.telescope = telescope;
    this.flip = flip;
    stopped = stop;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    profile = new TrapezoidProfile(constraints,
        new TrapezoidProfile.State(targetAngle, 0),
        new TrapezoidProfile.State(arm.getArmPosition(), 0));

    controller.reset();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (stopped) return;
    State expected = profile.calculate(timer.get());

    double output = controller.calculate(arm.getArmPosition(), expected.position);

    DebugTable.set("Output", output);
    DebugTable.set("Expected Position", expected.position);
    DebugTable.set("Leftover Profile Time", profile.timeLeftUntil(targetAngle));
    double extFactor = MathUtil.clamp((telescope.getPosition() / 24), 0, 0.75) * ((targetAngle < 0) ? -1 : 1);
    arm.setVoltage(MathUtil.clamp(output - extFactor, -12, 12));
  }

  public void setAngle(double angle) {
    stopped = false;
    long mask = 1 << 63;

    profile = new TrapezoidProfile(
        new Constraints(360, ( (((long)angle) & mask) != (((long)targetAngle) & mask)) ? 50 : 100),
        new State(angle, 0),
        new State(arm.getArmPosition(), 0));

    targetAngle = angle;
    flip.accept(angle);
    controller.reset();
    timer.reset();
  }

  public void stop() {
    stopped = true;
    arm.stop();
  }

  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
