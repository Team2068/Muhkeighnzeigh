// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ProfiledSetArmPosition extends CommandBase {
  private final double FUNNY_RATIO = 0.044;
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.01, 0.01);
  private final ProfiledPIDController controller = new ProfiledPIDController(1.1 * FUNNY_RATIO, 0, 0, constraints);

  private final ArmSubsystem armSubsystem;

  public ProfiledSetArmPosition(ArmSubsystem armSubsystem, TrapezoidProfile.State goal) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    controller.setGoal(goal);
    controller.enableContinuousInput(0, 360);
  }

  @Override
  public void initialize() {
  }

  public void setGoal(TrapezoidProfile.State state) {
    controller.setGoal(state);
  }

  @Override
  public void execute() {
    var setpoint = controller.getSetpoint();
    var output = controller.calculate(armSubsystem.getArmPosition());
    var feedforward = armSubsystem.calculateFeedforward(setpoint.position, setpoint.velocity);

    armSubsystem.setVoltage(output + feedforward);

    System.out.printf("E: %.2f, SP: %.2f, O: %.2f, FF: %.2f\n", controller.getPositionError(), setpoint.position,
        output, feedforward);
    System.out.printf("Voltage: %.2f\n", output + feedforward);
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
