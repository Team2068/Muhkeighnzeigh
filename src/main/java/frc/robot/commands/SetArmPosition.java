// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPosition extends CommandBase {
  private final PIDController controller = new PIDController(0.8, 0, 0);
  private final ArmSubsystem armSubsystem;
  private double lastPosition = 0;

  public SetArmPosition(ArmSubsystem armSubsystem, double angleDegrees) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    controller.setSetpoint(angleDegrees);
    controller.setTolerance(5); // 5 degree tolerance
    controller.enableContinuousInput(0, 360);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var setpoint = controller.getSetpoint();
    var currentPosition = armSubsystem.getArmPosition();

    double pidOutput = controller.calculate(currentPosition);
    double ffOutput = armSubsystem.calculateFeedforward(Math.toRadians(setpoint),
        (Math.toRadians(currentPosition) - Math.toRadians(lastPosition)) / controller.getPeriod());
    double newOutput = (pidOutput / 180 * 8) + ffOutput;

    SmartDashboard.putNumber("SAP PID", pidOutput);
    SmartDashboard.putNumber("SAP FF", ffOutput);
    SmartDashboard.putNumber("SAP Voltage", newOutput);

    armSubsystem.setVoltage(newOutput);
    lastPosition = currentPosition;
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.set(0);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
    // return false;
  }
}
