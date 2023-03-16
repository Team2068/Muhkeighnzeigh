// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPosition extends CommandBase {
  private final PIDController controller = new PIDController(0.07, 0, 0);
  private final ArmSubsystem armSubsystem;
  private double lastPosition = 0;

  public SetArmPosition(ArmSubsystem armSubsystem, double angleDegrees) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    controller.setSetpoint(angleDegrees);
    controller.setTolerance(0); // 5 degree tolerance
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
    double newOutput = (pidOutput) + -ffOutput;

    // SmartDashboard.putNumber("SAP PID", pidOutput);
    // SmartDashboard.putNumber("SAP FF", ffOutput);
    SmartDashboard.putNumber("SAP Voltage", newOutput);
    
    armSubsystem.setVoltage(MathUtil.clamp(newOutput, -12, 12));
    lastPosition = currentPosition;
  }

  public void updateSetpoint(double angleDegrees) {
    controller.setSetpoint(angleDegrees);
  }

  public void flipPosition() {
    controller.setSetpoint(-controller.getSetpoint());
  }

  public boolean isCloseEnough() {
    System.out.println("Error: " + controller.getPositionError());
    return Math.abs(controller.getPositionError()) < 20;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("exiting!!!");
    System.out.println("exiting!!!");
    System.out.println("exiting!!!");
    System.out.println("exiting!!!");
    System.out.println("exiting!!!");
    System.out.println("exiting!!!");
    armSubsystem.set(0);
  }

  @Override
  public boolean isFinished() {
    // return controller.atSetpoint();
    return false;
  }
}
