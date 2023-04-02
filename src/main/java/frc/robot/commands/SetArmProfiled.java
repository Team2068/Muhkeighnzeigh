// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Photonvision;
import frc.robot.utilities.DebugTable;

public class SetArmProfiled extends CommandBase {

  Constraints constraints = new Constraints(360, 320);
  PIDController controller = new PIDController(0.07, 0.06, 0);
  Timer timer = new Timer();
  TrapezoidProfile profile;
  ArmSubsystem arm;
  Photonvision photon;

  double targetAngle;
  boolean stopped;

  public SetArmProfiled(double angle, ArmSubsystem arm, Photonvision vision) {
    targetAngle = angle;
    this.arm = arm;
    photon = vision;
    addRequirements(arm, vision);
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
    DebugTable.set("Current Position", expected.position);
    DebugTable.set("Leftover Profile Time", profile.timeLeftUntil(targetAngle));

    arm.setVoltage(MathUtil.clamp(output, -12, 12));
    // arm.setReference(current.position); // NOTE: May use our own if this doesn't go too well
  }

  public void setAngle(double angle){
    profile = new TrapezoidProfile(constraints,
      new State(angle, 0),
      new State(arm.getArmPosition(), 0));

    photon.rotateMount(angle);
    controller.reset();
    timer.reset();

    // return this;
  }

  public void stop(){
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
    // return profile.isFinished(timer.get()) || Math.abs(targetAngle - arm.getArmPosition()) < 1 ;
  }
}
