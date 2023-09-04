// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.utilities.DebugTable;
import frc.robot.utilities.IO;

public class Aimbot extends PIDCommand {

  IO io;

  public static double AIMBOT_OFFSET_BACKWARD = 5.67;
  public static double AIMBOT_OFFSET_FORWARD = -11.13;

  public Aimbot(IO io) {
    super(
        new PIDController(Constants.AimbotConstants.kP, Constants.AimbotConstants.kI, Constants.AimbotConstants.kD),
        () -> io.photon.data.targetYaw, // Measurement
        // This should return the setpoint (can also be a constant)
        () ->  (Double)DebugTable.get("Aimbot_Offset_Forward", -11.13), //later replace this w actual offset after tested
        output -> {
          io.driveSubsystem.drive(new ChassisSpeeds(0, output * Constants.AimbotConstants.speed * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0));
        });

    addRequirements(io.driveSubsystem, io.photon);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < Constants.AimbotConstants.minimumAdjustment || !io.photon.camera.getLatestResult().hasTargets();
  }

  @Override
  public void end(boolean interrupted) {
    io.driveSubsystem.stop();
  }
}