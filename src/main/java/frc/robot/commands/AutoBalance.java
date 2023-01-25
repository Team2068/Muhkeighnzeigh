// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends PIDCommand {
    public AutoBalance(DriveSubsystem driveSubsystem) {
        super(
        new PIDController(2, 0, 0),
        driveSubsystem.pigeon2::getPitch,
        0,
        output -> {
            driveSubsystem.drive(new ChassisSpeeds(0, output * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND, 0));
        },
        driveSubsystem);
    }
    
    @Override
    public boolean isFinished() {
        return m_controller.getPositionError() < 1;
    }
}