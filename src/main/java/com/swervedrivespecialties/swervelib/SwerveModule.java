package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getDrivePosition();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void resetDrivePosition();

    void resetSteerPosition();
}
