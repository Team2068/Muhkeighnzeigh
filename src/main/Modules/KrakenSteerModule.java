package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class KrakenSteerModule {
    TalonFX steerMotor;
    CANSparkMax driveMotor;


    final double WHEEL_DIAMETER = 0.10033; // Metres
    final double DRIVE_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0);
    final double STEER_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    final double DRIVE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION;

    double desiredAngle;

    public SwerveModule(ShuffleboardLayout tab, int driveID, int steerID, double offset){
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        steerMotor = new TalonFX(steerCANID);

        steerMotor.configFactoryDefault();
        steerMotor.configIntegratedSensorOffset(offset, 250);
        steerMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 250);
        steerMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360, 250);
        // steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 250, 250); // TODO: find out what the different StatusFrameEnhanced Options do and which would be best for a swerve module and a good timeout
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setSensorPhase(true); // TODO: Check this
        //TODO: Check if this needs to be inverted
        // steerEncoder.configSensorDirection(false);

        steerMotor.setSmartCurrentLimit(20);
        driveMotor.setSmartCurrentLimit(40);

        steerMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.getEncoder().setPositionConversionFactor(DRIVE_CONVERSION_FACTOR);
        driveMotor.getEncoder().setVelocityConversionFactor(DRIVE_CONVERSION_FACTOR / 60.0);

        // steerMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * STEER_REDUCTION);
        // steerMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60);
        // steerMotor.getEncoder().setPosition(steerEncoder.getAbsolutePosition());

        driveMotor.setInverted(true);
        steerMotor.setInverted(false);

        driveMotor.enableVoltageCompensation(12);

        // steerMotor.getPIDController().setP(0.1);
        // steerMotor.getPIDController().setI(0.0);
        // steerMotor.getPIDController().setD(1.0);

        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

        tab.addDouble("Absolute Angle", steerEncoder::getAbsolutePosition); 
        tab.addDouble("Current Angle", () -> Math.toDegrees(steerMotor.getEncoder().getPosition()));
        tab.addDouble("Target Angle", () -> desiredAngle);
        tab.addDouble("Velocity", steerMotor.getEncoder()::getVelocity);
    }

    public void resetDrivePosition() {
        driveMotor.getEncoder().setPosition(0);
    }

    public void resetSteerPosition(){
        steerMotor.getEncoder().setPosition(Math.toRadians(steerEncoder.getAbsolutePosition()));
    }

    public double getDrivePosition(){
        return driveMotor.getEncoder().getPosition();
    }

    public double getSteerAngle(){
        return steerMotor.getEncoder().getPosition(); // May Switch to absolute Encoder
    }

    public void set(double driveVolts, double targetAngle){
        resetSteerPosition();
        
        // Put in range of [0, 360)
        targetAngle %= 360;
        targetAngle += (targetAngle < 0.0) ? 360 : 0;

        desiredAngle = targetAngle;

        double diff = targetAngle - steerEncoder.getAbsolutePosition();

        if (diff > 90 || diff < -90){ // move to a closer angle and drive backwards 
            targetAngle = (targetAngle + 180) % 360; 
            driveVolts *= -1.0;
        }

        driveMotor.setVoltage(driveVolts);
        // steerMotor.getPIDController().setReference(Math.toRadians(targetAngle), ControlType.kPosition);
    }
}