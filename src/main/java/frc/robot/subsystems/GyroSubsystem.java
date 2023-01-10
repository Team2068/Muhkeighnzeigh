package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
public class GyroSubsystem {
    Pigeon2 sensorPigeon;
    public GyroSubsystem(int deviceNum) {
        sensorPigeon = new Pigeon2(deviceNum);
    }

    public double[] getYawPitchRoll() {
        double buff[] = new double[3];
        sensorPigeon.getYawPitchRoll(buff);
        return buff;
    }

    public double[] getPose() {
        double buff[] = new double[3];
        sensorPigeon.configMountPose(buff[0], buff[1], buff[2]);
        return buff;
    }

    public void setYaw(double Yaw) {
        sensorPigeon.setYaw(Yaw);
    }

    public void addYaw(double Yaw) {
        sensorPigeon.addYaw(Yaw);
    }
}