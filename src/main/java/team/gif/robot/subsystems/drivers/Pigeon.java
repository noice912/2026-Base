package team.gif.robot.subsystems.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Pigeon {

    public static PigeonIMU _pigeon;

    private final PigeonIMU.GeneralStatus _pigeonGenStatus = new PigeonIMU.GeneralStatus();

    private double pitchOffset = 0;

    public Pigeon(int PigeonID) {
        _pigeon = new PigeonIMU(PigeonID);
    }

    public Pigeon(TalonSRX talon) {
        _pigeon = new PigeonIMU(talon);
    }

    public void addToShuffleboard(String tabName, String widgetTitle) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        tab.add(widgetTitle, (x) -> {
            x.setSmartDashboardType("Gyro");
            x.addDoubleProperty("Value", () -> getCompassHeading(), null);
        });
    }

    public double getHeading() {
        double[] ypr = new double[3];

        _pigeon.getYawPitchRoll(ypr);

        return ypr[0];
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(getHeading()));
    }

    public double get180Heading() {
        return Math.IEEEremainder(getHeading(), 360.0d);
    }

    public double get360Heading() {
        double heading = getHeading();

        heading = heading < 0 ? 360 + (heading % 360) : heading % 360;

        return heading;
    }

    public double getCompassHeading() {
        double heading = getHeading();

        heading = heading < 0 ? (0 - heading % 360) : (360 - (heading % 360));

        heading = heading == 360.0 ? 0 : heading;

        return heading;
    }

    public boolean isActive() {
        _pigeon.getGeneralStatus(_pigeonGenStatus);

        return _pigeonGenStatus.state == PigeonIMU.PigeonState.Ready;
    }

    public void resetPigeonPosition() {
        System.out.println("resetting pigeon empty");
        resetPigeonPosition(0);
    }

    public void resetPigeonPosition(double angle) {
        System.out.println("resetting pigeon " + angle);
        setYaw(angle);
        zeroPitch();
    }

    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw);
    }

    public void zeroPitch() {
        pitchOffset = getYPR()[1];
    }

    public double[] getYPR() {
        double[] ypr = new double[3];
        _pigeon.getYawPitchRoll(ypr);
        System.out.format("YPR %.1f %.1f %.1f", ypr[0], ypr[1], ypr[2]);
        return ypr;
    }

    public double[] getQuaternions() {
        double[] quaternions = new double[4];
        _pigeon.get6dQuaternion(quaternions);
        return quaternions;
    }

    public double getTemp() {
        return _pigeon.getTemp();
    }

    public double[] getAccumulatedGyro() {
        double[] accumGyro = new double[3];
        _pigeon.getAccumGyro(accumGyro);
        return accumGyro;
    }

    public short[] getBiasedAccel() {
        short[] biasedAccel = new short[3];
        _pigeon.getBiasedAccelerometer(biasedAccel);
        return biasedAccel;
    }

    public double[] getRawGyro() {
        double[] rawGyro = new double[3];
        _pigeon.getRawGyro(rawGyro);
        return rawGyro;
    }

    public double[] getAccelAngles() {
        double[] accelAngles = new double[3];
        _pigeon.getAccelerometerAngles(accelAngles);
        return accelAngles;
    }

    public short[] getBiasedMagnetometer() {
        short[] biasedMagnet = new short[3];
        _pigeon.getBiasedMagnetometer(biasedMagnet);
        return biasedMagnet;
    }

    public short[] getRawMagnet() {
        short[] rawMagnet = new short[3];
        _pigeon.getRawMagnetometer(rawMagnet);
        return rawMagnet;
    }

    public double getPitch() {
        double[] ypr = new double[3];

        _pigeon.getYawPitchRoll(ypr);

        return ypr[1] - pitchOffset;
    }
}
