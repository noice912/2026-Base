package team.gif.robot.subsystems.drivers;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon2_0 extends SubsystemBase {

    public static Pigeon2 _pigeon;

    public Pigeon2_0(int PigeonID) {
        _pigeon = new Pigeon2(PigeonID);
    }

    public void addToShuffleboard(String widgetTitle) {
        SmartDashboard.putData(widgetTitle, builder -> {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", this::getCompassHeading, null);
        });
    }

    public double getCompassHeading() {
        double heading = getHeading();

        heading = heading < 0 ? (0 - heading % 360) : (360 - (heading % 360));

        heading = heading == 360.0 ? 0 : heading;

        return heading;
    }

    public double getHeading() {
        return _pigeon.getYaw().getValueAsDouble();
    }

    public double get360Heading() {
        double heading = getHeading();

        heading = heading < 0 ? 360 + (heading % 360) : heading % 360;

        return heading;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(getHeading()));
    }

    public void setYaw(double yaw) {
        _pigeon.setYaw(yaw);
    }

    public double[] getYPR() {
        double[] ypr = new double[3];

        ypr[0] = _pigeon.getYaw().getValueAsDouble();
        ypr[1] = _pigeon.getPitch().getValueAsDouble();
        ypr[2] = _pigeon.getRoll().getValueAsDouble();
        return ypr;
    }

    public void resetPigeonPosition() {
        resetPigeonPosition(0);
    }

    public void resetPigeonPosition(double angle) {
        setYaw( (angle == 0.0) ? 0.0 : 360.0 - angle);
    }

    public double getPitch() {
        return _pigeon.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return _pigeon.getRoll().getValueAsDouble();
    }

    public double getYawRate() {
        return _pigeon.getAngularVelocityXDevice().getValueAsDouble();
    }
}
