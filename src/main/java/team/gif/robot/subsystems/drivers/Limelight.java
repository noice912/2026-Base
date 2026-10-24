package team.gif.robot.subsystems.drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable table;

    public static final int LED_PIPELINE = 0;
    public static final int LED_OFF      = 1;
    public static final int LED_BLINK    = 2;
    public static final int LED_ON       = 0;

    public static final int MODE_TRACKING  = 0;
    public static final int MODE_CAMERA    = 1;
    public static int test = 0;

    public double _mountingAngleLLDegrees = -1;
    public double _lensHeightInches = -1;
    public double _goalHeightInches = -1;
    public double _offsetInches = 0;

    public Limelight(String key) {
        table = NetworkTableInstance.getDefault().getTable(key);
        setLEDMode(Limelight.LED_ON);
    }

    public Limelight() {
        this("limelight");
    }

    public void setLEDMode(int mode) {
        if (mode >= 0 && mode <= 3) {
            table.getEntry("ledMode").setNumber(mode);
        }
    }

    public void setLEDOn() {
        setLEDMode(3);
    }

    public void setLEDOff() {
        setLEDMode(1);
    }

    public void setCamMode(int mode) {
        if (mode >= 0 && mode <= 1) {
            table.getEntry("camMode").setNumber(mode);
            System.out.println("cammode reset " + mode);
        }
    }

    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            table.getEntry("pipeline").setNumber(pipeline);
        }
    }

    public void setStreamMode(int mode) {
        if (mode >= 0 && mode <= 2) {
            table.getEntry("stream").setNumber(mode);
        }
    }

    public void setSnapshotMode(int mode) {
        if (mode >= 0 && mode <= 1) {
            table.getEntry("snapshot").setNumber(mode);
        }
    }

    public boolean hasTarget() {
        return (double)  (table.getEntry("tv").getNumber(0)) == 1;
    }

    public double getXOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getYOffset() {return table.getEntry("ty").getDouble(0.0);}

    public double getArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double getSkew() {
        return table.getEntry("ts").getDouble(0.0);
    }

    public double getLatency() {
        return table.getEntry("tl").getDouble(0.0);
    }

    public int getShortLength() {
        return table.getEntry("tshort").getNumber(0).intValue();
    }

    public int getLongLength() {
        return table.getEntry("tlong").getNumber(0).intValue();
    }

    public int getHorizLength() {
        return table.getEntry("thoriz").getNumber(0).intValue();
    }

    public int getVertLength() {
        return table.getEntry("tvert").getNumber(0).intValue();
    }

    public double[] getCamTran() {
        return table.getEntry("camtran").getDoubleArray(new double[]{0, 0, 0, 0, 0, 0});
    }

    public double getCustomValue(String key) {
        return table.getEntry(key).getDouble(0.0);
    }

    public double[] getTcornx() {
        return table.getEntry("tcornx").getDoubleArray(new double[]{0, 0, 0, 0});
    }

    public double[] getTcorny() {
        return table.getEntry("tcorny").getDoubleArray(new double[]{0, 0, 0, 0});
    }

    public void setDistanceEstimatorParams(double mountingAngleLLDegrees,
                                           double lensHeightInches,
                                           double goalHeightInches,
                                           double offsetInches) {
        _mountingAngleLLDegrees = mountingAngleLLDegrees;
        _lensHeightInches = lensHeightInches;
        _goalHeightInches = goalHeightInches;
        _offsetInches = offsetInches;
    }

    public double getDistance() {
        if (_mountingAngleLLDegrees == -1 ||
            _lensHeightInches == -1 ||
            _goalHeightInches == -1 ){
            return -2;
        }
        return DistanceEstimator(_mountingAngleLLDegrees,_lensHeightInches,_goalHeightInches,_offsetInches);
    }

    public double DistanceEstimator(double mountingAngleLLDegrees, double lensHeightInches, double goalHeightInches, double offsetInches) {
        if (!hasTarget()) {
            return -1;
        }

        double targetOffsetAngle_Vertical = getYOffset();
        double angleToGoalDegrees = mountingAngleLLDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        return offsetInches + (goalHeightInches - lensHeightInches) / Math.tan(angleToGoalRadians);
    }
}
