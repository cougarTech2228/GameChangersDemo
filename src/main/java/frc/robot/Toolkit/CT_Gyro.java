package frc.robot.Toolkit;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class CT_Gyro {

    private ADXRS450_Gyro m_gyro;
    private PigeonIMU m_pigeon;

    private double[] m_yawPitchRoll = new double[3];
    private GyroType m_gyroType;

    public enum GyroType {
        ADXRS450,
        PigeonImu
    }


    /**
     * ADXRS450 constructor.
     * 
     * @param SPI_Por port that the gyro is connected to.
     */
    public CT_Gyro(Port SPI_Port) {
        m_gyro = new ADXRS450_Gyro(SPI_Port);
        m_gyro.calibrate();
        m_gyroType = GyroType.ADXRS450;
    }

    /**
     * PigeonIMU constructor.
     * 
     * @param can_ID the Can ID the pigeon is connected to.
     */
    public CT_Gyro(int can_ID) {
        m_pigeon = new PigeonIMU(can_ID);
        m_gyroType = GyroType.PigeonImu;
        m_pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    /**
     * Resets the yaw by setting it to 0.
     */
    public void resetYaw() {
        m_pigeon.setYaw(0);
    }
    
    /** 
     * Gets the yaw.
     * 
     * @return the yaw [0, 360].
     */
    public double getYaw() {
        if(m_gyroType == GyroType.PigeonImu) {
            m_pigeon.getYawPitchRoll(m_yawPitchRoll);
            return Math.IEEEremainder(m_yawPitchRoll[0], 360.0);
        } else {
            return Math.IEEEremainder(m_gyro.getAngle(), 360.0);
        }
    }

    /** 
     * PigeonIMU method.
     * Gets the pitch.
     * 
     * @return the pitch within [-90,+90] degrees.
     */
    public double getPitch() {
        m_pigeon.getYawPitchRoll(m_yawPitchRoll);
        return m_yawPitchRoll[1];
    }

    /** 
     * PigeonIMU method.
     * Gets the roll.
     * 
     * @return the roll within [-90,+90] degrees.
     */
    public double getRoll() {
        m_pigeon.getYawPitchRoll(m_yawPitchRoll);
        return m_yawPitchRoll[2];
    }

    /**
     * PigeonIMU method.
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(getYaw(), 360.0d));
    }

}