package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Toolkit.CT_LEDStrip;
import frc.robot.Toolkit.CT_LEDStrip.ColorPattern;
import frc.robot.Toolkit.CT_LEDStrip.Speed;
import frc.robot.util.ShooterMotor;
import frc.robot.util.ShooterMotor.ShootingType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ShooterSubsystem extends SubsystemBase {

    private LEDSubsystem m_ledSubsystem;
    private ShooterMotor m_shooterMotor;
    private Solenoid m_bopper;
    private boolean m_isShooting;
    private boolean m_isMotorUpToSpeed;
    public int m_lobDistance; 
    public int m_targetDistance;
    private CT_LEDStrip m_led;

    public ShooterSubsystem(LEDSubsystem ledSubsystem) {
        register();

        m_shooterMotor = new ShooterMotor(this);
        m_bopper = new Solenoid(Constants.PCM_CAN_ID, Constants.BOPPER_PCM_PORT);

        m_ledSubsystem = ledSubsystem;
        m_isShooting = false;
        m_isMotorUpToSpeed = false;
        m_lobDistance = Constants.LOB_SHOOT_MIN_DISTANCE;
        m_targetDistance = Constants.TARGET_SHOOT_MIN_DISTANCE;
        m_led = ledSubsystem.m_led;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Calculated Shooter Velocity", m_shooterMotor.getVelocity());
        SmartDashboard.putBoolean("Is Robot Shooting", m_isShooting);
        SmartDashboard.putNumber("Lidar Distance: ", RobotContainer.getLidarManager().getLidarAverage());
        SmartDashboard.putBoolean("Is shooter motor up to speed", m_isMotorUpToSpeed);
        SmartDashboard.putNumber("Lob Distance (ft)", m_lobDistance);
        SmartDashboard.putNumber("Target Distance (ft)", m_targetDistance);
        SmartDashboard.putString("Shooting Type", m_shooterMotor.getShootingType().toString());

        m_isMotorUpToSpeed = false;
        if(m_isShooting) {
            if(((m_shooterMotor.getSelectedSensorVelocity()) + 3000) > m_shooterMotor.getVelocity()) {
                m_isMotorUpToSpeed = true;
            }
        
            if(m_shooterMotor.getShootingType() == ShootingType.LobShoot) {
                if(m_lobDistance == 5) {
                    m_led.doMovingColors(Speed.Slow, m_ledSubsystem.bluePattern);
                } else if (m_lobDistance == 10) {
                    m_led.doMovingColors(Speed.Fast, m_ledSubsystem.greenPattern);
                } else if (m_lobDistance == 15) {
                    m_led.doMovingColors(Speed.VeryFast, m_ledSubsystem.yellowPattern);
                } else {
                    m_led.doMovingColors(Speed.Ridiculous, m_ledSubsystem.redPattern);
                }
            } else {
                if(m_targetDistance == 10) {
                    m_led.doMovingColors(Speed.Slow, m_ledSubsystem.bluePattern);
                } else if (m_targetDistance == 15) {
                    m_led.doMovingColors(Speed.Fast, m_ledSubsystem.greenPattern);
                } else if (m_targetDistance == 20) {
                    m_led.doMovingColors(Speed.VeryFast, m_ledSubsystem.yellowPattern);
                } else {
                    m_led.doMovingColors(Speed.Ridiculous, m_ledSubsystem.redPattern);
                }
            }

        }
    }

    public boolean isShooterMotorUpToSpeed() {
        return m_isMotorUpToSpeed;
    }

    /**
     * Raises the solenoid which pushes the powercell into the shooter motor
     */
    public void raiseBopper() {
        m_bopper.set(true);
    }

    /**
     * Lowers the solenoid
     */
    public void lowerBopper() {
        m_bopper.set(false);
    }

    /**
     * Returns the boolean isShooting which determines the state of the drum slot
     * location
     * 
     * @return boolean isShooting
     */
    public boolean isShooting() {
        return m_isShooting;
    }

    /**
     * Sets isShooting to the passed in value
     * 
     * @param isShooting
     */
    public void setIsShooting(boolean isShooting) {
        //System.out.println("Setting is shooting to: " + isShooting);
        m_isShooting = isShooting;
    }

    /**
     * Starts the shooter motor and runs the velocity adjustment command
     */
    public void startShooterMotor() {
        
        m_shooterMotor.start(this);
        //m_isShooting = true;
    }

    /**
     * Starts the shooter motor, also sets the variable isShooting in the storage
     * subsystem and in the shooter subsystem to false. Rotates the drum back to
     * acquire position.
     */
    public void stopShooterMotor() {
        RobotContainer.getStorageSubsystem().getCompressor().start();
        m_shooterMotor.stopMotor();
        m_isShooting = false;
    }

    /**
     * Gets the shooter motor
     * 
     * @return the shooter motor
     */
    public ShooterMotor getShooterMotor() {
        return m_shooterMotor;
    }
}