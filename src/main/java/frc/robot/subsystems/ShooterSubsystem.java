package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMotor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterMotor m_shooterMotor;
    private Solenoid m_bopper;
    private LidarSubsystem m_lidarSubsystem;
    private boolean m_isShooting;

    public ShooterSubsystem(LidarSubsystem lidarSubsystem) {
        register();

        m_lidarSubsystem = lidarSubsystem;

        m_shooterMotor = new ShooterMotor();
        m_bopper = new Solenoid(Constants.PCM_CAN_ID, Constants.BOPPER_PCM_PORT);

        m_isShooting = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMotor.getSpeed());
        SmartDashboard.putBoolean("Is Robot Shooting", m_isShooting);
    }

    /**
     * Returns the opposite value of the getter for the sensor as for example if the
     * getter returns true that means the sensor is not blocked.
     * 
     * @return if the shooter slot is occupied by a powercell
     */
    // public boolean isShooterCellOccupied() {
    //     return !m_cellInput.get();
    // }

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
        System.out.println("Setting is shooting to: " + isShooting);
        m_isShooting = isShooting;
    }

    /**
     * Starts the shooter motor and runs the velocity adjustment command
     */
    public void startShooterMotor() {
        double currentMoveSpeed = RobotContainer.getDrivebaseSubsystem().getCurrentMoveSpeedAverage();

        if (currentMoveSpeed < 0.5 && currentMoveSpeed > -0.5) { // make sure the robot is lower than half speed
            m_shooterMotor.start((int) m_lidarSubsystem.getLidarAverage());
            //RobotContainer.getShooterMotorAdjustmentCommand().schedule();
        } else {
            System.out.println("Robot is running too fast to start shooter motor");
        }
    }

    /**
     * Starts the shooter motor, also sets the variable isShooting in the storage
     * subsystem and in the shooter subsystem to false. Rotates the drum back to
     * acquire position.
     */
    public void stopShooterMotor() {
        m_shooterMotor.stop();
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