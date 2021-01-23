package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.BallArray;
import frc.robot.Toolkit.CT_DigitalInput;
import frc.robot.commands.IndexDrumCommand;

public class StorageSubsystem extends SubsystemBase {

    private CT_DigitalInput m_cellStorageInput;
    private CT_DigitalInput m_drumStoragePositionInput;
    private CT_DigitalInput m_drumShooterPositionInput;
    private Spark m_drumSparkMotor;
    private boolean m_isFull;
    private boolean m_isRepopulating;
    private BallArray m_ballArray = new BallArray();

    public StorageSubsystem() {
        register();

        m_drumSparkMotor = new Spark(Constants.DRUM_SPARK_PWM_ID);

        m_drumStoragePositionInput = new CT_DigitalInput(Constants.ACQUIRE_POSITION_DIO);
        m_drumStoragePositionInput.setInterrupt(() -> endIndexDrum(m_drumStoragePositionInput), true, false);
        m_drumStoragePositionInput.setInterruptLatched(false);

        m_drumShooterPositionInput = new CT_DigitalInput(Constants.SHOOTER_POSITION_DIO);
        m_drumShooterPositionInput.setInterrupt(() -> endIndexDrum(m_drumShooterPositionInput), true, false);
        m_drumShooterPositionInput.setInterruptLatched(false);

        m_cellStorageInput = new CT_DigitalInput(Constants.ACQUIRE_BALL_DIO, true);

        m_cellStorageInput.setInterrupt(new IndexDrumCommand(this, m_drumStoragePositionInput, false), true, false);
        

        

        m_isFull = false;
        m_isRepopulating = false;
        SendableRegistry.add(m_ballArray, "balls");

        Shuffleboard.getTab("drum")
        .add(m_ballArray)
        .withWidget("DrumWidget");

    }

    /**
     * Will index the drum by spinning the motor and latching the position checker after 0.2 seconds.
     * 
     * @param input Digital Input that will be latched
     * @param ignoreCell will ignore a detected cell, should only be true when calling 
     *                   this method from a shuffleboard button.
     */
    // public void indexDrum(CT_DigitalInput input, boolean ignoreCell) {
        
    //     new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             System.out.println("Index drum because cell input interrupted");

    //             if(m_cellStorageInput.getStatus() && !ignoreCell) {
    //                 System.out.println("Acquiring ball");
    //                 m_ballArray.acquire();
    //             }
    //             m_ballArray.rotate();

    //             startDrumMotor();
    //         }),
    //         new WaitCommand(0.2),
    //         new InstantCommand(() -> input.setInterruptLatched(true))
    //     ).schedule();
    // }

/**
 * Ends the indexing of the drum by stopping the motor, checking if the drum is full, and unlatching the interrupt.
 * 
 * @param input Digital Input whose interrupt will be unlatched
 */
    public void endIndexDrum(CT_DigitalInput input) {
        stopDrumMotor();
        checkIfDrumFull();
        input.setInterruptLatched(false);
    }

    @Override
    public void periodic() {

        // Conditions for the drum to be able to index
        boolean[] drumConditions = {
            !RobotContainer.getShooterSubsystem().getIsShooting(), // Robot is not shooting, 
            !m_isFull, // Robot is not full,
            !m_isRepopulating // Robot is not repopulating
        };

        m_cellStorageInput.onlyHandleInterruptsWhen(drumConditions);

        SmartDashboard.putBoolean("Is Robot Full", m_isFull);
        SmartDashboard.putBoolean("Is Acquire Flag Tripped", !m_drumStoragePositionInput.get());
        SmartDashboard.putBoolean("Is Acquire Slot Occupied", !m_cellStorageInput.get());
        SmartDashboard.putBoolean("Is Robot Empty", m_ballArray.isEmpty());
    }

    /**
     * Set the repopulating variable
     * 
     * @param isRepopulating the value to be set
     */
    public void setIsRepopulating(boolean isRepopulating) {
        m_isRepopulating = isRepopulating;
    }

    /**
     * Determines if the acquierer slot is occupied
     * 
     * @return if the the acquierer slot is occupied
     */
    public boolean isAcquireBallOccupied() {
        return !m_cellStorageInput.get();
    }

    /**
     * Sets all elements in drumArray to false Sets m_isFull to false
     */
    public void resetBallArray() {
        System.out.println("reset drum");
        m_ballArray.data = 0;
    }

    /**
     * Starts the drum spark motor
     */
    public void startDrumMotor() {
        // System.out.println("start drum motor");
        m_drumSparkMotor.set(-Constants.DRUM_MOTOR_VELOCITY);
    }

    /**
     * Starts the drum spark motor backwards
     */
    public void startDrumMotorBackwards() {
        m_drumSparkMotor.set(Constants.DRUM_MOTOR_VELOCITY);
    }

    /**
     * Stops the drum spark motor
     */
    public void stopDrumMotor() {
        // System.out.println("stop drum motor");
        m_drumSparkMotor.set(0);
    }

    /**
     * Gets the ball array
     * 
     * @return the ball array instance
     */
    public BallArray getBallArray() {
        return m_ballArray;
    }

    /**
     * Gets the storage position digital input object
     * 
     * @return the storage position digital input object
     */
    public CT_DigitalInput getDrumStoragePositionInput() {
        return m_drumStoragePositionInput;
    }

    /**
     * Gets the shooter position digital input object
     * 
     * @return the shooter position digital input object
     */
    public CT_DigitalInput getDrumShooterPositionInput() {
        return m_drumShooterPositionInput;
    }

    /**
     * Gets the cell input onject
     * 
     * @return the cell input object
     */
    public CT_DigitalInput getCellInput() {
        return m_cellStorageInput;
    }

    /**
     * Checks if the drum is full (If all elements are true) Sets boolean m_isFull
     * to corresponding value
     */
    public void checkIfDrumFull() {
        m_isFull = m_ballArray.isFull();
    }
}