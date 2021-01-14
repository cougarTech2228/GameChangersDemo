package frc.robot.Toolkit;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;

public class CT_DigitalInput extends DigitalInput {

    private Runnable m_methodToRun;
    private Runnable m_lastMethodToRun;
    private boolean m_isInterruptLatched;
    private boolean m_negateLogic;
    private Runnable m_interruptRunnable;

    /**
     * Negate logic is useful if you want to reverse the output of getStatus(). A use for setting this variable true
     * would be if you were using a light beam sensor and wanted to differentiate between searching for something blocking
     * the sensors, or searching for the absence of something. 
     */ 

    /**
     * Creates a default DigitalIO instance with the ability to negate the logic.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param negateLogic can negate the return value of getStatus() for this instance.
     */
    public CT_DigitalInput(int pin, boolean negateLogic) {
        this(pin, null, negateLogic);
    }

    /**
     * Creates a default DigitalIO instance.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     */
    public CT_DigitalInput(int pin) {
        this(pin, false);
    }

    

    /**
     * Creates a DigitalIO instance with a method.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     */
    public CT_DigitalInput(int pin, Runnable methodToRun) { 
        this(pin, methodToRun, false);
    }

    /**
     * Creates a DigitalIO instance with a method and the ability to negate logic.
     * Used for sensors connected through DIO ports on the RoboRio. Limit Switches, Light Beam sensors, etc.
     * 
     * @param pin the channel the sensor is on.
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     * @param negateLogic can negate the return value of getStatus() for this instance
     */
    public CT_DigitalInput(int pin, Runnable methodToRun, boolean negateLogic) { 
        super(pin);
        m_negateLogic = negateLogic;
        m_methodToRun = methodToRun;
        m_lastMethodToRun = m_methodToRun;
        m_isInterruptLatched = false;
        m_interruptRunnable = null;
    }

    /**
     * Sets the method that will be ran in the runWhenTripped method.
     * 
     * @param methodToRun the method that will be ran in the runWhenTripped method.
     */
    public void setMethodToRun(Runnable methodToRun) {
        m_methodToRun = methodToRun;
        m_lastMethodToRun = m_methodToRun;
    }

    /**
     * Resets the method to run for the runWhenTripped method to use as it won't run 
     * again if a method is not inputted or isn't reseted.
     */
    public void resetMethodToRun() {
        m_methodToRun = m_lastMethodToRun;
    }

    /**
     * Checks to see if the digital IO is tripped depending on if it is acive high or low.
     * If the sensor was tripped, the method either passed in to a constructor or DigitalIO method setMethodToRun will run.
     * This method will not work if an interrupt has been enabled. 
     * The given method will only run ONCE and will have to be reset by the DigitalIO method setMethodToRun.
     * This method is best used when called in the periodic method of a subsystem.
     * 
     * @return If the digital IO was tripped.
     */
    public boolean runWhenTripped() {
        if(getStatus() && !m_isInterruptLatched) {

            if(m_methodToRun != null) {
                Runnable method = m_methodToRun;
                m_methodToRun = null;
                method.run();
                return true;
            } else {
                return false;
            }

        } else {
            return false;
        }
    }

    /**
     * Sets the interrupt runnable.
     * @param runnable the runnable to be used by the interrupt set in the setInterrupt() method.
     */
    public void setInterruptRunnable(Runnable runnable) {
        m_interruptRunnable = runnable;
    }

    /**
     * RISING_EDGE_MASK = 1;
     * FALLING_EDGE_MASK = 256;
     */

    /**
     * Sets an interrupt for the digital input. Can be used in conjunction with the 
     * onlyHandleInterruptsWhen() method to only run the method when certain conditions are met.
     * 
     * @param runnable the runnable that will run when the interrupt is fired.
     * @param interruptOnRisingEdge fire interrupt on the rising edge.
     * @param interruptOnFallingEdge fire interrupt on the falling edge.
     */
    public void setInterrupt(Runnable runnable, boolean interruptOnRisingEdge, boolean interruptOnFallingEdge) {

        m_interruptRunnable = runnable;

        requestInterrupts(new InterruptHandlerFunction<Object>() {

            @Override
            public void interruptFired(int interruptAssertedMask, Object param) {
                if(m_isInterruptLatched) {
                    m_interruptRunnable.run();
                } else { /* Do Nothing */ }

            }
        });

        setUpSourceEdge(interruptOnFallingEdge, interruptOnRisingEdge);
        enableInterrupts();
        m_isInterruptLatched = true;
    }

    /**
     * Enables the interrupts when all of the conditions are true.
     * Use this method in a periodic for full effect.
     * @param conditions the conditions to be met for the interrupt to be activated. 
     */
    public void onlyHandleInterruptsWhen(boolean... conditions) {
        if (isAllTrue(conditions)) {
            m_isInterruptLatched = true;
        } else {
            m_isInterruptLatched = false;
        }
    }

    /**
     * Determines if every value in a boolean array is true.
     * @return whether or not every value is true.
     */
    private boolean isAllTrue(boolean[] array) {
        for (boolean b: array) {
            if (!b) {
                return false;
            }
        }
        return true;
    }

    /**
     * Gets the status of the digital input. 
     * Returned value is negated depending on the value of negateLogic passed into the constructor.
     * If no negateLogic boolean was passed into the constructor, then the logic will not be negated.
     * 
     * @return the status of the digital input.
     */
    public boolean getStatus() {
        if(m_negateLogic)
            return !get();
        else
            return get();
    }

}

