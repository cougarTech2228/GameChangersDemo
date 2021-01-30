/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Toolkit.CT_CommandToggler;
import frc.robot.Toolkit.CT_DigitalInput;
import frc.robot.util.TrajectoryManager;
import frc.robot.Toolkit.CT_CommandToggler.CommandState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Initializes the xbox controller at port 0
  private final static OI m_oi = new OI();

  
  // Robot Subsystems
  private final static DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();
  
  // TrajectoryManager must be instantiated after DrivebaseSubsystem since it relies on it
  private final static TrajectoryManager m_trajectoryManager = new TrajectoryManager();
  
  private final static VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final static LidarSubsystem m_lidarSubsystem = null;//new LidarSubsystem();
  private final static AcquisitionSubsystem m_acquisitionSubsystem = null;//new AcquisitionSubsystem();
  private final static StorageSubsystem m_storageSubsystem = null;//new StorageSubsystem();
  private final static ShooterSubsystem m_shooterSubsystem = null;//new ShooterSubsystem(m_storageSubsystem, m_lidarSubsystem, m_acquisitionSubsystem);

  private final static SendableChooser<Double> m_manualVelocityChooser = new SendableChooser<>();
  private final static SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private static Command m_basicTrajectoryCommand;
  private static Command m_barrelRacingTrajectoryCommand;
  private static Command m_slalomTrajectoryCommand;
  private static Command m_bounceTrajectoryCommand;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    DriverStation.getInstance().silenceJoystickConnectionWarning(true);
    
    // The import of trajectories from PathWeaver-generated json files can be very
    // time consuming so we're putting the Trajectory Manager into a thread.
    Thread thread = new Thread(m_trajectoryManager);
    thread.start();

    // Configure the button and shuffleboard bindings
    configureButtonBindings();
    configureShuffleboardBindings();
  }

  // Use this method to define shuffleboard buttons or widgets
  private void configureShuffleboardBindings() {

    // Velocity Options

    SmartDashboard.putData("Velocity Chooser", m_manualVelocityChooser);
    m_manualVelocityChooser.setDefaultOption("Use Lidar", -1.0);
    m_manualVelocityChooser.addOption("Initiation Line", 75000.0);
    m_manualVelocityChooser.addOption("Trench", 77000.0);
    m_manualVelocityChooser.addOption("Control Panel", 85000.0);
    m_manualVelocityChooser.addOption("Diagnostic Velocity", 30000.0);

    
    /*
     * // Command Buttons SmartDashboard.putData("Cancel all commands", new
     * InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
     * SmartDashboard.putData("Reset Drum Array", new
     * PrintCommand("resetting drum array").andThen(() ->
     * m_storageSubsystem.resetBallArray()));
     * SmartDashboard.putData("Repopulate drum array", getRepopulateArrayCommand());
     * 
     * // Diagnostic buttons SmartDashboard.putData("Rotate Drum Forwards", new
     * InstantCommand(m_storageSubsystem::startDrumMotor, m_storageSubsystem));
     * SmartDashboard.putData("Rotate Drum Backwards", new
     * InstantCommand(m_storageSubsystem::startDrumMotorBackwards,
     * m_storageSubsystem)); SmartDashboard.putData("Stop Drum Motor", new
     * InstantCommand(m_storageSubsystem::stopDrumMotor, m_storageSubsystem));
     * SmartDashboard.putData("Change mode to shooting", new InstantCommand(() ->
     * m_shooterSubsystem.setIsShooting(true), m_shooterSubsystem));
     * SmartDashboard.putData("Change mode to acquiring", new InstantCommand(() ->
     * m_shooterSubsystem.setIsShooting(false), m_shooterSubsystem));
     * SmartDashboard.putData("Rotate drum one index",
     * getIndexDrumCommand(m_storageSubsystem.getDrumStoragePositionInput(), true));
     * SmartDashboard.putData("Bopper", getBopperCommand());
     * SmartDashboard.putData("Wiggle", getShakeDialCommand());
     * SmartDashboard.putData("Run Acquirer Motor", new InstantCommand(() ->
     * m_acquisitionSubsystem.startAcquirerMotor()));
     * SmartDashboard.putData("Stop Acquirer Motor", new
     * InstantCommand(m_acquisitionSubsystem::stopAcquirerMotor));
     * SmartDashboard.putData("Deploy Acquirer", new
     * InstantCommand(m_acquisitionSubsystem::deployAcquirer));
     * SmartDashboard.putData("Retract Acquirer", new
     * InstantCommand(m_acquisitionSubsystem::retractAcquirer));
     * SmartDashboard.putData("Reverse Acquirer", new
     * InstantCommand(m_acquisitionSubsystem::startAcquirerMotorReverse));
     */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Button rightTrigger = new Button(OI::getXboxRightTriggerPressed);
    Button leftTrigger = new Button(OI::getXboxLeftTriggerPressed);
    Button rightBumper = new Button(OI::getXboxRightBumper);
    Button leftBumper = new Button(OI::getXboxLeftBumper);

    Button aButton = new Button(OI::getXboxAButton);
    Button bButton = new Button(OI::getXboxBButton);
    Button xButton = new Button(OI::getXboxXButton);
    Button yButton = new Button(OI::getXboxYButton);

    // --------------------------------------Acquirer
    // Buttons-----------------------------------------------------
    

     //rightTrigger.whenPressed(() -> m_acquisitionSubsystem.startAcquirerMotor());
      //rightTrigger.whenReleased(() -> m_acquisitionSubsystem.stopAcquirerMotor());
      
      //leftTrigger.whenPressed(() -> m_acquisitionSubsystem.startAcquirerMotorReverse());
      //leftTrigger.whenReleased(() -> m_acquisitionSubsystem.stopAcquirerMotor());
     /* 
     * new CT_CommandToggler( // Acquirer Motor Toggle - Right Bumper new
     * InstantCommand(m_acquisitionSubsystem::deployAcquirer), new
     * InstantCommand(m_acquisitionSubsystem::retractAcquirer) )
     * .setDefaultState(CommandState.Interruptible) .setToggleButton(rightBumper)
     * .setCycle(true);
     */
    // --------------------------------------Shooter
    // Buttons--------------------------------------------------------
    /*
     * new CT_CommandToggler( // Shooter Motor Toggle - Left Bumper new
     * InstantCommand(m_shooterSubsystem::startShooterMotor, m_shooterSubsystem),
     * new InstantCommand(m_shooterSubsystem::stopShooterMotor, m_shooterSubsystem)
     * ) .setDefaultState(CommandState.Interruptible) .setToggleButton(leftBumper)
     * .setCycle(true);
     * 
     * new CT_CommandToggler() // Shoot Entire Drum Toggle - A Button
     * .setDefaultState(CommandState.Interruptible) .addJumpCommand(
     * getShootEntireDrumCommand().beforeStarting(() ->
     * m_shooterSubsystem.setIsShooting(true)), CommandState.Interruptible )
     * .addCommand(null) .setToggleButton(aButton) .setCycle(true);
     */
    // --------------------------------------Dial
    // Buttons-----------------------------------------------------------
    /*
     * bButton.whenPressed(getShakeDialCommand());
     * yButton.whenPressed(getRepopulateArrayCommand());
     */
    // --------------------------------------Other
    // Buttons----------------------------------------------------------
    xButton.whenPressed(new PrintCommand("X Button Pressed"));
  }

  public static void configureAutoChooser() {
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    m_autoChooser.setDefaultOption("Barrel Race", m_barrelRacingTrajectoryCommand);
    m_autoChooser.addOption("Slalom", m_slalomTrajectoryCommand);
    m_autoChooser.addOption("Bounce", m_bounceTrajectoryCommand);
  }

  public static OI getOI() {
    return m_oi;
  }

  // Command Getters

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("Auto: " + m_autoChooser.getSelected().getName());
    return m_autoChooser.getSelected();
  }

  // Utilization Commands

  public static RumbleCommand getRumbleCommand(double time) {
    return new RumbleCommand(time);
  }

  public static Double getManualVelocity() {
    return m_manualVelocityChooser.getSelected();
  }

  // Storage Commands

  public static IndexDrumCommand getIndexDrumCommand(CT_DigitalInput input, boolean ignoreCell) {
    // if(m_shooterSubsystem.getIsShooting()) {
    // return new IndexDrumCommand(m_storageSubsystem,
    // m_storageSubsystem.getDrumShooterPositionInput(), ignoreCell);
    // } else {
    return new IndexDrumCommand(m_storageSubsystem, input, ignoreCell);
    // }
  }

  // Shooting Commands

  public static TryToShootCommand getTryToShootCommand() {
    return new TryToShootCommand(m_shooterSubsystem, m_storageSubsystem);
  }

  public static ShootEntireDrumCommand getShootEntireDrumCommand() {
    return new ShootEntireDrumCommand(m_shooterSubsystem, m_storageSubsystem);
  }

  public static BopperCommand getBopperCommand() {
    return new BopperCommand(m_shooterSubsystem);
  }

  // Diagnostic Commands

  public static RepopulateArrayCommand getRepopulateArrayCommand() {
    return new RepopulateArrayCommand(m_storageSubsystem, m_shooterSubsystem);
  }

  public static ShakeDialCommand getShakeDialCommand() {
    return new ShakeDialCommand(m_storageSubsystem);
  }

  // Subsystem Getters

  public static DrivebaseSubsystem getDrivebaseSubsystem() {
     return m_drivebaseSubsystem;
  }

  public static AcquisitionSubsystem getAcquisitionSubsystem() {
    return m_acquisitionSubsystem;
  }

  public static StorageSubsystem getStorageSubsystem() {
    return m_storageSubsystem;
  }

  public static ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  public static VisionSubsystem getVisionSubsystem() {
    return m_visionSubsystem;
  }

  public static LidarSubsystem getLidarSubsystem() {
    return m_lidarSubsystem;
  }

  public static void setBasicTrajectoryCommand(CommandBase command) {
    m_basicTrajectoryCommand = command;
  }

  public static void setBarrelRacingTrajectoryCommand(CommandBase command) {
    m_barrelRacingTrajectoryCommand =  command.withName("Barrel Racing");
  }
  
  public static void setSlalomTrajectoryCommand(CommandBase command) {
    m_slalomTrajectoryCommand = command.withName("Slalom");
  }

  public static void setBounceTrajectoryCommand(CommandBase command) {
    m_bounceTrajectoryCommand = command.withName("Bounce");
  }

}
