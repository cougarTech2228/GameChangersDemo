package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;



public class TargetCorrectionCommand extends CommandBase {
    // The subsystem the command runs on
    private final DrivebaseSubsystem m_drivebaseSubsystem;
    private NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntinst.getTable("PowerTower");

    private NetworkTableEntry difference = table.getEntry("difference");
    public TargetCorrectionCommand(DrivebaseSubsystem subsystem) {
        m_drivebaseSubsystem = subsystem;
      addRequirements(m_drivebaseSubsystem);
    }
  
    @Override
    public void initialize() {
    }
  
    @Override
    public boolean isFinished() {
        difference = table.getEntry("difference");
        double diff = difference.getDouble(-1000);
        if ((diff < 40.0) && (diff > -40.0)){
            if (diff > 5.0){ 
                System.out.println("Turn left");
                m_drivebaseSubsystem.turnLeft(0.1);
            }
            else if(diff < -5.0){
                System.out.println("Turn right");
                m_drivebaseSubsystem.turnRight(0.1);
            }
            else{
                return true;
            }

        }

      return false;
    }

    @Override
    public void end(boolean interuppted){
        m_drivebaseSubsystem.stop();
    }
  }
