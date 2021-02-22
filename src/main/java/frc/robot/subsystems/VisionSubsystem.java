package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
    private String selectedAuto = null;
    NetworkTable visionModeTable = NetworkTableInstance.getDefault().getTable("Vision Mode");
    NetworkTableEntry selected = visionModeTable.getEntry("selected");
    
    public VisionSubsystem() {
        register();
        
    }

    @Override
    public void periodic() {
        String selection = RobotContainer.getAutoChooserOption();
        
        
        if(selection != null &&
            !selection.equals (selectedAuto)){
                
                System.out.println(selection);
                selectedAuto = selection;
                selected.setString(selection);
        }
    }
}
