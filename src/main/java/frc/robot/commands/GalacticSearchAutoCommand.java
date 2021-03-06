package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;



/**
 * GalacticSearchAutoCommand
 * 
 * Will use vision to determine which path is the correct one, and run the correct path based on that.
 * 
 */
public class GalacticSearchAutoCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    NetworkTable gsTable = ntinst.getTable("GalaticSearch");
    NetworkTableEntry path = gsTable.getEntry("path");
    NetworkTableEntry neuralNetwork = gsTable.getEntry("Neural");
    NetworkTableEntry neuralNetworkConf = gsTable.getEntry("NeuralConf");


    public GalacticSearchAutoCommand(VisionSubsystem visionSubsystem) {
        addCommands(
            new PrintCommand("Running gs"),
            new SelectCommand(
                Map.ofEntries(
                    Map.entry("aRed", RobotContainer.getGalacticSearchTrajectoryCommands()[2]),
                    Map.entry("aBlue", RobotContainer.getGalacticSearchTrajectoryCommands()[0]),
                    Map.entry("bRed", RobotContainer.getGalacticSearchTrajectoryCommands()[3]),
                    Map.entry("bBlue", RobotContainer.getGalacticSearchTrajectoryCommands()[1])

                ), new Supplier<Object>(){
                    @Override
                    public Object get() {

                        String bridgettePath = path.getString("noPath");
                        String neuralPath = neuralNetwork.getString("noPath");
                        Double neuralConf = neuralNetworkConf.getDouble(-1);

                        System.out.println(neuralConf + "% sure it's " + neuralPath + ", Vision says it's " + bridgettePath);

                        /*if((!bridgettePath.equals("noPath")) && (neuralConf <= 95)){
                            System.out.println("Running bridgettePath");
                            return bridgettePath;
                        }
                        else{
                            System.out.println("Running neuralPath");
                            return neuralPath;
                        }*/
                        
                        return neuralPath;
                        
                    }
                }
            )
        );
    }
}
