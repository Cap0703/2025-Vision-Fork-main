package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase{
    public Command test(){
        return AutoBuilder.buildAuto("newtest");
    }
    
}