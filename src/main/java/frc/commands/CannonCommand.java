package frc.commands;

import java.lang.reflect.Field;
import java.util.Collection;
import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Shooter;

public class CannonCommand extends CommandBase
{
    private XboxController controller;
    private final Shooter m_shooter;
    private final String name = "Cannon Command";
    
    public CannonCommand(Shooter shooter, XboxController newController)
    {
        addRequirements(shooter);
        m_shooter = shooter;
        controller = newController;
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute() 
    {
        m_shooter.SetActive(controller.getAButton());
    }

    @Override
    public boolean isFinished() 
    {
        //This command runs once
        return true;
    }

    @Override
    public String getName() {
        return name;
    }
}
