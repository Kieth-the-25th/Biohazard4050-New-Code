package frc.commands;

import java.math.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Drivetrain;

public class DriveCommand extends CommandBase
{
    private final Drivetrain m_drivetrain;
    private XboxController controller;
    private final String name = "Driving Command";

    public DriveCommand(Drivetrain drivetrain, XboxController newController) 
    {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        controller = newController;
    }

    @Override
    public void initialize() 
    {
        
    }

    @Override
    public void execute()
    {
        m_drivetrain.drive(Math.acos(controller.getLeftX()), controller.getRightY());
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
