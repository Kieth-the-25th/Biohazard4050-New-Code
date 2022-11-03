package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.commands.CannonCommand;
import frc.commands.DriveCommand;
import frc.subsystems.Drivetrain;
import frc.subsystems.Shooter;

public class RobotCmdManager {
    public final XboxController m_controller = new XboxController(0);
    public final Drivetrain m_drive = new Drivetrain();
    public final DriveCommand c_drive = new DriveCommand(m_drive, m_controller);
    public final Shooter s_cannon = new Shooter();
    public final CannonCommand c_cannon = new CannonCommand(s_cannon, m_controller);

    public RobotCmdManager() {
        /**
         * Put other robot classes that need initialized here?
         */
    }

    public void ScheduleCommands() {
        //Schedules cannon & drive commands
        CommandScheduler.getInstance().schedule(true, c_cannon, c_drive);
    }

    public void RunCommands() {
        //Runs all scheduled commands
        CommandScheduler.getInstance().run();
    }

    public void AutonomousUpdate() {
        m_drive.updateOdometry();
    }
}
