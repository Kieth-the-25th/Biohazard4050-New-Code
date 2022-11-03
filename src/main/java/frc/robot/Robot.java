// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

//Nice & small
public class Robot extends TimedRobot {
  private RobotCmdManager cmdManager;

  @Override
  public void robotInit() 
  {
    cmdManager = new RobotCmdManager();
  }

  @Override
  public void robotPeriodic() {
    cmdManager.RunCommands();
  }


  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    cmdManager.AutonomousUpdate();
  }

  @Override
  public void teleopPeriodic() {
    cmdManager.ScheduleCommands();
  }
}
