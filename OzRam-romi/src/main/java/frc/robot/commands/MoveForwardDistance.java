// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/** An example command that uses an example subsystem. */
public class MoveForwardDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;

//fix it 
// midpoint = m_subsystem.getRightDistanceInch + m_subsystem.getLeftDistanceInch / 2;
  



/**
   * Creates a new MoveForward.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveForwardDistance(RomiDrivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( m_subsystem.getRightDistanceInch()  >=6.2 &&  m_subsystem.getLeftDistanceInch() >= 6.2) 
   { m_subsystem.drive(-0.3, -0.3);}
    if( m_subsystem.getRightDistanceInch()  <=6 &&  m_subsystem.getLeftDistanceInch() <= 6)
    {m_subsystem.drive(0.3, 0.3);}
  }
  //m_subsystem.getRightDistanceInch + m_subsystem.getLeftDistanceInch / 2

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    m_subsystem.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    
   if( m_subsystem.getRightDistanceInch()  >=6 &&  m_subsystem.getLeftDistanceInch() >= 6)
     {return true;}
      return false;
  }
}
