// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import org.ejml.equation.Variable;
import org.ejml.equation.VariableDouble;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/** An example command that uses an example subsystem. */
public class MoveForwardDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
 




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
     Double midpoint = m_subsystem.getRightDistanceInch() + m_subsystem.getLeftDistanceInch() /2   ;
  
    m_subsystem.resetEncoders();

  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  
    Double midpoint =( m_subsystem.getRightDistanceInch() + m_subsystem.getLeftDistanceInch() ) /2;
  m_subsystem.drive((9-midpoint)*0.1+0.05, (9-midpoint)*0.1 +0.05); // make the +0.05 pos/neg

  

    /*if (m_subsystem.getLeftSpeed() <= 0.2  && midpoint <9 &&  midpoint > 8.8) {m_subsystem.drive(0.2, 0.2);}
    if (m_subsystem.getLeftSpeed() <= 0.2  && midpoint <9.2 &&  midpoint > 9) {m_subsystem.drive(-0.2, -0.2);} */
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
    Double midpoint =( m_subsystem.getRightDistanceInch() + m_subsystem.getLeftDistanceInch() ) /2;
   
    if (midpoint <9.1 && midpoint > 8.9 
    &&  Math.abs( m_subsystem.getLeftSpeed() ) <=0.15)
    {return true;}
  
      return false;
  }
}
