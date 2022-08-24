// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import org.ejml.equation.Variable;
import org.ejml.equation.VariableDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/** An example command that uses an example subsystem. */
public class MoveAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;
 
 public int m_targetdistance;
 public double m_targetrange;
 public int m_lefttarget;
           public double angle;
           public int m_TargetAngle;
// angle90/90
 //targetvalue*angle1;
/**
   * Creates a new MoveForward.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveAngle(RomiDrivetrain subsystem, int Targetvalue, int Lefttarget,  Double targetrange, int TargetAngle ) {
    m_subsystem = subsystem;
    m_targetdistance = Targetvalue;
    m_lefttarget = Lefttarget;
    m_targetrange = targetrange;
    m_TargetAngle = TargetAngle;
   angle =  m_subsystem.getRightDistanceInch()- m_subsystem.getLeftDistanceInch(); 
 

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
  
SmartDashboard.putNumber("angle", angle);
    angle =  m_subsystem.getRightDistanceInch()- m_subsystem.getLeftDistanceInch(); 
  
    var leftdrive  = (m_TargetAngle- Math.abs( angle)) /6  ;//+ Math.signum((m_targetdistance-m_subsystem.getLeftDistanceInch()))* 0.06; 
    var rightdrive =(m_TargetAngle-angle)/6 ;//+ Math.signum((m_lefttarget-m_subsystem.getRightDistanceInch()))* 0.06;
  
  

    m_subsystem.drive(MathUtil.clamp(leftdrive, -1, 1),   0);//MathUtil.clamp(rightdrive, -1, 1));
          

   

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
   
    angle =  m_subsystem.getRightDistanceInch()- m_subsystem.getLeftDistanceInch() *60; 
    if (Math.abs(angle) > Math.abs(m_TargetAngle)-m_targetrange
    &&  Math.abs(angle) < Math.abs(m_TargetAngle)+m_targetrange
    &&  Math.abs( m_subsystem.getLeftSpeed() ) <=0.01)
    {return true;}
  
      return false;
  }
}
