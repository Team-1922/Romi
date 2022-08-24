// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.KeyDrive;
import frc.robot.commands.MoveAngle;
import frc.robot.commands.MoveForward;
import frc.robot.commands.MoveForwardDistance;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final Joystick m_keyboard = new Joystick(1);
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final KeyDrive m_KeyDrive = new KeyDrive(m_romiDrivetrain, m_keyboard);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);
  private final MoveForward m_moveForward = new MoveForward(m_romiDrivetrain);
  private final MoveForwardDistance m_MoveForwardDistance = new MoveForwardDistance(m_romiDrivetrain, 9 , 0.1 );
  public final ParallelDeadlineGroup m_attemp = new ParallelDeadlineGroup(new WaitCommand(1), m_moveForward);
  public final MoveAngle m_MoveAngle = new MoveAngle(m_romiDrivetrain, -3, 1, 2.0, 90);
  //public final ParallelDeadlineGroup m_MoveDistanceTime = new ParallelDeadlineGroup(new WaitCommand(9), m_MoveForwardDistance);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    //  final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain.();
    m_romiDrivetrain.setDefaultCommand(m_KeyDrive);  
    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
  //keydrive().whenpressed();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_MoveAngle;
  }
}
