// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  ArmSubsystem m_armsubsystem = new ArmSubsystem();
  ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  EndEffectorSubsystem m_endEffectorSubsystem = new EndEffectorSubsystem();
  SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  ArmCommand m_armCommand = new ArmCommand(m_armsubsystem, 0, 0, 0);
  ClimbCommand m_climbCommand = new ClimbCommand(m_climbSubsystem);
  ManualDriveCommand m_manualDriveCommand = new ManualDriveCommand();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
