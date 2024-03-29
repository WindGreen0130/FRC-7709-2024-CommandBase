// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double setpoint;
  private double intakeSpeed;
  private double shooterSpeed;
  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem _armSubsystem, double _setpoint, double _intakeSpeed, double _shooterSpeed) {
    this.armSubsystem = _armSubsystem;
    this.setpoint = _setpoint;
    this.intakeSpeed = _intakeSpeed;
    this.shooterSpeed = _shooterSpeed;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
