// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  //Motor
  private final CANSparkMax shooterMotor1 = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
  //RelativeEncoder
  private final RelativeEncoder shooterEncoder = shooterMotor1.getEncoder();

  private double intakeGoalSpeed;
  private double shooterGoalSpeed;

  private double shooterMotorVelocity;
  public EndEffectorSubsystem() {
    shooterMotor2.follow(shooterMotor1);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    shooterMotor1.setInverted(false);
    shooterMotor2.setInverted(true);
    intakeMotor.setInverted(false);

    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor1.burnFlash();
    shooterMotor2.burnFlash();
    intakeMotor.burnFlash();
  }

  public void getMotorVelocity(double intakeSetSpeed, double shooterSetSpeed){
    intakeGoalSpeed = intakeSetSpeed;
    shooterGoalSpeed = shooterSetSpeed;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterMotorVelocity = shooterEncoder.getVelocity();
  }
}
