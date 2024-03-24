// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import io.github.oblarg.oblog.Loggable;

public class Shooter extends SubsystemBase implements Loggable{
  private final CANSparkFlex leftShooterMotor = new CANSparkFlex(CANIds.kShooterLeftID, MotorType.kBrushless);
  private final CANSparkFlex rightShooterMotor = new CANSparkFlex(CANIds.kShooterRightID, MotorType.kBrushless);
  public Shooter(){
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();

    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);

    rightShooterMotor.setInverted(true);
    rightShooterMotor.follow(leftShooterMotor);

    leftShooterMotor.burnFlash();
    rightShooterMotor.burnFlash();
  }
  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/
  
  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

   /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

}

