// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.RevJNIWrapper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ShooterCommands;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable{
  private final CANSparkFlex leftShooterMotor = new CANSparkFlex(CANIds.kShooterLeftID, MotorType.kBrushless);
  private final CANSparkFlex rightShooterMotor = new CANSparkFlex(CANIds.kShooterRightID, MotorType.kBrushless);

  private final RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

  private final PIDController shooterPID = new PIDController(
    ShooterConstants.shooterP,
    ShooterConstants.shooterI,
    ShooterConstants.shooterD
  );

  //private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, ShooterConstants.shooterV);
  @Log
  private double setpoint = 0;
  
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
  @Log
  public double getLeftRPM(){
    return leftShooterEncoder.getVelocity();
  }

  @Log
  public double getRightRPM(){
    return rightShooterEncoder.getVelocity();
  }
  /* 
  public double feedForwardOut(double speed){
    return feedForward.calculate(setpoint);
  }
  */

  @Log
  public double PIDOut(double speed){
    return shooterPID.calculate(speed, setpoint);
  }

  

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/
  @Config
  public void setShooterSpeed(double speed){
    leftShooterMotor.set(speed);
  }

  @Config
  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
  }



   /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

}

