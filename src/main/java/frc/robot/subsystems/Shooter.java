// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;

/**
 * The shooter subsystem for 3322's 2024 robot.
 */
public class Shooter extends SubsystemBase implements Loggable {
  
  private final CANSparkMax shooterTopMotor = new CANSparkMax(CANIds.kShooterTopCanId, MotorType.kBrushless);
  private final CANSparkMax shooterBottomMotor = new CANSparkMax(CANIds.kShooterBottomCanId, MotorType.kBrushless);

  private final RelativeEncoder shooterTopEncoder = shooterTopMotor.getEncoder();
  private final RelativeEncoder shooterBottomEncoder = shooterBottomMotor.getEncoder();

  private final PIDController shooterRPMController = new PIDController(
    ShooterConstants.shooterP,
    ShooterConstants.shooterI, 
    ShooterConstants.shooterD
  );

  private final SimpleMotorFeedforward shooterRPMFeedforward = new SimpleMotorFeedforward(0, ShooterConstants.shooterV);

  private double shooterTopSetpoint = 0;
  private double shooterBottomSetpoint = 0;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooterTopMotor.restoreFactoryDefaults();
    shooterBottomMotor.restoreFactoryDefaults();

    shooterTopMotor.setIdleMode(IdleMode.kCoast);
    shooterBottomMotor.setIdleMode(IdleMode.kCoast);

    shooterTopMotor.setInverted(false);
    shooterBottomMotor.setInverted(true);

    shooterTopMotor.burnFlash();
    shooterBottomMotor.burnFlash();
  }

  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  /**
   * Returns the calculations of the PID controller given the top shooter's current RPM, and RPM setpoint.
   * @return The PID controller's output.
   */
  public double getTopMotorPIDOutput() {
    return shooterRPMController.calculate(shooterTopEncoder.getVelocity(), shooterTopSetpoint);
  }
  
  /**
   * Returns the calculations of the PID controller given the bottom shooter's current RPM, and RPM setpoint.
   * @return The PID controller's output.
   */
  public double getBottomMotorPIDOutput() {
    return shooterRPMController.calculate(shooterBottomEncoder.getVelocity(), shooterBottomSetpoint);
  }

  /**
   * Returns the calculations of the feedforward controller given the top shooter's RPM setpoint.
   * @return The feedforward controller's output.
   */
  public double getTopMotorFeedforwardOutput() {
    return shooterRPMFeedforward.calculate(shooterTopSetpoint);
  }

  /**
   * Returns the calculations of the feedforward controller given the bottom shooter's RPM setpoint.
   * @return The feedforward controller's output.
   */
  public double getBottomMotorFeedforwardOutput() {
    return shooterRPMFeedforward.calculate(shooterBottomSetpoint);
  }

  /**
   * Returns the calculations of the feedforward controller and the PID controller for the top motor.
   * @return The combined feedfoward and PID controller's output.
   */
  public double getTopCombinedControllers() {
    return getTopMotorFeedforwardOutput() + getTopMotorFeedforwardOutput();
  }

  /**
   * Returns the calculations of the feedforward controller and the PID controller for the bottom motor.
   * @return The combined feedfoward and PID controller's output.
   */
  public double getBottomCombinedControllers() {
    return getBottomMotorFeedforwardOutput() + getBottomMotorFeedforwardOutput();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Set the top motors speed.
   * @param speed The speed of the motor
   */
  public void setTopShooterSpeed(double speed) {
    shooterTopMotor.set(speed);
  }

  /**
   * Set the bottom motors speed.
   * @param speed The speed of the motor.
   */
  public void setBottomShooterSpeed(double speed) {
    shooterTopMotor.set(speed);
  }

  /**
   * Stop the top shooter motor.
   */
  public void stopTopShooter() {
    shooterTopMotor.stopMotor();
  }

  /**
   * Stop the bottom shooter motor
   */
  public void stopBottomShooter() {
    shooterBottomMotor.stopMotor();
  }

  /**
   * Set the setpoint of the top wheel.
   * @param setpoint The setpoint in RPM.
   */
  public void setTopShooterSetpoint(double setpoint) {
    shooterTopSetpoint = setpoint;
  }

  /**
   * Set the setpoint of the bottom wheel.
   * @param setpoint The setpoint in RPM.
   */
  public void setBottomShooterSetpoint(double setpoint) {
    shooterBottomSetpoint = setpoint;
  }
  
  /**
   * Set the speed of both the top and bottom motors.
   * @param speed The speed of the motors.
   */
  public void setShooterSpeeds(double speed) {
    setTopShooterSpeed(speed);
    setBottomShooterSpeed(speed);
  }

  /**
   * Stop both shooter motors.
   */
  public void stopShooter() {
    stopTopShooter();
    stopBottomShooter();
  }

  /**
   * Set the setpoint of both the top and the bottom wheels.
   * @param setpoint The setpoint in RPM.
   */
  public void setShooterSetpoint(double setpoint){
    setTopShooterSetpoint(setpoint);
    setBottomShooterSetpoint(setpoint);
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  /**
   * A run command that drives the shooter wheel RPMs to the given speed.
   * @param RPM the desired wheel speeds in RPM.
   * @return A run command.
   */
  public Command shooterRevUpCommand(double RPM) {
    return new SequentialCommandGroup(
      new InstantCommand(
        () -> setShooterSetpoint(RPM),
        this
      ),
      new RunCommand(
        () -> {
          setTopShooterSpeed(getTopCombinedControllers());
          setBottomShooterSpeed(getBottomCombinedControllers());
        },
        this
      )
    );
  }

  /**
   * A run command that drives the shooter wheels to the auto line shooting rpm.
   * @return A run command.
   */
  public Command shooterAutoLineRevUpCommand(){
    return shooterRevUpCommand(ShooterConstants.shootingRMPAutoLine);
  }

  /**
   * An instant command that sets the shooter setpoint to zero and stops both shooter wheels.
   * @return An instant command
   */
  public Command stopShooterCommand() {
    return new InstantCommand(
      () ->{
        setShooterSetpoint(0);
        stopShooter();
      },
      this
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}