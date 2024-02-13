// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase implements Loggable {
  /** Creates a new GroundIntake. */

  // Creates motor objects
  private final CANSparkMax wheelsMotor = new CANSparkMax(Constants.CANIds.kIntakeWheelsCanId, MotorType.kBrushless);
  private final CANSparkMax intakeArmLeft = new CANSparkMax(Constants.CANIds.kIntakePivotLeftCanId, MotorType.kBrushless);
  private final CANSparkMax intakeArmRight = new CANSparkMax(Constants.CANIds.kIntakePivotRightCanId, MotorType.kBrushless);


  // Creates encoder for intakeArm
  private final RelativeEncoder armLeftEncoder = intakeArmLeft.getEncoder();
  private final RelativeEncoder armRightEncoder = intakeArmRight.getEncoder();

  // Creates the intake beam sensor objects
  private final DigitalInput intakeOuterBeamBreak = new DigitalInput(0);
  private final DigitalInput intakeInnerBeamBreak = new DigitalInput(1);

  // Creates a new Profiled PID Controller
  public final ProfiledPIDController intakePIDController = new ProfiledPIDController(
    IntakeConstants.intakeP, 
    IntakeConstants.intakeI, 
    IntakeConstants.intakeD, 
    new Constraints(
      IntakeConstants.velocityConstraint, 
      IntakeConstants.accelerationConstraint
      )
    );

  // GroundIntake Constructor
  public Intake() {
    intakeArmLeft.restoreFactoryDefaults();
    intakeArmRight.restoreFactoryDefaults();

    wheelsMotor.setIdleMode(IdleMode.kBrake);
    intakeArmLeft.setIdleMode(IdleMode.kBrake);
    intakeArmRight.setIdleMode(IdleMode.kBrake);

    armLeftEncoder.setPositionConversionFactor(IntakeConstants.kIntakeArmGearRatio);
    armLeftEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeArmGearRatio);

    
    armRightEncoder.setPositionConversionFactor(IntakeConstants.kIntakeArmGearRatio);
    armRightEncoder.setVelocityConversionFactor(IntakeConstants.kIntakeArmGearRatio);

    intakeArmLeft.setInverted(true);
    intakeArmRight.follow(intakeArmLeft, true);

    wheelsMotor.burnFlash();
    intakeArmLeft.burnFlash();
    intakeArmRight.burnFlash();

    SmartDashboard.putData("intake", intakePIDController);
  }


  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/

  /**
   * Returns the current position goal of the profiled PID.
   * @return The position goal of the elevator PID.
   */
  @Log
  public double getSetpoint() {
    return intakePIDController.getGoal().position;
  }

  /**
   * Returns the encoder position of the left arm encoder.
   * @return The position of the intake arm encoder.
   */
  @Log
  public double getIntakeEncoderPosition() {
    return armLeftEncoder.getPosition();
  }

  @Log
  public Boolean intakeEmpty(){
    return !intakeInnerBeamBreak.get() && !intakeOuterBeamBreak.get();
  }

  /**
   * Returns true if both of the beams detect a ring, returning false otherwise
   * @return Whether or not the intake has (true) or doesn't have (false) a ring
   */
  @Log
  public Boolean intakeFull() {
    return intakeInnerBeamBreak.get() && intakeOuterBeamBreak.get();
  }

  @Log
  public Boolean outerIntakeFull() {
    return intakeOuterBeamBreak.get();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  /**
   * Set the setpoint of the intake PID controller. 
   * Unit is Rotations?
   * @param setpoint The setpoint of the PID controller.
   */
  public void setIntakeSetpoint(double setpoint) {
    intakePIDController.setGoal(setpoint);
  }

  /**
   * Set the power of both intake arm motors, with the right one inverted.
   * @param power The power for both motors.
   */
  public void setArmSpeed(double speed){
    intakeArmLeft.set(speed);
    intakeArmRight.set(speed);
  }

  public void spinRollers(double power) {
    wheelsMotor.set(power);
  }

  /**
   * Stop both intake motors.
   */
  public void stopArm() {
    intakeArmLeft.stopMotor();
    intakeArmRight.stopMotor();
  }

  /**
   * Sets the position of the left intake arm encoder to 0
   */
  public void resetArmEncoder() {
    armLeftEncoder.setPosition(0);
  }

  /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

  /**
   * Returns a command that uses the intake arm PID command to go to its setpoint until interuppted.
   * @return A run command that moves the intake arm towards its setpoint.
   */
  public Command flipToSetpoint() {
    return new RunCommand(
      () -> {
        setArmSpeed((intakePIDController.calculate(getIntakeEncoderPosition())));
      }, 
      this
    );
  }

  /**
   * A command that sets the PID goal to the flipped up position and moves to it.
   * @return A sequential command group that moves the intake arm to the flipped up position.
   */
  public Command flipToStow(){
    //set flipper to correct setting until it is true that the flipper is at the top.
    return new SequentialCommandGroup (
      new InstantCommand(
        () -> setIntakeSetpoint(IntakeConstants.stowPosition),
        this
      ),
      flipToSetpoint()
    );
  }

  /**
   * A command that sets the PID goal to the ground position and moves to it.
   * @return A sequential command group that moves the intake arm to the ground position.
   */
  public Command flipToGround(){
    //flips to bottom, does not spin. may be able to delete
    return new SequentialCommandGroup (
      new InstantCommand(
        () -> setIntakeSetpoint(IntakeConstants.intakePosition),
        this
      ),
      flipToSetpoint()
    );
  }


  /**
   * A command that sets the PID goal to the AMP position and moves to it.
   * @return A sequential command group that moves the intake arm to the AMP position.
   */
  public Command flipToAmp() {
    return new SequentialCommandGroup (
      new InstantCommand(
        () -> setIntakeSetpoint(IntakeConstants.ampZonePosition),
        this
      ),
      flipToSetpoint()
    );
  }

  /**
   * An instant command that stops the intake arm. Can be used to return to manual control.
   * @return An instant command that stops the intake arm's movement
   */
  public Command stopIntakeArmCommand() {
    return new InstantCommand(
      () -> {
        stopArm();
      },
      this
    );
  }

  /**
   * A run command that keeps spinning the intake rollers until the beam sensors detect a ring
   * @return A run command that spins the intake's rollers until it ring is detected
   */

  public Command intakeUntilBeamBreak() {
    return new StartEndCommand(
      () ->{
        spinRollers(-IntakeConstants.groundIntakeSpeed);
      }, 
      () -> {
        spinRollers(0);
      }, this)
      .until(this::intakeFull);
  }

    /**
   * A command that keeps spinning the intake rollers until the beam sensors detect a ring, then run them again until its empty
   * @return A run command that spins the intake's rollers until it ring is detected
   */
  public Command intakeToMiddleCommand(){
    return new StartEndCommand(
        () -> {
          spinRollers(-IntakeConstants.groundIntakeSpeed);
        },
        () -> {}, this)
        .until(this::intakeFull)
        .andThen(new StartEndCommand(
            () -> {
              spinRollers(-IntakeConstants.groundIntakeSpeed);
            },
            () -> {
              spinRollers(0);
            }, this).until(this::intakeEmpty));
      
  }

  /**
   * An instant command that stops the intake rollers
   * @return An instant command that stops the intake rollers
   */
  public Command stopRollersCommand(){
    return new InstantCommand(
      () -> {
        spinRollers(0);
      }
    );
  }
  /**
   * An instant command that spins the intake rollers at a constant speed
   * @return An instant command that spins the intake's rollers
   */
    public Command intakeCommand(){
    return new InstantCommand(
      () -> {
        spinRollers(IntakeConstants.groundIntakeSpeed);
      }
    );
  }

  /**
   * A StartEndCommand that spins the intake rollers until the intake is empty, where the rollers stop spinning
   * @return A StartEndCommand that spins the intake's rollers until a ring is inside it
   */
  public Command ejectCommand(){
    return new StartEndCommand(
      () ->{
        spinRollers(-IntakeConstants.groundIntakeSpeed);
      }, 
      () -> {
        spinRollers(0);
      }, this)
      .until(this::intakeEmpty);
      
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

  }



}
