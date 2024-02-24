// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FloopIntakeConstants;
import frc.utils.SwerveUtils;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloopIntakeConstants;
import frc.robot.Constants.CANIds;

public class FloopIntake extends SubsystemBase implements Loggable{
  private final CANSparkMax intake = new CANSparkMax(CANIds.kIntakeCanID, MotorType.kBrushless);
  private final CANSparkMax intakeTurnPitch = new CANSparkMax(CANIds.kIntakeTurnPitchCanID, MotorType.kBrushless);

   private final DoubleSolenoid intakeTurnYaw =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, FloopIntakeConstants.kIntakePistonOut, FloopIntakeConstants.kIntakePistonIn);

  public final ProfiledPIDController intakePIDController = new ProfiledPIDController(
    FloopIntakeConstants.intakeP, 
    FloopIntakeConstants.intakeI, 
    FloopIntakeConstants.intakeD, 
    new Constraints(
      FloopIntakeConstants.velocityConstraint, 
      FloopIntakeConstants.accelerationConstraint
      )
    );
  public FloopIntake(){
    intake.restoreFactoryDefaults();
    intakeTurnPitch.restoreFactoryDefaults();

    intake.setIdleMode(IdleMode.kCoast);
    intakeTurnPitch.setIdleMode(IdleMode.kBrake);

    intake.burnFlash();
    intakeTurnPitch.burnFlash();
  }
  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/
  @Log
  public double getSetpoint() {
    return intakePIDController.getGoal().position;
  }

  


  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/
  @Config
  public void intakeYawOut(){
    intakeTurnYaw.set(Value.kForward);
  }

  @Config
  public void intakeYawIn(){
    intakeTurnYaw.set(Value.kReverse);
  }

  @Config
  public void setPitchSetpoint(double setpoint) {
    intakePIDController.setGoal(setpoint);
  }

  @Config
  public void setIntakeSpeed(double speed){
    intake.set(speed);
  }


  
}



