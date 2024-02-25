// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FloopIntakeConstants.ShooterConstants;
import frc.utils.SwerveUtils;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase implements Loggable{
  private final CANSparkMax shooterAngleMotor = new CANSparkMax(CANIds.kShooterAngleMotorCanId, MotorType.kBrushless);
  private final CANSparkFlex shooterLeft = new CANSparkFlex(CANIds.kShooterLeftMotorCanId, MotorType.kBrushless);
  private final CANSparkFlex shooterRight = new CANSparkFlex(CANIds.kShooterRightMotorCanId, MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = shooterAngleMotor.getEncoder();
  private final RelativeEncoder shooterLeftEncoder = shooterAngleMotor.getEncoder();
  private final RelativeEncoder shooterRightEncoder = shooterAngleMotor.getEncoder();
  
  private final PIDController shooterAngleController = new PIDController(
    ShooterConstants.shooterAngleP,
    ShooterConstants.shooterAngleI, 
    ShooterConstants.shooterAngleD
  );
  
  public Shooter(){
    shooterAngleMotor.restoreFactoryDefaults();
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    

    shooterAngleMotor.setIdleMode(IdleMode.kCoast);
    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);

    shooterLeft.setInverted(true);

    shooterAngleMotor.burnFlash();
    shooterLeft.burnFlash();
    shooterRight.burnFlash();

  }

  
  /*◇─◇──◇─◇
  ✨Getters✨
  ◇─◇──◇─◇*/
  @Log
  public double shooterRPM(){
    return shooterEncoder.getVelocity();
  }

  /*◇─◇──◇─◇
  ✨Setters✨
  ◇─◇──◇─◇*/

  @Config
  public void setShooterSpeed(double speed){
    shooterLeft.set(speed);
    shooterRight.set(speed);
  }
  @Config
  public void moveShooterAngle(double speed){
    shooterAngleMotor.set(speed);
  }
  
   /*◇─◇──◇─◇
  ✨Commands✨
  ◇─◇──◇─◇*/

}

