// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.FloopIntakeConstants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final Shooter shooter = new Shooter();
  

  // The driver's controller
  CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // Secondary controller
  CommandXboxController secondaryController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

  SendableChooser<SequentialCommandGroup> autoSelector = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure Oblog logger
    Logger.configureLoggingAndConfig(this, true);

    // Auton selector config
    // autoSelector.setDefaultOption("Test", new Test4And1Auto());

    autoSelector.addOption("No auto", null);

    // Configure default commands

    /*
     * ◇─◇──◇─◇
     * Drivetrain
     * ◇─◇──◇─◇
     */

    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX() / 1.5, OIConstants.kDriveDeadband),
                true, true),
            robotDrive));

    /*
     * ◇─◇──◇─◇
     * Elevator
     * ◇─◇──◇─◇
     */

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /*◇─◇──◇─◇
     Drivetrain
    ◇─◇──◇─◇*/

    driverController.x()
        .whileTrue(new RunCommand(
            () -> robotDrive.setX(),
            robotDrive));

    driverController.start().onTrue(new InstantCommand(()->robotDrive.zeroHeading()));


  /*◇─◇──◇─◇
  ✨Shooter✨
  ◇─◇──◇─◇*/
  
  driverController.y()
        .whileTrue(new RunCommand(
            () -> shooter.setShooterSpeed(ShooterConstants.SHOOT),
            shooter));

  driverController.a().whileTrue(new RunCommand(
            () -> shooter.setShooterSpeed(0),
            shooter));

  driverController.b().whileTrue(new RunCommand(
            () -> shooter.pointShooter(0.25),
            shooter));
            
             driverController.b().whileFalse(new RunCommand(
            () -> shooter.stopPointShooter(0),
            shooter));
  
  /*◇─◇──◇─◇
  ✨FloopIntake✨
  ◇─◇──◇─◇*/




   
            
  }
}