// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.shooter.Hinge;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Rollers;
import frc.robot.subsystems.sequences.Amp;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController shooterXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));
  private final Hinge hinge = new Hinge();
  private final Intake intake = new Intake();
  private final Rollers rollers = new Rollers();
  private final Amp amp = new Amp();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                 OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(

        // Apply Deadband and trigger-based speed control :3
        () -> MathUtil.applyDeadband((driverXbox.getLeftY() * (0.30 + (0.70 * (1 - driverXbox.getRightTriggerAxis())))), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband((driverXbox.getLeftX()* (0.30 + (0.70 * (1 - driverXbox.getRightTriggerAxis())))), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 1);


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.povUp().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //    Commands.deferredProxy(() -> drivebase.driveToPose(
    //                               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                          ));
    // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));


    // Hinge Setpoints
    shooterXbox.b().whileTrue(amp.lamp());


    shooterXbox.a().whileTrue(hinge.Setpoints(72));
    shooterXbox.x().whileTrue(hinge.Setpoints(1));

    // Flywheel Speeds
    shooterXbox.leftBumper().whileTrue(intake.flywheel(85.0).withTimeout(2));
    shooterXbox.rightBumper().whileTrue(intake.flywheel(-10.0));

    
    //none of this works
    shooterXbox.leftBumper().and(shooterXbox.rightBumper()).whileTrue(intake.flywheel(0.0));

    
    // Roller Speeds
    shooterXbox.leftBumper().whileTrue(rollers.roller(-40.0).withTimeout(2));
    shooterXbox.rightBumper().whileTrue(rollers.roller(10.0));


    // yeah this doesn't either
    shooterXbox.leftBumper().and(shooterXbox.rightBumper()).whileTrue(rollers.roller(0.0));

    

  
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Test AUto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
