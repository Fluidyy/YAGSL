// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.teleop;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.intakesub;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.intakesub;
import frc.robot.subsystems.IntakeSystem;


import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// import frc.robot.subsystems.outake;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    


  // The robot's subsystems and c ommands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final Limelight limelight = new Limelight();

  private final IntakeSystem intakeSystem = new IntakeSystem();


  // private final outake outake = new outake();
  // private final intakesub intake = new intakesub();


                                                                  
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  Joystick operator = new Joystick(1);

  private final SendableChooser<Command> autoChooser;




  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  private final JoystickButton driver_limelightButton = new JoystickButton(driverXbox, XboxController.Button.kB.value);

  private final JoystickButton pidButton = new JoystickButton(driverXbox, XboxController.Button.kX.value);

  private final JoystickButton runafucktonofshit = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kY.value);




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()

  {
    // NamedCommands.registerCommand("shoot", new kracken().speed(1));
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
   

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox::getYButtonPressed,
                                                                   driverXbox::getAButtonPressed,
                                                                   driverXbox::getXButtonPressed,
                                                                   driverXbox::getBButtonPressed);

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
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    teleop closedFieldRelOperator = new teleop(
      drivebase,
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),

      () -> driverXbox.getRightX(), () -> true);

    // drivebase.setDefaultCommand(!RobotBase.isSimulation() ?  driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngle);
    
    drivebase.setDefaultCommand(closedFieldRelOperator);



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



    new JoystickButton(driverXbox, 1).onTrue((new InstantCommand(drivebase::zeroGyro)));

    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox,
    //                    4).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new oInstantCommand(drivebase::lock, drivebase)));

// driver_limelightButton.whileTrue(new teleoplimelight(
//       limelight,
//       drivebase,
//       () -> driverXbox.getLeftY(),
//       () -> driverXbox.getLeftX(), 

//       () -> driverXbox.getRightX(), () -> false));

pidButton.whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(1.844,4.98),Rotation2d.fromDegrees(180))));





// runafucktonofshit.whileTrue(

// new SequentialCommandGroup(new intakesub().intakeCommand(0.5).withTimeout(0.5), new kracken().pid(12))
  




// );
intakeButton.whileTrue(
  new SequentialCommandGroup(new IntakeSystem().Intakepid(10),
  new IntakeSystem().intakeMotorsCommand().withTimeout(1),
   new IntakeSystem().Intakepid(5).withTimeout(1),
    new IntakeSystem().Intakepid(0)));

intakeButton.whileTrue(
  new IntakeSystem().feederCommand()
);



// runafucktonofshit.whileFalse(new kracken().pid(0));
		
    SmartDashboard.putData("Auto chooser",autoChooser);



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("3PiecePath1");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }
  public void camera(){
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}