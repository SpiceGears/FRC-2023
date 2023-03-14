// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.TeleOpIntakeCommand;
import frc.robot.commands.Auto.TestArm;
import frc.robot.commands.Auto.TestDriveOnly;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static ArmSubsystem armSubsystem = new ArmSubsystem();
  public static OtherLogs otherLogs = new OtherLogs();

  public static TeleOpDriveCommand driveCommand = new TeleOpDriveCommand();
  public static TeleOpIntakeCommand rollIntakeCommand = new TeleOpIntakeCommand();
  public static AutonomousCommand autonomousCommand = new AutonomousCommand();

  public static TestArm testArm = new TestArm();
  public static TestDriveOnly testDriveOnly = new TestDriveOnly();

  public static XboxController driver = new XboxController(PortMap.JOYSTICK.DRIVER_JOYSTICK);
  public static XboxController operator = new XboxController(PortMap.JOYSTICK.OPERATOR_JOYSTICK);
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    driveTrainSubsystem.resetEncoders();
    driveTrainSubsystem.tankDrive(0, 0);

    m_chooser.setDefaultOption("do nothing", autonomousCommand);
    m_chooser.addOption("test arm and drive", testArm);
    m_chooser.addOption("test drive only", testDriveOnly);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driveTrainSubsystem.setDefaultCommand(new TeleOpDriveCommand());
    intakeSubsystem.setDefaultCommand(new TeleOpIntakeCommand());


    // SET POSITION IN RADIANS FROM HORIZONTAL
    // 3.14 radians = 180 degrees

      
      new JoystickButton(driver, Button.kA.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(-0.3);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kB.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(0);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kY.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(0.5);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(driver, Button.kX.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.disable();
          }
        )
      );
      new JoystickButton(driver, Button.kLeftBumper.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.resetEncoder();
          }
        )
      );
      new JoystickButton(driver, Button.kRightBumper.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(1.5);
          }
        )
      );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    System.out.println("> getAutonomous() ran");
    return m_chooser.getSelected();

  }

  public void disablePIDSubsystems() {
    armSubsystem.disable();
  }

}
