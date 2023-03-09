// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.SetArmCommand;
import frc.robot.commands.TeleOpIntakeCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DisableArmFFCommand;
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

  public static TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(driveTrainSubsystem);
  public static TeleOpIntakeCommand rollIntakeCommand = new TeleOpIntakeCommand(intakeSubsystem);
  // public static MoveArmCommand moveArmCommand = new MoveArmCommand(armSubsystem);
  public static AutonomousCommand autonomousCommand = new AutonomousCommand();

  public static XboxController driver = new XboxController(PortMap.JOYSTICK.DRIVER_JOYSTICK);
  public static XboxController operator = new XboxController(PortMap.JOYSTICK.OPERATOR_JOYSTICK);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveTrainSubsystem.resetEncoders();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driveTrainSubsystem.setDefaultCommand(new TeleOpDriveCommand(driveTrainSubsystem));
    intakeSubsystem.setDefaultCommand(new TeleOpIntakeCommand(intakeSubsystem));
    // armSubsystem.setDefaultCommand(new MoveArmCommand(armSubsystem));


    // SET POSITION IN RADIANS FROM HORIZONTAL
    // 2pi = 360 degrees of rotation


    // if(driver.getAButton()) {new SetArmCommand(armSubsystem, -0.2);}

    // new JoystickButton(driver, Button.kB.value)
    //   .onTrue(new SetArmCommand(armSubsystem, 0));
      
      new JoystickButton(driver, Button.kA.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(1);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(driver, Button.kX.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.disable();;
          }
        )
      );

      // new JoystickButton(driver, Button.kX.value)
      // .onTrue(armSubsystem.disable());

    // if(driver.getAButtonPressed()) {
    //   armSubsystem.setGoal(0.2);
    //   armSubsystem.enable();
    // }

      
      // if(driver.getXButton()) {new DisableArmFFCommand(armSubsystem);}
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void disablePIDSubsystems() {
    armSubsystem.disable();
  }

}
