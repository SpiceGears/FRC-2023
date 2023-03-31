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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto.BetweenCommunityAndLoading2Cube;
import frc.robot.commands.Auto.CenterAutoShootAndDriveToPlatform;
import frc.robot.commands.Auto.RedAllianceFromGrid;
import frc.robot.commands.Auto.Test.TestArm;
import frc.robot.commands.Auto.Test.TestArmAndDrive;
import frc.robot.commands.Auto.Test.TestArmAndDriveByGyro;
import frc.robot.commands.Auto.Test.TestDriveByGyro;
import frc.robot.commands.Auto.Test.TestDriveOnly;
import frc.robot.commands.Auto.Test.TestNewCommands;
import frc.robot.commands.Auto.Test.TestStartFromTop;
import frc.robot.commands.Drive.DriveForwardByGyro;
import frc.robot.commands.Drive.DriveWithGyro;
import frc.robot.commands.Drive.TeleOpDrive;
import frc.robot.commands.Intake.ShootCube;
import frc.robot.commands.Intake.TeleOpIntake;
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

  public static TeleOpDrive driveCommand = new TeleOpDrive();
  public static TeleOpIntake rollIntakeCommand = new TeleOpIntake();

  // add autopaths here
  public static TestArm testArm = new TestArm();
  public static TestDriveOnly testDriveOnly = new TestDriveOnly();
  public static TestArmAndDrive testArmAndDrive = new TestArmAndDrive();
  public static TestDriveByGyro testDriveByGyro = new TestDriveByGyro();
  public static TestArmAndDriveByGyro testArmAndDriveByGyro = new TestArmAndDriveByGyro();
  public static TestStartFromTop testStartFromTop = new TestStartFromTop();
  public static TestNewCommands testNewCommands = new TestNewCommands();

  public static CenterAutoShootAndDriveToPlatform centerAutoShootAndDriveToPlatform = new CenterAutoShootAndDriveToPlatform();
  public static BetweenCommunityAndLoading2Cube betweenCommunityAndLoading2Cube = new BetweenCommunityAndLoading2Cube();

  public static XboxController driver = new XboxController(PortMap.JOYSTICK.DRIVER_JOYSTICK);
  public static XboxController operator = new XboxController(PortMap.JOYSTICK.OPERATOR_JOYSTICK);
  
  
  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    configureButtonBindings();
    driveTrainSubsystem.resetEncoders();
    driveTrainSubsystem.tankDrive(0, 0);
    
    configureAutoChooser(); // adds chooser to dashboard and adds auto options
  }
  
  /** Add chooser to dashboard and add auto options. */
  public void configureAutoChooser() {

    // TEST
    // m_chooser.setDefaultOption("do nothing", new WaitCommand(1));
    // m_chooser.addOption("test arm then drive", testDriveOnly);
    // m_chooser.addOption("test arm only", testArm);
    // m_chooser.addOption("test drive only", testDriveOnly);
    // m_chooser.addOption("TEST DRIVE BY GYRO", testDriveByGyro);
    // m_chooser.addOption("TEST ARM AND DRIVE BY GYRO", testArmAndDriveByGyro);
    // m_chooser.addOption("TeSt ArM FrOm ToP PoSiTiOn To LoW AnD InTaKe AuTo", testStartFromTop);

    // COMPETITION
    m_chooser.addOption("centerAutoShootAndDriveToPlatform", centerAutoShootAndDriveToPlatform);
    m_chooser.addOption("betweenCommunityAndLoading2Cube", betweenCommunityAndLoading2Cube);
    

    // m_chooser.addOption("RESETARM -> SETARM(0) -> INTAKETEST -> GO BACKWARD, ROTATETEST, GO FORWARD", testNewCommands);

    SmartDashboard.putData(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    driveTrainSubsystem.setDefaultCommand(new TeleOpDrive());
    intakeSubsystem.setDefaultCommand(new TeleOpIntake());


    // SET POSITION IN RADIANS FROM HORIZONTAL
    // 3.14 radians = 180 degrees

      // DRIVER
      new JoystickButton(driver, Button.kA.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.INTAKE);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kB.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.HORIZONTAL);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kY.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.SECONDLEVEL);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(driver, Button.kX.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(1.4);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(driver, Button.kRightBumper.value)
      .onTrue(new ShootCube());


      //OPERATOR
      new JoystickButton(operator, Button.kA.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.INTAKE);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(operator, Button.kB.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.HORIZONTAL);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(operator, Button.kY.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.SECONDLEVEL);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(operator, Button.kX.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.VERTICAL);
            armSubsystem.enable();
          }
        )
      );

      new JoystickButton(operator, Button.kRightBumper.value)
      .onTrue(new ShootCube());
      


      // new JoystickButton(driver, Button.kLeftBumper.value)
      // .onTrue(
      //   Commands.runOnce(
      //     () -> {
      //       armSubsystem.resetEncoder(robot.commands.Intake.ShootCube);
      //     }
      //   )
      // );
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
