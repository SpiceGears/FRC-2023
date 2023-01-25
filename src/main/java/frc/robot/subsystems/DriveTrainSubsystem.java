// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

  public PowerDistribution pdp;

  public Spark leftMaster;
  public Spark rightMaster;
  public Spark leftSlave1;
  public Spark rightSlave1;

  public MotorControllerGroup leftDrive;
  public MotorControllerGroup rightDrive;

  public DifferentialDrive differentialDrive;

  public Encoder leftEncoder;
  public Encoder rightEncoder;

  private PIDController leftPIDController;
  private PIDController rightPIDController;

  private AHRS gyro;

  private DifferentialDriveOdometry odometry;

  private Pose2d pose;


  // SIMULATION
  private AnalogGyro m_gyro = new AnalogGyro(1);
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private AnalogGyroSim gyroSim;
  private Field2d m_field = new Field2d();

  public DriveTrainSubsystem() {

    pdp = new PowerDistribution();

    leftMaster = new Spark(PortMap.DRIVE.DRIVE_LEFT_MASTER_PORT);
    rightMaster = new Spark(PortMap.DRIVE.DRIVE_RIGHT_MASTER_PORT);
    leftSlave1 = new Spark(PortMap.DRIVE.DRIVE_LEFT_SLAVE1_PORT);
    rightSlave1 = new Spark(PortMap.DRIVE.DRIVE_RIGHT_SLAVE1_PORT);

    leftDrive = new MotorControllerGroup(leftMaster, leftSlave1);
    rightDrive = new MotorControllerGroup(rightMaster, rightSlave1);

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
    // differentialDrive.setSafetyEnabled(false);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftSlave1.setInverted(false);
    rightSlave1.setInverted(true);

    leftEncoder = new Encoder(PortMap.DRIVE.LEFT_ENCODER_PORT_A, PortMap.DRIVE.LEFT_ENCODER_PORT_B);
    leftEncoder.setDistancePerPulse(
            Constants.DRIVETRAIN.DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    leftEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
    leftEncoder.setReverseDirection(Constants.DRIVETRAIN.ENCODER_LEFT_REVERSE);
    leftEncoder.setSamplesToAverage(Constants.DRIVETRAIN.ENCODER_SAMPLES_TO_AVERAGE);

    rightEncoder = new Encoder(PortMap.DRIVE.RIGHT_ENCODER_PORT_A, PortMap.DRIVE.RIGHT_ENCODER_PORT_B);
    rightEncoder.setDistancePerPulse(
            Constants.DRIVETRAIN.DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    rightEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
    rightEncoder.setReverseDirection(Constants.DRIVETRAIN.ENCODER_RIGHT_REVERSE);
    rightEncoder.setSamplesToAverage(Constants.DRIVETRAIN.ENCODER_SAMPLES_TO_AVERAGE);

    resetEncoders();

    leftPIDController = new PIDController(Constants.DRIVETRAIN.PID_LEFT_KP, 
                                          Constants.DRIVETRAIN.PID_LEFT_KI,
                                          Constants.DRIVETRAIN.PID_LEFT_KD);
    rightPIDController = new PIDController(Constants.DRIVETRAIN.PID_RIGHT_KP, 
                                          Constants.DRIVETRAIN.PID_RIGHT_KI,
                                          Constants.DRIVETRAIN.PID_RIGHT_KD);
    
    gyro = new AHRS();

    pose = new Pose2d(5.0, 13.5, new Rotation2d());

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), new Pose2d(5.0, 13.5, new Rotation2d())); // must be in meters


    // SIMULATION
    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    gyroSim = new AnalogGyroSim(m_gyro);
    SmartDashboard.putData("Field", m_field);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logDriveTrain();

      odometry.update(m_gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance());
      m_field.setRobotPose(odometry.getPoseMeters());
  }

  /** Controls the robot based on encoders reading and pid. 
   * Removes unintentional deadband.
   * */
  public void pidDrive(double speed, double turn) {

    if (Math.abs(speed) >= Constants.JOYSTICK.DEADBAND || Math.abs(turn) >= Constants.JOYSTICK.DEADBAND) {

      double leftSpeed = 0;
      double rightSpeed = 0;

          leftSpeed = speed + turn;
          rightSpeed = speed - turn;
          if(rightSpeed < -1) {rightSpeed = -1;}
          if(leftSpeed > 1) {leftSpeed = 1;}
          if(rightSpeed > 1) {rightSpeed = 1;}
          if(leftSpeed < -1) {leftSpeed = -1;}

      //                                      read speed (m/s)              setpoint (1 * x m/s)
      tankDrive(leftPIDController.calculate(-getLeftMetersPerSecond(), leftSpeed * 5),  // blad = getencoder * setpoint  |||||||         blad*kP
                rightPIDController.calculate(-getRightMetersPerSecond(), rightSpeed * 5));

    } else {
      stopDriving();
      leftPIDController.setSetpoint(0);
      rightPIDController.setSetpoint(0);
    }

  }

  /** Controls the robot with simple speed and rotation values.
   * Removes unintentional deadband.
   * */  
  public void arcadeDrive(double xSpeed, double zRotation) {

    if(Math.abs(xSpeed) >= Constants.JOYSTICK.DEADBAND || Math.abs(zRotation) >= Constants.JOYSTICK.DEADBAND) {

      differentialDrive.arcadeDrive(Constants.DRIVETRAIN.SPEED_MULTIPLIER * xSpeed, Constants.DRIVETRAIN.ROTATION_MULTIPLIER * zRotation);

    } else {

      differentialDrive.arcadeDrive(0, 0);

    }

  }

  /** Sends raw speeds directly to motors.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {

    // Bounds speeds in range -1 to 1
    leftSpeed = Math.min(1, Math.max(-1, leftSpeed));
    rightSpeed = Math.min(1, Math.max(-1, rightSpeed));

    differentialDrive.tankDrive(leftSpeed, rightSpeed);

  }

  /** Returns left distance in xxxxxxxxxx */
  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }

  /** Returns right distance in xxxxxxxxxx */
  public double getRightDistance(){
    return rightEncoder.getDistance();
  }

  /** Returns left encoder speed in xxxxxxxxxx */
  public double getLeftMetersPerSecond() {
    return leftEncoder.getRate();
  }

  /** Returns right encoder speed in xxxxxxxxxx */
  public double getRightMetersPerSecond() {
    return rightEncoder.getRate();
  }

  /** Stops drive motors */
  public void stopDriving() {
    tankDrive(0, 0);
  }

  /** Resets endoders */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /** Logs important values to Smart Dashboard */
  public void logDriveTrain() {

    SmartDashboard.putNumber("LeftSpeed in xxxxx", getLeftMetersPerSecond());
    SmartDashboard.putNumber("RighSpeed in xxxxx", getRightMetersPerSecond());
    SmartDashboard.putNumber("Left Distance in xxxxx", getLeftDistance());
    SmartDashboard.putNumber("Right Distance in xxxxx", getRightDistance());
    SmartDashboard.putNumber("LEFTSTICKAXIS", -RobotContainer.driver.getRawAxis(1));
    SmartDashboard.putNumber("RIGHTSTICKAXIS", RobotContainer.driver.getRawAxis(4));

    SmartDashboard.putNumber("PDP Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("PDP Temperature", pdp.getTemperature());
    SmartDashboard.putNumber("PDP Total Current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("PDP Total Power", pdp.getTotalPower());
    SmartDashboard.putNumber("PDP Total Energy", pdp.getTotalEnergy());
  }


  // SIMULATION

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
  DCMotor.getCIM(2),       // 2 NEO motors on each side of the drivetrain.
  8.45,                    // 8.45:1 gearing reduction.
  7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
  60.0,                    // The mass of the robot is 40 kg.
  Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  0.7112,                  // The track width is 0.7112 meters.

  // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  public void simulationPeriodic() {

    
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(leftDrive.get() * RobotController.getInputVoltage(),
    rightDrive.get() * RobotController.getInputVoltage());
    
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);
  
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    m_driveSim.setInputs(leftDrive.get() * RobotController.getInputVoltage(),
                        rightDrive.get() * RobotController.getInputVoltage());
                        
    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

    // Update all of our sensors.
    leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());                    
                        
  }
  
}
