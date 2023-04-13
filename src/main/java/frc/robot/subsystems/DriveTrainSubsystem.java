// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PortMap;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */

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

  public AHRS gyro;

  private DifferentialDriveOdometry odometry;

  private Pose2d pose;

  public static PowerDistribution pdp = new PowerDistribution();


  // SIMULATION
  // private AnalogGyro m_gyro = new AnalogGyro(1);
  private EncoderSim leftEncoderSim;
  private EncoderSim rightEncoderSim;
  private AHRS gyro_sim;
  private Field2d m_field = new Field2d();

  public DriveTrainSubsystem() {

    leftMaster = new Spark(PortMap.DRIVE.LEFT_MASTER_PORT);
    rightMaster = new Spark(PortMap.DRIVE.RIGHT_MASTER_PORT);
    leftSlave1 = new Spark(PortMap.DRIVE.LEFT_SLAVE1_PORT);
    rightSlave1 = new Spark(PortMap.DRIVE.RIGHT_SLAVE1_PORT);

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
            Constants.DRIVETRAIN.ENCODER_DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    // leftEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
    leftEncoder.setReverseDirection(Constants.DRIVETRAIN.ENCODER_LEFT_REVERSE);
    leftEncoder.setSamplesToAverage(Constants.DRIVETRAIN.ENCODER_SAMPLES_TO_AVERAGE);

    rightEncoder = new Encoder(PortMap.DRIVE.RIGHT_ENCODER_PORT_A, PortMap.DRIVE.RIGHT_ENCODER_PORT_B);
    rightEncoder.setDistancePerPulse(
            Constants.DRIVETRAIN.ENCODER_DISTANCE_PER_ROTATION / Constants.DRIVETRAIN.ENCODER_TICK_RATE);
    // rightEncoder.setMaxPeriod(Constants.DRIVETRAIN.ENCODER_MIN_RATE);
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
    gyro_sim = gyro;
    SmartDashboard.putData("Field", m_field);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logDriveTrain();
    logPDP();

    odometry.update(gyro.getRotation2d(),
    leftEncoder.getDistance(),
    rightEncoder.getDistance());
    m_field.setRobotPose(odometry.getPoseMeters());
  }

  /** Controls the robot based on encoders reading and pid. 
   * Removes unintentional deadband.
   * */
  public void pidDrive(double speed, double turn) {

    double leftSpeed;
    double rightSpeed;

    // SPEED != 0 and TURN != 0
    if (Math.abs(speed) >= Constants.JOYSTICK.DEADBAND && Math.abs(turn) >= Constants.JOYSTICK.DEADBAND) {

      // reduce turn when going forwards/backwards
      speed *= 1;
      turn *= 0.5;

      leftSpeed = MathUtil.clamp(speed + turn, -1, 1);
      rightSpeed = MathUtil.clamp(speed - turn, -1, 1);

      //                                      read speed (m/s)              setpoint (1 * x m/s)
      tankDrive(leftPIDController.calculate(getLeftMetersPerSecond(), leftSpeed * Constants.DRIVETRAIN.MAX_SPEED),
                rightPIDController.calculate(getRightMetersPerSecond(), rightSpeed * Constants.DRIVETRAIN.MAX_SPEED));

    }
    // SPEED != 0 and TURN = 0
    else if(Math.abs(speed) >= Constants.JOYSTICK.DEADBAND && Math.abs(turn) <= Constants.JOYSTICK.DEADBAND) {

      // drive both wheels the same speed based on one encoder when not turning
      // compensates for drift between wheels when driving forwards

      //                                      read speed (m/s)              setpoint (1 * x m/s)
      tankDrive(leftPIDController.calculate(getLeftMetersPerSecond(), speed * Constants.DRIVETRAIN.MAX_SPEED),
                rightPIDController.calculate(getLeftMetersPerSecond(), speed * Constants.DRIVETRAIN.MAX_SPEED));
    }
    // SPEED = 0 and TURN != 0
    else if(Math.abs(speed) <= Constants.JOYSTICK.DEADBAND && Math.abs(turn) >= Constants.JOYSTICK.DEADBAND) {

      leftSpeed = MathUtil.clamp(speed + turn, -1, 1);
      rightSpeed = MathUtil.clamp(speed - turn, -.95, .95);

      //                                      read speed (m/s)              setpoint (1 * x m/s)
      tankDrive(leftPIDController.calculate(getLeftMetersPerSecond(), leftSpeed * Constants.DRIVETRAIN.MAX_SPEED),
                rightPIDController.calculate(getRightMetersPerSecond(), rightSpeed * Constants.DRIVETRAIN.MAX_SPEED));
    } 
    // SPEED = 0 and TURN = 0
    else {
    
      //set motors to 0 and PID setpoint to 0
      stopDriving();
      leftPIDController.setSetpoint(0);
      rightPIDController.setSetpoint(0);
    }
    

  }

   /** Drive to direction that is gyro.getHeading() == 0 ,
   * to drive straight, execute gyro.reset() before this method. 
   * Can be used to drive backward. */
  public void driveStraightAtZeroHeading(double speed) {

    // WHEN GOING FORWARD:
    if(speed > 0) {

      // if angle < maxerror1 => drive normally minus error
      if(Math.abs(gyro.getAngle()) <= Constants.GYRO.MAX_ERROR_1) {
  
        //                 v this part adjusts motor powers to go straight
        tankDrive(speed - (gyro.getAngle() / 30),
                  speed + (gyro.getAngle() / 30)); 
      }
      // if angle < maxerror2 => slow down motor near center of heading
      else if (Math.abs(gyro.getAngle()) <= Constants.GYRO.MAX_ERROR_2) {
        if(gyro.getAngle() > 0) {
          tankDrive(speed * Constants.GYRO.MOTOR_SLOWDOWN_ON_ERROR2,
                    speed);
        } else if(gyro.getAngle() < 0) {
          tankDrive(speed,
                    speed * Constants.GYRO.MOTOR_SLOWDOWN_ON_ERROR2);
        }
      }
      // if angle > maxerror2 => rotate robot in place
      else {
        if(gyro.getAngle() > 0) {
          differentialDrive.tankDrive(  -speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER,
                                        speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER);
        } else if(gyro.getAngle() < 0) {
          differentialDrive.tankDrive(  speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER,
                                        -speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER);
        }
      }
    }

    // WHEN GOING BACKWARD:
    else if(speed < 0) {

      // if angle < maxerror1 => drive normally minus error
      if(Math.abs(gyro.getAngle()) <= Constants.GYRO.MAX_ERROR_1) {

        //                 v this part adjusts motor powers to go straight
        tankDrive(speed - (gyro.getAngle() / 30),
                  speed + (gyro.getAngle() / 30)); 
                  System.out.println("under maxerror1");
      }
      // if angle < maxerror2 => slow down motor near center of heading
      else if (Math.abs(gyro.getAngle()) <= Constants.GYRO.MAX_ERROR_2) {
        if(gyro.getAngle() < 0) {
          tankDrive(speed * Constants.GYRO.MOTOR_SLOWDOWN_ON_ERROR2,
                    speed);
        } else if(gyro.getAngle() > 0) {
          tankDrive(speed,
                    speed * Constants.GYRO.MOTOR_SLOWDOWN_ON_ERROR2);
        }
        System.out.println("under maxerror2");
      }
      // if angle > maxerror2 => rotate robot in place
      else {
        if(gyro.getAngle() > 0) {
          differentialDrive.tankDrive(  speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER,
                                        -speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER);
        } else if(gyro.getAngle() < 0) {
          differentialDrive.tankDrive(  -speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER,
                                        speed * Constants.GYRO.ROTATION_SPEED_MULTIPLIER);
        }
        System.out.println("over maxerr2");
      }
    }
    System.out.println("error = " + gyro.getAngle());
  }
  
  /** Controls the robot with simple speed and rotation values.
   * Removes unintentional deadband.
   * */  
  public void arcadeDrive(double xSpeed, double zRotation) {

    if(Math.abs(xSpeed) >= Constants.JOYSTICK.DEADBAND || Math.abs(zRotation) >= Constants.JOYSTICK.DEADBAND) {

      differentialDrive.arcadeDrive(Constants.DRIVETRAIN.SPEED_MULTIPLIER * xSpeed, Constants.DRIVETRAIN.ROTATION_MULTIPLIER * -zRotation);
      
    } else {
      
      differentialDrive.arcadeDrive(0, 0);
      
    }
    
  }
  
  /** Sends raw speeds directly to motors.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    
    // Bounds speeds in range -1 to 1
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -.95, .95);
    
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
    
  }

  /** Return left distance in meters */
  public double getLeftDistance(){
    return leftEncoder.getDistance();
  }
  
  /** Return right distance in meters */
  public double getRightDistance(){
    return rightEncoder.getDistance();
  }
  
  /** Return right distance in meters */
  public double getAverageDistance(){
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  /** Return left encoder speed in meters per second */
  public double getLeftMetersPerSecond() {
    return leftEncoder.getRate();
  }
  
  /** Return right encoder speed in meters per second */
  public double getRightMetersPerSecond() {
    return rightEncoder.getRate();
  }
  
  /** Stop drive motors */
  public void stopDriving() {
    tankDrive(0, 0);
  }

  /** Reset gyro Yaw, make gyro.getAngle() == 0  */
  public void resetGyro() {
    gyro.reset();
    System.out.println("> Gyro heading reset!");
  }

  /** Reset encoders */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
    System.out.println("> resetEncoders() [drivetrain]");
  }

  /** Log important values to Smart Dashboard */
  public void logDriveTrain() {
    
    SmartDashboard.putNumber("DRIVE/leftSpeed in m per s", getLeftMetersPerSecond());
    SmartDashboard.putNumber("DRIVE/rightSpeed in m per s", getRightMetersPerSecond());
    SmartDashboard.putNumber("DRIVE/leftDistance in m", getLeftDistance());
    SmartDashboard.putNumber("DRIVE/rightDistance in m", getRightDistance());
    
    SmartDashboard.putNumber("GYRO/getAngle()", gyro.getAngle());
    SmartDashboard.putNumber("GYRO/getPitch()", gyro.getPitch());
    SmartDashboard.putNumber("GYRO/getYaw()", gyro.getYaw());
    SmartDashboard.putNumber("GYRO/getRoll()", gyro.getRoll());
    SmartDashboard.putNumber("GYRO/getRawGyroX()", gyro.getRawGyroX());
    SmartDashboard.putNumber("GYRO/getRawGyroY()", gyro.getRawGyroY());
    SmartDashboard.putNumber("GYRO/getRawGyroZ()", gyro.getRawGyroZ());
    
    SmartDashboard.putNumber("GYRO/getAccelFullScaleRangeG()", gyro.getAccelFullScaleRangeG());
  }

  /** Log values from PDP */
  public void logPDP() {

    SmartDashboard.putNumber("PDP/Voltage (V)", pdp.getVoltage());
    SmartDashboard.putNumber("PDP/Temperature", pdp.getTemperature());
    SmartDashboard.putNumber("PDP/TotalCurrent (A)", pdp.getTotalCurrent());
    SmartDashboard.putNumber("PDP/TotalPower (W)", pdp.getTotalPower());
    SmartDashboard.putNumber("PDP/TotalEnergy (J)", pdp.getTotalEnergy());

    for (int i = 0; i <= 15; i++) {
      SmartDashboard.putNumber("PDP/PORTCURRENT/" + i, pdp.getCurrent(i));
    }
    
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
    // gyro_sim.setAngle(-m_driveSim.getHeading().getDegrees());                    
                        
  }
  

}
