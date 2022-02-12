// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDelay.h"

AutoDelay::AutoDelay(units::time::second_t seconds) : m_seconds(seconds) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_timer = frc::Timer(); 
}

// Called when the command is initially scheduled.
void AutoDelay::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  // m_seconds = (units::time::second_t)m_driveTrain->m_nte_a_DriveDelay.GetDouble(0.0); // Seconds delay before driving
  #ifdef DEBUG && DEBUG == TRUE
    printf("AutoDelay()::Initialize()\n");
  #endif
}   

// Called repeatedly when this Command is scheduled to run
// void AutoDelay::Execute() {}

// Called once the command ends or is interrupted.
void AutoDelay::End(bool interrupted) {
  #ifdef DEBUG
    printf("AutoDelay::End()\n");
  #endif
}

// Returns true when the command should end.
bool AutoDelay::IsFinished() {
  // m_timer.Get() returns a units::time::second_t
  return m_timer.Get() > m_seconds;
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoDriveDistance.h"

AutoDriveDistance::AutoDriveDistance(DriveTrain *drivetrain) : m_driveTrain(drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

// Called when the command is initially scheduled.
void AutoDriveDistance::Initialize() {
  m_driveTrain->ResetEncoders();
  // Read the desired travel distance from network tables (shuffleboard driver input)
  m_distance_inches = m_driveTrain->m_nte_b_DriveDistance.GetDouble(0.0);
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveDistance::Execute() {
  // See TeleOpDrive for more filtering information
  constexpr double speedN = 11.0; // length of digital filter
  constexpr double maxSpeed = 0.5;
  constexpr double rotation = 0.0;

  double desiredSpeed = (m_distance_inches > m_driveTrain->GetAverageDistanceInches()) ? maxSpeed : -maxSpeed;
  double speed = (((speedN - 1.0) * m_speedOut) + desiredSpeed) / speedN;
  m_driveTrain->ArcadeDrive(speed, rotation);
  m_speedOut = speed;
}

// Called once the command ends or is interrupted.
void AutoDriveDistance::End(bool interrupted) {
  m_driveTrain->ArcadeDrive(0.0, 0.0);
}

// Returns true when the command should end.
bool AutoDriveDistance::IsFinished() {
  constexpr double epsilon = 5.0;

  return ((fabs(m_distance_inches + copysign(epsilon / 2.0, m_distance_inches))- m_driveTrain->GetAverageDistanceInches()) < epsilon);
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TeleOpDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>

TeleOpDrive::TeleOpDrive(DriveTrain *drivetrain,
                         std::function<double()> speed,
                         std::function<double()> rotation)
            : m_driveTrain(drivetrain),
              m_speed(speed),
              m_rotation(rotation) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

#ifdef ENABLE_DRIVETRAIN
// Called when the command is initially scheduled.
void TeleOpDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleOpDrive::Execute() {
  // Digital filter lengths- between 1.0 (no filter) and 20.0 (90% at 1 second) (11.0 is 90% at 0.5 sec)
  // Idea from simple filter at https://www.chiefdelphi.com/t/moderating-acceleration-deceleration/77960/4

  // Get adjustment values
  double speedN = m_driveTrain->m_nte_DriveSpeedFilter.GetDouble(10.0);
  double rotationN = m_driveTrain->m_nte_DriveRotationFilter.GetDouble(8.0);
  if (speedN < 1.0) { speedN = 1.0; }
  if (rotationN < 1.0) { rotationN = 1.0; }
  double exponent = m_driveTrain->m_nte_InputExponent.GetDouble(1.0);
  if (exponent < 1.0) { exponent = 1.0; }
  if (exponent > 3.0) { exponent = 3.0; }

  // Adjust input speed with exponentiation
  double speed = m_speed();
  double adjustedSpeed = copysign(pow(fabs(speed), exponent), speed);

  // Adjust input speed and input rotation with filters
  speed = (((speedN - 1.0) * m_speedOut) + adjustedSpeed) / speedN;
  double rotation = (((rotationN - 1.0) * m_rotationOut) + m_rotation()) / rotationN;

  // Do it
  m_driveTrain->ArcadeDrive(speed, rotation);

  // Display the distance we've driven
  m_driveTrain->m_nte_Testing.SetDouble(m_driveTrain->GetLeftDistanceInches());

  m_speedOut = speed;
  m_rotationOut = rotation;

  // Options for more accurate time:
  //frc::RobotController::GetFPGATime()/1000
  // For Linear Filters:
  // From https://docs.wpilib.org/en/latest/docs/software/advanced-control/filters/linear-filter.html#creating-a-linearfilter
  //frc::LinearFilter<double> filter = frc::LinearFilter<double>::SinglePoleIIR(0.1_s, 0.02_s);
}

// Called once the command ends or is interrupted.
void TeleOpDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleOpDrive::IsFinished() { return false; }

#endif // ENABLE_DRIVETRAIN
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {
#ifdef ENABLE_DRIVETRAIN
  // Settings for Spark Max motor controllers should be done here, in code
  // and not in the Spark Max Client Software
  m_rightMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_rightMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorA.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);
  m_leftMotorB.SetOpenLoopRampRate(ConDriveTrain::RAMP_RATE);

  /* 
    To swap front & back of robot, swap the INVERTED/NONINVERTED below and add or remove the minus sign 
    in RobotContainer.cpp on the line which contains "ConXBOXControl::LEFT_JOYSTICK_X/4" for the turning axis
  */
  // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
  m_rightMotorA.SetInverted(ConDriveTrain::NONINVERTED);
  m_rightMotorB.SetInverted(ConDriveTrain::NONINVERTED);
  // WARNING! TuffBox Mini requires BOTH motors spin in the same direction!!!
  m_leftMotorA.SetInverted(ConDriveTrain::INVERTED);
  m_leftMotorB.SetInverted(ConDriveTrain::INVERTED);

  // set PID coefficients
  // left pid controller
  left_pidController.SetP(kP);
  left_pidController.SetI(kI);
  left_pidController.SetD(kD);
  left_pidController.SetIZone(kIz);
  left_pidController.SetFF(kFF);
  left_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  // right pid controller
  right_pidController.SetP(kP);
  right_pidController.SetI(kI);
  right_pidController.SetD(kD);
  right_pidController.SetIZone(kIz);
  right_pidController.SetFF(kFF);
  right_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  /**
   * Smart Motion coefficients are set on a SparkMaxPIDController  object
   * 
   * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
   * the pid controller in Smart Motion mode
   * - SetSmartMotionMinOutputVelocity() will put a lower bound in
   * RPM of the pid controller in Smart Motion mode
   * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
   * of the pid controller in Smart Motion mode
   * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
   * error for the pid controller in Smart Motion mode
   */
  // left pid controller
  left_pidController.SetSmartMotionMaxVelocity(kMaxVel);
  left_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
  left_pidController.SetSmartMotionMaxAccel(kMaxAcc);
  left_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  //right pid controller
  right_pidController.SetSmartMotionMaxVelocity(kMaxVel);
  right_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
  right_pidController.SetSmartMotionMaxAccel(kMaxAcc);
  right_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);

  // Set additional motor controllers on drive train to follow
  m_rightMotorB.Follow(m_rightMotorA, false);
  m_leftMotorB.Follow(m_leftMotorA, false);

  // NavX gyro
  gyro = new AHRS(frc::SPI::Port::kMXP);

  /*
     FIXME: This may be a better way to set the distance conversion: Right on the SparkMax!
     Native Tick counts * Gear Ratio divided by Wheel circumference (42 * 10.71)/(6 * pi) = ticks per inch
     We can use the SetPositionConversionFactor() to use this as our tick reference.
     // m_leftEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_rightEncoderA.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_leftEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
     // m_rightEncoderB.SetPositionConversionFactor(ConDriveTrain::TICKS_PER_INCH);
    BUUUT: IF YOU DO THIS, CHANGE THE GetLeftDistanceInches() and GetRightDistanceInches() methods!!!

  */

  // Save all SparkMax firmware parameters to flash memory
  m_leftMotorA.BurnFlash();
  m_leftMotorB.BurnFlash();
  m_rightMotorA.BurnFlash();
  m_rightMotorB.BurnFlash();

#endif // ENABLE_DRIVETRAIN

  // Create and get reference to SB tab
  m_sbt_DriveTrain = &frc::Shuffleboard::GetTab(ConShuffleboard::DriveTrainTab);

  // Create widgets for digital filter lengths
  m_nte_DriveSpeedFilter    = m_sbt_DriveTrain->AddPersistent("Drive Speed Filter", 10.0)   .WithSize(2, 1).WithPosition(0, 0).GetEntry();
  m_nte_DriveRotationFilter = m_sbt_DriveTrain->AddPersistent("Drive Rotation Filter", 5.0) .WithSize(2, 1).WithPosition(0, 1).GetEntry();

  // Create widget for non-linear input
  m_nte_InputExponent       = m_sbt_DriveTrain->AddPersistent("Input Exponent", 1.0)        .WithSize(1, 1).WithPosition(0, 2).GetEntry();

  // Create widgets for AutoDrive
  m_nte_a_DriveDelay        = m_sbt_DriveTrain->AddPersistent("a Drive Delay", 0.0)         .WithSize(1, 1).WithPosition(3, 0).GetEntry();
  m_nte_b_DriveDistance     = m_sbt_DriveTrain->AddPersistent("b Drive Distance", 0.0)    .WithSize(1, 1).WithPosition(3, 1).GetEntry();
  m_nte_c_DriveTurnAngle     = m_sbt_DriveTrain->AddPersistent("c Turn Angle", 0.0)       .WithSize(1, 1).WithPosition(3, 2).GetEntry();
  //  m_nte_Testing     = m_sbt_DriveTrain->AddPersistent("Testing", 0.0)       .WithSize(1, 1).WithPosition(3, 3).GetEntry();

  // Display current encoder values
  m_nte_LeftEncoder = m_sbt_DriveTrain->AddPersistent("Left Side Encoder", 0.0)             .WithSize(2,1).WithPosition(4,0).GetEntry();
  m_nte_RightEncoder = m_sbt_DriveTrain->AddPersistent("Right Side Encoder", 0.0)            .WithSize(2,1).WithPosition(4,1).GetEntry();
  m_nte_IMU_ZAngle = m_sbt_DriveTrain->AddPersistent("IMU Z-Axis Angle", 0.0)               .WithSize(2,1).WithPosition(4,2).GetEntry();

  // End of DriveTrain Constructor
  printf("DriveTrain() Constructor returning...\n");
}

#ifdef ENABLE_DRIVETRAIN
// This method will be called once per scheduler run
void DriveTrain::Periodic() {
  m_nte_LeftEncoder.SetDouble(GetAverageLeftEncoders());
  m_nte_RightEncoder.SetDouble(GetAverageRightEncoders());
  m_nte_IMU_ZAngle.SetDouble(GetGyroAngle());
}

// Used by TeleOpDrive
void DriveTrain::ArcadeDrive(double speed, double rotation) {
  m_driveTrain.ArcadeDrive(speed, DeadZone(rotation));
}

// Used by AlignToPlayerStationPID
void DriveTrain::TankDrive(double left, double right){
  m_driveTrain.TankDrive(left, right);
}

// Used by TeleOpSlowDrive
double DriveTrain::GetMaxOutput() {
    return m_maxOutput;
}
// Why is this method recursive?
void DriveTrain::SetMaxOutput(double maxOutput) {
  m_maxOutput = maxOutput;
  m_driveTrain.SetMaxOutput(maxOutput);
}

// Used by AutoDriveDistance
void DriveTrain::ResetEncoders() {
  m_rightEncoderA.SetPosition(0.0);
  m_rightEncoderB.SetPosition(0.0);
  m_leftEncoderA.SetPosition(0.0);
  m_leftEncoderB.SetPosition(0.0);
}

// Account for two encoders per side
double DriveTrain::GetRightDistanceInches() {
  return (GetAverageRightEncoders() * ConDriveTrain::INCHES_PER_TICK);
}

double DriveTrain::GetLeftDistanceInches() {
  return (GetAverageLeftEncoders() * ConDriveTrain::INCHES_PER_TICK);
}

// Used by AutoDriveDistance
double DriveTrain::GetAverageDistanceInches() {
  // FIXME: Should't these be added, or is one negative? I think we just REVERSE the encoder. CRE 2022-01-25
  return ((GetLeftDistanceInches() + GetRightDistanceInches()) / 2.0);
}

double DriveTrain::GetAverageLeftEncoders() {
  return (m_leftEncoderA.GetPosition() + m_leftEncoderB.GetPosition() ) / 2.0;
}

double DriveTrain::GetAverageRightEncoders() {
  return (m_rightEncoderA.GetPosition() + m_rightEncoderB.GetPosition() ) / 2.0;
}
void DriveTrain::GoToAngle(double angle) {
  angle *= ConDriveTrain::ANGLE_2_IN;
  // FIXME: The following syntax is deprecated in 2022 and throws a warning error, but the recommended
  // fix throws a compiler/unknown reference error for CANSparkMax::ControlType
  //  Use SetReference(double, CANSparkMax::ControlType, int, double, SparkMaxPIDController::ArbFFUnits) instead [-Wdeprecated-declarations]
  //  right_pidController.SetReference(angle, rev::ControlType::kSmartMotion);
  left_pidController.SetReference(angle, rev::ControlType::kSmartMotion);
  right_pidController.SetReference(angle, rev::ControlType::kSmartMotion);
}

double DriveTrain::GetGyroAngle() {return gyro->GetAngle();}

// Used by AutoTurn 																	   
void DriveTrain::ResetGyro() {
  gyro->Reset();
}																				 
// void DriveTrain::SetSafety(bool safety) { SetSafetyEnabled(safety);}

#endif // ENABLE_DRIVETRAIN
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>  // for std::fabs
#include <math.h>

//On error, create env.h from env-default.h and modify ROBOT_VERSION_STRING
#include "env.h"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace ConMath {
    constexpr double PI = M_PI; // 3.141592;
    constexpr double METERS_2_INCH = .0254; // m/in
    constexpr double MINUTES_2_SECONDS = 1/60.; // sec/min
    constexpr double RAD_2_DEG = 180.0/PI;
    constexpr double DEG_2_RAD = 1/RAD_2_DEG;
}

namespace ConLimelight {
    constexpr int VISION_MODE = 0;
    constexpr int CAMERA_MODE = 1;

    constexpr int LED_PIPLINE_DEFAULT = 0;
    constexpr int LED_OFF = 1;
    constexpr int LED_BLINK = 2;
    constexpr int LED_ON = 3;

    constexpr int SNAPSHOT_STOP = 0;
    constexpr int SNAPSHOT_START = 1;

    constexpr double HORIZONTAL_TOLERANCE = 1.0;  //degrees
    constexpr double TARGET_HEIGHT = 38.5; //in to center of target
    constexpr double CAMERA_HEIGHT = 19.5; //in to center of camera
    constexpr double MAX_HORIZONTAL_OFFSET = 29.8; //degrees

    // constexpr cv::Matx33d cameraMatrix = cv::Matx33d(
    //                     772.53876202, 0., 479.132337442,
    //                     0., 769.052151477, 359.143001808,
    //                     0., 0., 1.0);
    // constexpr std::vector istortionCoefficient =  std::vector<double> {
    //                     2.9684613693070039e-01, -1.4380252254747885e+00,-2.2098421479494509e-03,
    //                     -3.3894563533907176e-03, 2.5344430354806740e+00};

    constexpr double focalLength = 2.9272781257541; //mm
}

namespace ConSparkMax {
    constexpr double POSITION_CONVERSION_FACTOR = 42.0;
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AutoDriveDistance
    : public frc2::CommandHelper<frc2::CommandBase, AutoDriveDistance> {
 public:
  AutoDriveDistance(DriveTrain *drivetrain);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveTrain *m_driveTrain;
  double m_distance_inches; // CRE 2022-01-28 Read from shuffleboard NOT passed as argument
  double m_speedOut = 0.0;
};
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <AHRS.h>				 

#include "OI.h"
#include "Constants.h"
#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
using namespace units;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::dimensionless;
using namespace units::length;
using namespace units::time;
using namespace units::voltage;

namespace ConDriveTrain {
    // Motors
    constexpr int RIGHT_MOTOR_A_ID = 2;
    constexpr int RIGHT_MOTOR_B_ID = 4;
    constexpr int LEFT_MOTOR_A_ID = 3;
    constexpr int LEFT_MOTOR_B_ID = 5;
    //constexpr double ROTATION_FACTOR = 1/1.3;

    //Spark Max Settings
    constexpr second_t RAMP_RATE = 0.100_s; //seconds
    constexpr bool INVERTED = true; //
    constexpr bool NONINVERTED = false; //
    
    // Neo Motor & Gearbox
    constexpr scalar_t ENCODER_TICK_RESOLUTION = 42.0; // IS IT REALLY 42? or 48? or maybe 24?  
    constexpr scalar_t GEAR_RATIO = 10.71; // Neo rotates 10.71 times for one rotation of the output
    constexpr meter_t WHEEL_DIAMETER = 6.0_in;
    constexpr meter_t WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI; // Abt 19 in.

    constexpr scalar_t TICKS_PER_WHEEL_REVOLUTION = ENCODER_TICK_RESOLUTION * GEAR_RATIO; // Abt 450 ticks

    //Conversions
    //constexpr scalar_t TICKS_PER_INCH = TICKS_PER_WHEEL_REVOLUTION / WHEEL_CIRCUMFERENCE; // Abt 24 ticks per inch
    constexpr meter_t INCHES_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_WHEEL_REVOLUTION; // Abt 1/24 (.042)

    // degrees to in
    constexpr meter_t ANGLE_2_IN = 25.5_in;
    //constexpr radian_t IN_2_ANGLE = 1.0 / ANGLE_2_IN;
}

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  frc::ShuffleboardTab *m_sbt_DriveTrain;
  nt::NetworkTableEntry m_nte_DriveSpeedFilter;
  nt::NetworkTableEntry m_nte_DriveRotationFilter;
  nt::NetworkTableEntry m_nte_InputExponent;

  // Encoder outputs
  nt::NetworkTableEntry m_nte_LeftEncoder;
  nt::NetworkTableEntry m_nte_RightEncoder;
  nt::NetworkTableEntry m_nte_IMU_ZAngle;

  nt::NetworkTableEntry m_nte_Testing;

  // Autonomous Variables
  nt::NetworkTableEntry m_nte_a_DriveDelay;
  nt::NetworkTableEntry m_nte_b_DriveDistance;
  nt::NetworkTableEntry m_nte_c_DriveTurnAngle;
  
#ifdef ENABLE_DRIVETRAIN
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  /**
   * Drives the robot using arcade controls.
   *
   * @param speed the commanded forward movement
   * @param rotation the commanded rotation
   */
  void ArcadeDrive(double speed, double rotation);

  void TankDrive(double left, double right);

  double GetMaxOutput();

  void SetMaxOutput(double maxOutput);

  inch_t GetRightDistanceInches();
  inch_t GetLeftDistanceInches();
  inch_t GetAverageDistanceInches();
  
  double GetAverageRightEncoders();
  double GetAverageLeftEncoders();

  double GetGyroAngle();
  void ResetEncoders();

  void GoToAngle(double angle);
  void ResetGyro();
  //void SetSafety(bool safety);
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double m_maxOutput = 1.0;
  AHRS *gyro;

  // Neo motor controllers
  rev::CANSparkMax m_rightMotorA{ConDriveTrain::RIGHT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotorB{ConDriveTrain::RIGHT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorA{ConDriveTrain::LEFT_MOTOR_A_ID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotorB{ConDriveTrain::LEFT_MOTOR_B_ID, rev::CANSparkMax::MotorType::kBrushless};

  /* Drive Train Smart Motion PID set-up below */
  rev::SparkMaxPIDController  left_pidController = m_leftMotorA.GetPIDController();
  rev::SparkMaxPIDController  right_pidController = m_rightMotorA.GetPIDController();

  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  const revolutions_per_minute_t MaxRPM = 5700_rpm;


  // Drive encoders
  rev::SparkMaxRelativeEncoder m_rightEncoderA = m_rightMotorA.GetEncoder();
  rev::SparkMaxRelativeEncoder m_rightEncoderB = m_rightMotorB.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftEncoderA = m_leftMotorA.GetEncoder();
  rev::SparkMaxRelativeEncoder m_leftEncoderB = m_leftMotorB.GetEncoder();

  // Robot Drive
  frc::DifferentialDrive m_driveTrain{m_leftMotorA, m_rightMotorA};
#endif // ENABLE_DRIVETRAIN
};
