/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.MoveToOne;

public class ElevatorSubsystem extends PIDSubsystem {
 
  WPI_TalonSRX elevator = Robot.LeftMotor;
  private static final double kp = 0.95;
  private static final double ki = 0.005;
  private static final double kd = 0.0;

  public ElevatorSubsystem() {
    super("ElevatorSubsystem", kp, ki, kd);

    //elevator.setSelectedSensorPosition(0);
    setSetpoint(2.4);    
    //getPIDController().setContinuous(false);
    setAbsoluteTolerance(.05);

    enable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveToOne());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    //double value = ((double)elevator.getSelectedSensorPosition())/maxCounts;
    //System.out.println(value);
    return Robot.WristPot.getAverageVoltage();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    elevator.set(output);
    System.out.println("output: " + output + ", sensor: " + Robot.WristPot.getAverageVoltage());
    //System.out.println(elevator.getSelectedSensorPosition());
  }
}
