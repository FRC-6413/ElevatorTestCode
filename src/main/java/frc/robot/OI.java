/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick Controller;
  JoystickButton MoveToOne; 
  JoystickButton MoveToTwo; 

  public OI(){
    Controller = new Joystick(0);  
    MoveToOne = new JoystickButton(Controller, 1);
    MoveToTwo = new JoystickButton(Controller, 2);

    MoveToOne.whenPressed(new frc.robot.commands.MoveToOne());
    MoveToTwo.whenPressed(new frc.robot.commands.MoveToTwo());
  }
}
