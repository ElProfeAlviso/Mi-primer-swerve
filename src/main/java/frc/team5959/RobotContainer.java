// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// By: Beatriz Marún 5959
package frc.team5959;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team5959.commands.Auto;
import frc.team5959.commands.Drive;
import frc.team5959.subsystems.Chassis;;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private static final Chassis chassis = new Chassis(); //necesitas que exista y esté disponible el chassis para poder manejarlo, entonces se tiene que crear una instancia de este
  private static final PS4Controller control = new PS4Controller(0);
  
  public RobotContainer() { //en el container hay una instancia de cada subsistema
    // Configure the trigger bindings
    chassis.setDefaultCommand(new Drive(chassis, () -> control.getLeftX(), () -> control.getLeftY(), () -> control.getRightX())); //esto es en forma de suppliers, de la manera en la que estabamos acostumbrados sólo se quedaría un valor y no se estaría actulaizando
    configureBindings();
  }


  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Auto(chassis);
  }
}
