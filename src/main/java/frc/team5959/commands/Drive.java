// By: Beatriz Marún 5959
package frc.team5959.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5959.subsystems.Chassis;

public class Drive extends Command{
    
    Chassis chassis;
    Supplier<Double> xSpeed; //en Command based no se puede instanciar el control como en timed, para comandos se ocupa sólo en el container pero se pasan suplieres
    // pasa valores o metodos del control y los actualiza en vivo
    Supplier<Double> ySpeed;
    Supplier<Double> zSpeed;

    public Drive(Chassis chassis,Supplier<Double> xSpeed,Supplier<Double> ySpeed, Supplier<Double> zSpeed){
        addRequirements(chassis); //si hay otro comando que quiere usar el chassis se reemplazan, así no hay conflictos entre los diferentes comandos
        this.chassis = chassis;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

    }

    @Override //se actualiza cada 20 milisegundos
    public void execute(){
        chassis.setFieldOrientedSpeed(xSpeed.get(), ySpeed.get(), zSpeed.get());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
