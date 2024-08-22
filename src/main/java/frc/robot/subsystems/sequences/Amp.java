package frc.robot.subsystems.sequences;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Hinge;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Rollers;

public class Amp extends SubsystemBase{
    private final Hinge hinge = new Hinge();
    private final Intake intake = new Intake();
    private final Rollers rollers = new Rollers();


    public Amp(){

    

    }

    public Command lamp(){
        return run(() -> hinge.Setpoints(70).withTimeout(2).andThen(Commands.parallel(hinge.Setpoints(100).withTimeout(3), intake.flywheel(12.0).withTimeout(3), rollers.roller(-40.0).withTimeout(3))));

    }

    @Override
    public void periodic() {
        
    }
    @Override
    public void simulationPeriodic() {

    }

    



}
