package frc.robot.commands;

import java.util.function.DoubleSupplier;

public class WaitCommand extends edu.wpi.first.wpilibj2.command.WaitCommand {
    private DoubleSupplier supplier;

    public WaitCommand(double seconds) {
        this(() -> seconds);
    }
    
    public WaitCommand(DoubleSupplier supplier) {
        super(supplier.getAsDouble());
        this.supplier = supplier;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(supplier.getAsDouble());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
