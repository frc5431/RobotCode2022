package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team5431.titan.swerve.sim.*;

public class Simulation {
    private final List<SwerveModuleSim> modules;

    @SuppressWarnings("unused")
    private final QuadSwerveSim qSwerveSim;

    public Simulation(RobotContainer rc) {
        modules = new ArrayList<>();
        modules.add(createSim(rc.getSystems().getDrivebase().getSwerveModules().get(0), "FL"));
        modules.add(createSim(rc.getSystems().getDrivebase().getSwerveModules().get(1), "FR"));
        modules.add(createSim(rc.getSystems().getDrivebase().getSwerveModules().get(2), "BL"));
        modules.add(createSim(rc.getSystems().getDrivebase().getSwerveModules().get(3), "BR"));

        qSwerveSim = new QuadSwerveSim(
                Constants.DRIVETRAIN_TRACKWIDTH_METERS, 
                Constants.DRIVETRAIN_WHEELBASE_METERS, 
                Constants.ROBOT_MASS_KG, 
                1.0/12.0 * Constants.ROBOT_MASS_KG * Math.pow((Constants.DRIVETRAIN_TRACKWIDTH_METERS*1.1),2) * 2, //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
                modules
        );
    }

    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void simulationPeriodic() {

    }

    public static SwerveModuleSim createSim(SwerveModule module, String namePrefix) {
        // ModuleConfiguration modConfig = module.getModuleConfiguration();
        MechanicalConfiguration modConfig = SdsModuleConfigurations.MK4_L2;
        return new SwerveModuleSim(DCMotor.getFalcon500(1),
                                   DCMotor.getFalcon500(1),
                                   modConfig.getWheelDiameter() / 2,
                                   1 / modConfig.getSteerReduction(),
                                   1 / modConfig.getDriveReduction(),
                                   1.0, // CANCoder is directly on the shaft
                                   1 / modConfig.getDriveReduction(),
                                   1.1,
                                   0.8,
                                   Constants.ROBOT_MASS_KG * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                   0.01, namePrefix
                                   );
    }
}
