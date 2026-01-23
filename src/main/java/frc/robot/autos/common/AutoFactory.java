package frc.robot.autos.common;

import java.util.function.Supplier;

import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotState;
import frc.robot.util.Dashboard;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AutoFactory {
    private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();
    private Auto currentAuto;
    private AutoBase compiledAuto;

    private static LoggedNetworkBoolean autoCompiled =
        new LoggedNetworkBoolean(DashboardConstants.AUTO_COMPILED_KEY, false);
    private static LoggedNetworkString autoDescription =
            new LoggedNetworkString(DashboardConstants.AUTO_DESCRIPTION_KEY, "No Description");

    private boolean isRedAlliance = RobotState.getInstance().isRedAlliance();
    private static AutoFactory INSTANCE;
    public static AutoFactory getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoFactory();
        }
        return INSTANCE;
    }
    private AutoFactory() {};

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto
                || isRedAlliance == !RobotState.getInstance().isRedAlliance();
    }

    public void recompile() {
        isRedAlliance = RobotState.getInstance().isRedAlliance();
        // update auto
        autoCompiled.set(false);
        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }
        compiledAuto = currentAuto.getInstance();
        if (compiledAuto == null) {
            autoDescription.set("No Auto Selected");
        } else {
            compiledAuto.init(); // starts?
        }
        autoCompiled.set(true);
    }

    public static enum Auto {
        NO_AUTO(null);

        private final Class<? extends AutoBase> autoClass;

        private Auto(Class<? extends AutoBase> autoClass) {
            this.autoClass = autoClass;
        }

        public AutoBase getInstance() {
            if (autoClass != null) {
                try {
                    return autoClass.getConstructor().newInstance();

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            return null;
        }
    }
}
