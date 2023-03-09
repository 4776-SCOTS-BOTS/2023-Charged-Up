    public static double getArmHeight(double shoulderPos, double elbowPos) {
        return -shoulderLength * Math.cos(shoulderPos) - elbowLength * Math.cos(shoulderPos + elbowPos - Math.PI);
    }

    public static double getArmExtension(double shoulderPos, double elbowPos) {
        return shoulderLength * Math.sin(shoulderPos) + elbowLength * Math.cos(shoulderPos + elbowPos - 3 * Math.PI / 2);
    }

    public static boolean heightDanger(double shoulderPos, double elbowPos) {
        return getArmHeight(shoulderPos, elbowPos) >= MAX_HEIGHT;
    }

    public static boolean extensionDanger(double shoulderPos, double elbowPos) {
        // Checking for negative values here as critical extension is behind robot.
        return getArmExtension(shoulderPos, elbowPos) <= -MAX_EXTENSION;
    }

    public static double heightSensShoulder(double shoulderPos, double elbowPos) {
        return shoulderLength * Math.sin(shoulderPos) - elbowLength * Math.sin(shoulderPos + elbowPos);
    }

    public static boolean heightSensShoulderIsPositive(double shoulderPos, double elbowPos) {
        return heightSensShoulder(shoulderPos, elbowPos) >= 0;
    }

    public static double heightSensElbow(double shoulderPos, double elbowPos) {
        return -elbowLength * Math.sin(shoulderPos + elbowPos);
    }

    public static boolean heightSensElowIsPositive(double shoulderPos, double elbowPos) {
        return heightSensElbow(shoulderPos, elbowPos) >= 0;
    }

    public static double extensionSensShoulder(double shoulderPos, double elbowPos) {
        return shoulderLength * Math.cos(shoulderPos) - elbowLength * Math.cos(shoulderPos + elbowPos);
    }

    public static boolean extensionSensShoulderIsPositive(double shoulderPos, double elbowPos) {
        // Extension critical value is negative. Need to check of decreasing value in
        // critical regions
        return extensionSensShoulder(shoulderPos, elbowPos) >= 0;
    }

    public static double extensionSensElbow(double shoulderPos, double elbowPos) {
        return -elbowLength * Math.cos(shoulderPos + elbowPos);
    }

    public static boolean extensionSensElbowIsPositive(double shoulderPos, double elbowPos) {
        // Extension critical value is negative. Need to check of decreasing value in
        // critical regions
        return extensionSensElbow(shoulderPos, elbowPos) >= 0;
    }

    public static boolean elbowPowerOk(double elbowPower, double shoulderPos, double elbowPos) {
        boolean powerOk;

        if (heightDanger(shoulderPos, elbowPos)) {
            powerOk = (Math.signum(elbowPower) * Math.signum(heightSensElbow(shoulderPos, elbowPos))) <= 0;
        } else if (extensionDanger(shoulderPos, elbowPos)) {
            powerOk = (Math.signum(elbowPower) * Math.signum(extensionSensElbow(shoulderPos, elbowPos))) >= 0;
        } else {
            powerOk = true;
        }

        return powerOk;
    }

    public static boolean shoulderPowerOk(double shoulderPower, double shoulderPos, double elbowPos) {
        boolean powerOk;

        if (heightDanger(shoulderPos, elbowPos)) {
            powerOk = (Math.signum(shoulderPower) * Math.signum(heightSensShoulder(shoulderPos, elbowPos))) <= 0;
        } else if (extensionDanger(shoulderPos, elbowPos)) {
            powerOk = (Math.signum(shoulderPower) * Math.signum(extensionSensShoulder(shoulderPos, elbowPos))) >= 0;
        } else {
            powerOk = true;
        }

        return powerOk;
    }
