package org.firstinspires.ftc.teamcode.mollusc.auto;

public abstract class MecanumAutoBase {
    private double moveTimeoutSeconds = 5.0;
    private int staticTimeoutMilliseconds = 500;

    private DrivetrainBaseFourWheel base;
    private Interpreter interpreter;
    private PIDF drivePIDF, strafePIDF, turnPIDF;
    private VoltageCompensator c1, c2, c3, c4;
    private double maxPower = 1.0;

    public void waitDelay(double seconds) throws ParityException {
        LinearOpMode opMode = Configuration.useLinearOpMode();
        ElapsedTime temp = new ElapsedTime();
        while (temp.seconds() < seconds && !opMode.isStopRequested()) {
            opMode.idle();
        }
    }

    public void register() throws ParityException {
        Configuration.useLinearOpMode();
        interpreter.register("drive", (Object[] pose) -> {
            driveTo(
                new Pose(
                    Double.parseDouble((String)pose[0]), 
                    Double.parseDouble((String)pose[1]), 
                    Double.parseDouble((String)pose[2])
                )
            );
        }, String.class, String.class, String.class);
        interpreter.register("wait_seconds", (Object[] pose) -> {
            waitDelay((Double)pose[0]);
        }, Double.class);
        interpreter.register("set_move_timeout_seconds", (Object[] t) -> {
            moveTimeoutSeconds = (Double)t[0];
        }, Double.class);
        interpreter.register("set_static_timeout_milliseconds", (Object[] t) -> {
            staticTimeoutMilliseconds = (Integer)t[0];
        }, Integer.class);
    }

    public double getMoveTimeoutSeconds() {
        return moveTimeoutSeconds;
    }
    public void setMoveTimeoutSeconds(double seconds) {
        this.moveTimeoutSeconds = seconds;
    }

    public int getStaticTimeoutMilliseconds() {
        return staticTimeoutMilliseconds;
    }
    public void setStaticTimeoutMilliseconds(double milliseconds) {
        this.staticTimeoutMilliseconds = milliseconds;
    }

    public DrivetrainBaseFourWheel getBase() {
        return base;
    }

    public Interpreter getInterpreter() {
        return interpreter;
    }
    public void setInterpreter(Interpreter interpreter) {
        this.interpreter = interpreter;
    }

    public PIDF getDrivePIDF() {
        return drivePIDF;
    }
    public PIDF getStrafePIDF() {
        return strafePIDF;
    }
    public PIDF getTurnPIDF() {
        return turnPIDF;
    }

    // Order: frontLeft, frontRight, rearLeft, rearRight.
    public VoltageCompensator[] getVoltageCompensators() {
        return new VoltageCompensator[] {c1, c2, c3, c4};
    }

    public double getMaxPower() {
        return maxPower;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }
}

/*
Copyright 2024 Trobotix 8696

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
