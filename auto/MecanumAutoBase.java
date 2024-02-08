package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.Interpreter;

import org.firstinspires.ftc.teamcode.mollusc.utility.VoltageCompensator;
import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

public abstract class MecanumAutoBase {
    
    public double moveTimeoutSeconds = 5.0;
    public int staticTimeoutMilliseconds = 500;

    public DrivetrainBaseFourWheel base;
    public Interpreter interpreter = null;
    public PIDF drivePIDF, strafePIDF, turnPIDF;
    public VoltageCompensator c1 = null, c2 = null, c3 = null, c4 = null;
    public double maxPower = 1.0, maxPowerDeltaDurationSeconds = 0.5, closeEnough = 0.1;

    public void resetPIDF() {
        drivePIDF.restart();
        strafePIDF.restart();
        turnPIDF.restart();
    }

    private double utilDelta(double p, double d) {
        p = Math.abs(d) <= closeEnough ? p - d : p - Math.signum(d) * maxPowerDeltaDurationSeconds * dt;
        return p;
    }

    public void waitDelay(double seconds) throws ParityException {
        LinearOpMode opMode = Mollusc.useLinearOpMode("MecanumAuto waitDelay.");
        ElapsedTime temp = new ElapsedTime();
        while (temp.seconds() < seconds && !opMode.isStopRequested()) {
            opMode.idle();
        }
    }

    public void register() throws ParityException {
        Mollusc.useLinearOpMode("MecanumAuto register.");
        if (interpreter != null) {
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
            interpreter.register("set_max_power", (Object[] power) -> {
                maxPower = (Double)power[0];
            }, Double.class);
        }
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
