package org.firstinspires.ftc.teamcode.mollusc.auto;

import org.firstinspires.ftc.teamcode.mollusc.drivetrain.DrivetrainBaseFourWheel;

import org.firstinspires.ftc.teamcode.mollusc.auto.interpreter.Interpreter;
import org.firstinspires.ftc.teamcode.mollusc.auto.odometry.Pose;

import org.firstinspires.ftc.teamcode.mollusc.exception.ParityException;

import org.firstinspires.ftc.teamcode.mollusc.utility.VoltageCompensator;
import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumAutoI extends MecanumAutoBase implements Auto {
    
    private IMU imu;
    private double headingOffsetRadians = 0.0;
    private double powerTolerance;

    private boolean useVoltageCompensator = false;
    private double fl, fr, rl, rr;

    public MecanumAutoI(
        DrivetrainBaseFourWheel base, 
        PIDF drivePIDF, 
        PIDF strafePIDF, 
        PIDF turnPIDF, 
        IMU imu, 
        double powerTolerance
    ) {
        this.base = base;
        this.drivePIDF = drivePIDF;
        this.strafePIDF = strafePIDF;
        this.turnPIDF = turnPIDF;
        this.imu = imu;
        this.powerTolerance = powerTolerance;
    }

    public void setInterpreter(Interpreter interpreter) {
        this.interpreter = interpreter;
    }

    public void setVoltageCompensators(PIDF voltageCompensatorPIDF, double maxCurrent) {
        useVoltageCompensator = true;
        c1 = new VoltageCompensator(base.frontLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c2 = new VoltageCompensator(base.frontRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c3 = new VoltageCompensator(base.rearLeft, new PIDF(voltageCompensatorPIDF), maxCurrent);
        c4 = new VoltageCompensator(base.rearRight, new PIDF(voltageCompensatorPIDF), maxCurrent);
    }

    // Field-centric style automated drive.
    // Recommended to use MecanumAutoII instead, as the heading is also accounted for whilst driving.
    public void driveTo(Pose newPose) throws ParityException {
        LinearOpMode opMode = Mollusc.useLinearOpMode("MecanumAutoI driveTo.");

        resetPIDF();

        ElapsedTime runtime = new ElapsedTime();
        int previousTime = 0;

        while (opMode.opModeIsActive() && runtime.seconds() < moveTimeoutSeconds) {
            drivePowers(newPose);

            base.frontLeft.setPower(fl);
            base.frontRight.setPower(fr);
            base.rearLeft.setPower(rl);
            base.rearRight.setPower(rr);

            int currentTime = (int)runtime.milliseconds();
            double powerNet = Math.abs(fl) + Math.abs(fr) + Math.abs(rl) + Math.abs(rr);
            if (Math.abs(powerNet) > powerTolerance) {
                previousTime = currentTime;
            }
            if (currentTime >= previousTime + staticTimeoutMilliseconds) {
                break;
            }
        }

        base.stopAll();
    }

    private void drivePowers(Pose newPose) {
        int[] positions = base.getEncoderCounts();
        double drive = drivePIDF.out(newPose.x - (positions[0] + positions[1] + positions[2] + positions[3]) / 4);
        double strafe = strafePIDF.out(newPose.y - (positions[0] + positions[3]) / 2);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffsetRadians;
        double turn = turnPIDF.out(-1 * AngleUnit.normalizeRadians(Math.toRadians(newPose.z) - heading));

        // Calculations based on GM0.
        double rotX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
        // Normalize. Also prevents power values from exceeding 1.0.
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        double fld = fl - (rotY + rotX + turn) / max * maxPower;
        double frd = fr - (rotY - rotX - turn) / max * maxPower;
        double rld = rl - (rotY - rotX + turn) / max * maxPower;
        double rrd = rr - (rotY + rotX - turn) / max * maxPower;
        fl = utilDelta(fl, fld);
        fr = utilDelta(fr, frd);
        rl = utilDelta(rl, rld);
        rr = utilDelta(rr, rrd);

        double voltage = VoltageCompensator.getVoltage();
        if (useVoltageCompensator) {
            fl = c1.adjustPower(fl, voltage);
            fr = c2.adjustPower(fr, voltage);
            rl = c3.adjustPower(rl, voltage);
            rr = c4.adjustPower(rr, voltage);
        }
    }

    public double[] getDrivePowers(Pose newPose) {
        drivePowers(newPose);
        return new double [] {fl, fr, rl, rr};
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

    public IMU getIMU() {
        return imu;
    }

    public double getHeadingOffsetRadians() {
        return headingOffsetRadians;
    }
    public void setHeadingOffsetRadians(double radians) {
        this.headingOffsetRadians = radians;
    }

    public double getPowerTolerance() {
        return powerTolerance;
    }
    public void setPowerTolerance(double powerTolerance) {
        this.powerTolerance = powerTolerance;
    }
}

/*
Copyright 2023 Trobotix 8696

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
