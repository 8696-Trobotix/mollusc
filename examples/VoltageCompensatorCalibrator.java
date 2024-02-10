package org.firstinspires.ftc.teamcode.mollusc.examples;

import org.firstinspires.ftc.teamcode.mollusc.wrapper.*;
import org.firstinspires.ftc.teamcode.mollusc.utility.*;
import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Voltage Compensator Calibrator", group="Mollusc")
@Disabled

public class VoltageCompensatorCalibrator extends MolluscLinearOpMode {

    private VoltageCompensator voltageCompensator;
    private double maxCurrent = 0;

    @Override
    public void molluscRunOpMode() {

        Make make = new Make();
        DcMotorEx motor = make.motor("frontLeft", DcMotorEx.Direction.FORWARD);
        PIDF pidf = new PIDF(0, 0, 0, 0, 0, 0);

        telemetry.addLine("Press (A) to proceed to step 1. Press (B) to proceed to step 2.");
        telemetry.update();
        int step = 0;
        while (step == 0) {
            if (gamepad1.a) {
                step = 1;
            } else if (gamepad1.b) {
                step = 2;
            }
            idle();
        }
        
        if (step == 1) {
            telemetry.addLine("Before starting, be sure to use a fully-charged battery (14V).");
            telemetry.addData("Current Voltage", VoltageCompensator.getVoltage());
            telemetry.addLine("Press start to begin step 1: retrieve max current draw and velocity.");
            telemetry.update();
        
            waitForStart();

            ElapsedTime timer = new ElapsedTime();
            Filter.LowPass filter1 = new Filter.LowPass(0, 0.5);
            Filter.LowPass filter2 = new Filter.LowPass(0, 0.8);
            double maxCurrent = 0, maxVelocity = 0;
            while (timer.seconds() < 5.0) {
                maxCurrent = Math.max(maxCurrent, filter1.out(motor.getCurrent(CurrentUnit.AMPS)));
                maxVelocity = Math.max(maxVelocity, filter2.out(motor.getVelocity()));
                telemetry.addData("Max Current", maxCurrent);
                telemetry.addData("Max Velocity", maxVelocity);
                telemetry.update();
            }
            telemetry.addLine("Determined Values");
            telemetry.addData("Max Current", maxCurrent);
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addLine("Press (A) to end. Swap out the battery for one of lower voltage and continue with step 2.");
            telemetry.update();
            while (!gamepad1.a && !isStopRequested()) {
                idle();
            }
        } else if (step == 2) {
            telemetry.addLine("Before starting use a battery with a lower voltage than in step 1.");
            telemetry.addData("Current Voltage", VoltageCompensator.getVoltage());
            telemetry.addLine("Press start to begin step 2: adjust PIDF Kd coefficient.");
            telemetry.update();
        
            waitForStart();

            configDoubleNonBlocking(maxCurrent, 1.0, (double value) -> {
                telemetry.addLine("Input Max Current");
                telemetry.addData("Max Current", maxCurrent);
                maxCurrent = value;
            });
            voltageCompensator = new VoltageCompensator(motor, pidf, maxCurrent);
            configDoubleNonBlocking(pidf.Kd, 1.0, (double value) -> {
                telemetry.addLine("Adjust Kd until the motor velocity matches what it was measured in step 1.");
                telemetry.addData("Velocity", motor.getVelocity());
                telemetry.addData("Kd", pidf.Kd);
                pidf.Kd = value;
            });
        }
    }

    private void configDoubleNonBlocking(double value, double delta, Synchronous synchronous) {
        while (gamepad1.a && !isStopRequested()) {
            idle();
        }
        while (!gamepad1.a && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                value += delta;
                sleep(250);
            } else if (gamepad1.dpad_down) {
                value -= delta;
                sleep(250);
            } else if (gamepad1.dpad_left) {
                delta /= 10;
                sleep(250);
            } else if (gamepad1.dpad_right) {
                delta *= 10;
                sleep(250);
            }
            synchronous.process(value);
            telemetry.addLine("Press (A) to confirm / end.");
            telemetry.addData("Value", value);
            telemetry.addData("Delta", delta);
            telemetry.update();
            idle();
        }
        while (gamepad1.a && !isStopRequested()) {
            idle();
        }
    }

    @FunctionalInterface
    private interface Synchronous {
        void process(double value);
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
