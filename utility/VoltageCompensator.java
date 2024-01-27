package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.utility.PIDF;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class VoltageCompensator {

    public static final MAX_VOLTAGE = 14.0;

    private DcMotorEx motor;
    private PIDF pidf;
    private double maxCurrent;
    private Filter.LowPass filter = new Filter.LowPass(0, 0.5);

    public VoltageCompensator(DcMotorEx motor, PIDF pifd, double maxCurrent) {
        this.motor = motor;
        this.pidf = pidf;
        this.maxCurrent = maxCurrent;
    }

    // Returns a feed-forward value to be added to the original power.
    public double adjustPower(double power, double voltage) {
        double targetCurrent = power / (voltage / MAX_VOLTAGE);
        double actualCurrent = filter.out(motor.getCurrent(CurrentUnit.AMPS)) / maxCurrent;
        return pidf.out(targetCurrent - actualCurrent);
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
