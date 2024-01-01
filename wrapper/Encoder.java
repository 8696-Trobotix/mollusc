package org.firstinspires.ftc.teamcode.mollusc.wrapper;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    private DcMotorEx encoder;

    private double multiplier;
    private int ticksPerRevolution;
    private double distancePerTick;

    public Encoder(String name, double multiplier, int ticksPerRevolution, double wheelDiameter) {
        this(Mollusc.opMode.hardwareMap.get(DcMotorEx.class, name), multiplier, ticksPerRevolution, wheelDiameter);
    }

    public Encoder(DcMotorEx motor, double multiplier, int ticksPerRevolution, double wheelDiameter) {
        this.encoder = motor;
        this.multiplier = multiplier;
        this.ticksPerRevolution = ticksPerRevolution;
        this.distancePerTick = Math.PI * wheelDiameter / ticksPerRevolution;

        reset();
    }

    public void reset() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getTicks() {
        return (int)(encoder.getCurrentPosition() * multiplier);
    }
    public double getRevolutions() {
        return (double)getTicks() / ticksPerRevolution;
    }
    public double getDisplacement() {
        return getTicks() * distancePerTick;
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
