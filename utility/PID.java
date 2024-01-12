/*
From GM0.

The most common method for tuning a PID controller is as follows:

    Set the I and D gains to zero

    Increase the P gain until there are oscillations around the target

    Increase the D gain until no overshoot occurs

    If there is steady state error, increase the I gain until it is corrected
*/

package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PID {

    public double Kp, Ki, Kd, magnitude;

    private double errorPrev, integral, t = 0;

    private ElapsedTime runtime = new ElapsedTime();

    public PID(double Kp, double Ki, double Kd, double magnitude) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.magnitude = magnitude;
    }

    public double out(double error) {
        double seconds = runtime.seconds();
        double dt = seconds - t;
        integral += error * dt;
        double derivative = (error - errorPrev) / dt;
        errorPrev = error;
        t = seconds;
        return Range.clip(Kp * error + Ki * integral + Kd * derivative, -magnitude, magnitude);
    }

    public void restart() {
        t = runtime.seconds();
    }

    public static void bulkMode() {
        List<LynxModule> allHubs = Mollusc.opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            module.clearBulkCache();
        }
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
