package org.firstinspires.ftc.teamcode.mollusc.utilities;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Configuration {

    public static Telemetry telemetry;

    // public static String 

    // public static void getBoolean(String label, boolean defaultValue) {
    //     getBoolean(label, "true", "false", defaultValue);
    // }
    // public static void getBoolean(String label, String trueLabel, String falseLabel, boolean defaultValue) {
    // }
    // public static void saveBoolean(boolean value) {
    // }
    // public static boolean loadBoolean() {
    // }

    public long BOOL_DELAY = 100;
    public long DOUBLE_DELAY = 250;
    public long FULL_CHARGE_VOLTAGE = 14;

    public boolean bool(OpMode opMode, LinearOpMode linearOpMode, Gamepad gamepad1, boolean defaultValue, String caption, String label0, String label1) {
        boolean autoClear = telemetry.isAutoClear();
        if (autoClear) telemetry.setAutoClear(false); // Can't clear items now.
        boolean b = defaultValue; // Not strictly necessary, but good for being explicit.
        Telemetry.Item telItem = telemetry.addData(caption, b ? label1 : label0);
        while (!gamepad1.a) {
            telItem.setValue(b ? label1 : label0);
            telemetry.update();
            if (gamepad1.dpad_up) b = false;
            else if (gamepad1.dpad_down) b = true;
            if (opMode == null){
                linearOpMode.sleep(BOOL_DELAY);
            } else {
                // opMode.sleep(BOOL_DELAY);
            }
        }
        while (gamepad1.a); // In case they hold A for too long.
        telItem.setCaption(caption.toUpperCase());
        telemetry.update();
        if (autoClear) telemetry.setAutoClear(true); // Reset to original state.
        return b;
    }

    public double float64(OpMode opMode, LinearOpMode linearOpMode, Gamepad gamepad1, double defaultValue, double var, int precision, String caption) {
        boolean autoClear = telemetry.isAutoClear();
        if (autoClear) telemetry.setAutoClear(false);
        double num = defaultValue;
        Telemetry.Item telItem = telemetry.addData(caption, (long) (num * Math.pow(10, precision)) / Math.pow(10, precision));
        while (!gamepad1.a) {
            telItem.setValue((long) (num * Math.pow(10, precision)) / Math.pow(10, precision));
            telemetry.update();
            if (gamepad1.dpad_up) num += var;
            else if (gamepad1.dpad_down) num -= var;
            if (opMode == null){
                linearOpMode.sleep(BOOL_DELAY);
            } else {
                // opMode.sleep(BOOL_DELAY);
            }
        }
        while (gamepad1.a);
        telItem.setCaption(caption.toUpperCase());
        telemetry.update();
        if (autoClear) telemetry.setAutoClear(true);
        return num;
    }

    // public double batteryPower() { // Returns infinity upon error. This is not a level, but the power from 0 to 1.
    //     double p = Double.POSITIVE_INFINITY;
    //     for (VoltageSensor sensor : hardwareMap.voltageSensor) {
    //         double voltage = sensor.getVoltage();
    //         if (voltage > 0) p = Math.min(p, voltage);
    //     }
    //     return (p == Double.POSITIVE_INFINITY ? p : p / FULL_CHARGE_VOLTAGE);
    // }
    
    public void success() {
        telemetry.log().add("CONFIGURATION COMPLETE");
        telemetry.update();
        telemetry.speak("Configuration Complete");
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
