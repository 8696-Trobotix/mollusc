package org.firstinspires.ftc.teamcode.mollusc.utility;

import org.firstinspires.ftc.teamcode.mollusc.Mollusc;

import java.util.HashMap;

public class Controls {

    public static HashMap<Object, Boolean> singlePressMarkers = new HashMap<>();
    public static HashMap<Object, Double> holdMarkers = new HashMap<>();

    // Squares `value` while retaining sign. Useful for more natural joystick feel.
    public static double quadratic(double value) {
        return value * Math.abs(value);
    }
    
    public static boolean singlePress(Object marker, boolean value) {
        Boolean previous = singlePressMarkers.get(marker);
        if (previous == null) {
            previous = false;
            singlePressMarkers.put(marker, previous);
        }
        boolean b = previous;
        previous = value;
        if (b || !value) {
            return false;
        }
        return true;
    }

    // Returns true on held value after a specified duration (seconds), false otherwise.
    public static boolean spacedHold(Object marker, boolean value, double duration) {
        Double time = holdMarkers.get(marker);
        double runtime = Mollusc.opMode.getRuntime();
        if (time == null) {
            time = runtime + duration;
            holdMarkers.put(marker, time);
        }
        if (time > runtime) {
            return false;
        }
        holdMarkers.put(marker, runtime + duration);
        return true;
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
