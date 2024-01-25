package org.firstinspires.ftc.teamcode.mollusc.utility;

public class Filter {

    public static class LowPass {
        
        public double previousEstimate;
        public double gain;

        public LowPass(double initialValue, double gain) {
            this.previousEstimate = initialValue;
            this.gain = gain;
        }

        // Returns the current estimate based on the previous estimate.
        public double out(double currentValue) {
            double ret = gain * previousEstimate + (1 - gain) * currentValue;
            previousEstimate = ret;
            return ret;
        }
    }

    // Simple SISO Kalman filter from CTRL ALT FTC extended to support multiple inputs.
    public static class SimpleKalman {
        
        public double Q;
        public double[] R, P, K;

        // Q --> confidence in model, R --> accuracy of other inputs.
        public SimpleKalman(double Q, double ...R) {
            this.Q = Q;
            this.R = R;
            this.P = new double[R.length];
            this.K = new double[R.length];
            for (int i = 0; i < R.length; ++i) {
                this.P[i] = this.K[i] = 1;
            }
        }

        public double out(double currentState, double ...inputs) {
            double x = currentState;
            for (int i = 0; i < R.length; ++i) {
                P[i] += Q;
                K[i] = P[i] / (P[i] + R[i]);
                currentState += K[i] * (inputs[i] - x);
                P[i] *= (1 - K[i]);
            }
            return currentState;
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
