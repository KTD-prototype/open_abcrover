// func at the HW interrupt. read encoders.
void encoder_read() {
        //  refresh each value if it is changed
        //  handle those as a single 2 bit number; 1st bit is High/Low of A, 2nd bit is High/Low of B
        //  e.g.1: if pulse A is HIGH, pulse value is 1. Shift it to left, then you get 10. if pulse B is also high, add it to the 10, then you get 11, as a combined information of the A and B
        //  e.g.2: if pulse A is LOW, pulse value is 0. Shift it to left, then you get 00. if pulse B is hig
        pulse_L = !digitalRead(ENC_LA) << 1;
        pulse_L += !digitalRead(ENC_LB);
        pulse_R = !digitalRead(ENC_RA) << 1;
        pulse_R += !digitalRead(ENC_RB);

        //  pulse_L will incremented as 0, 01(1), 11(3), 10(2) or reversed order of them.
        //  exchange value of 2 & 3 since this order is a bit confusing.
        pulse_L = process_byte(pulse_L);
        pulse_R = process_byte(pulse_R);

        // count up encoder(way to increment the count should be different between left and right)
        encoder_count_L = count_encoder(pulse_L, last_pulse_L, encoder_count_L, 'L');
        encoder_count_R = count_encoder(pulse_R, last_pulse_R, encoder_count_R, 'R');

        //  reserve pulse info as last one of those
        last_pulse_L = pulse_L;
        last_pulse_R = pulse_R;
}


byte process_byte(byte pulse) {
        if (pulse == 3) {
                pulse = 2;
        }
        else if (pulse == 2) {
                pulse = 3;
        }
        return pulse;
}



long count_encoder(byte pulse, byte last_pulse, long count, char L_or_R) {
        // process for left motor encoder
        if (L_or_R == 'L') {
                //  exception process
                if (pulse == last_pulse) {
                        //  do nothing when encoder signal has chattered.
                }
                else if (pulse == 0 && last_pulse == 3) {
                        count--;
                }
                else if (pulse == 3 && last_pulse == 0) {
                        count++;
                }

                //  counting encoder;main process
                else if (abs(pulse - last_pulse) > 1) {
                        // do nothing when encoder signal has hopped(= chattering).
                }

                else if (pulse > last_pulse) {
                        count--;
                }
                else {
                        count++;
                }
        }


        // process for right motor encoder
        else if (L_or_R == 'R') {
                //  exception process
                if (pulse == last_pulse) {
                        //  do nothing when encoder signal has chattered.
                }
                else if (pulse == 0 && last_pulse == 3) {
                        count++;
                }
                else if (pulse == 3 && last_pulse == 0) {
                        count--;
                }

                //  counting encoder;main process
                else if (abs(pulse - last_pulse) > 1) {
                        // do nothing when encoder signal has hopped(= chattering).
                }

                else if (pulse > last_pulse) {
                        count++;
                }
                else {
                        count--;
                }
        }
        return count;
}
