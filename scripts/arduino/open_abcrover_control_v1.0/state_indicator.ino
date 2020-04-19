int interval_LED[2] = {0, 0};
int last_time_led = 0;
bool led[2] = {false, false};

// color of the LEDs : (0-255 for green, 0-255 for red, 0-255 for blue) in series
void rover_state_indicator(int mode, float voltage[2])
{
    // set of colors for the LEDs : G, R, B
    int colors[3] = {0, 0, 0};
    // check the operation mode of the rover
    if (mode == 0) // if mode is 0, the rover is disabled
    {
        //blue
        colors[0] = 0;
        colors[1] = 0;
        colors[2] = 50;
    }
    else if (mode == 1) //if mode is 1, the rover is teleoperated without turbo
    {
        //green
        colors[0] = 50;
        colors[1] = 0;
        colors[2] = 0;
    }
    else if (mode == 2) //if mode is 2, the rover is teleoperated with turbo
    {
        //yellow
        colors[0] = 50;
        colors[1] = 50;
        colors[2] = 0;
    }
    else if (mode == 3) //if mode is 3, the rover is locomoting autonomously
    {
        //white
        colors[0] = 50;
        colors[1] = 50;
        colors[2] = 50;
    }
    else // there is something wrong
    {
        // red
        colors[0] = 0;
        colors[1] = 50;
        colors[2] = 0;
    }

    // check the voltage for battery(#1 for onboard PC, #2 for motors)
    int blink_interval[2] = {0, 0}; //[msec]
    for (int i = 0; i < 2; i++)
    {
        if (voltage[i] < 13.5) // 3.375[V/cell]
        {
            blink_interval[i] = 250; //blink rapidly
        }
        else if (voltage[i] < 14.4) // 3.6[V/cell]
        {
            blink_interval[i] = 750; // blink slowly
        }
        else // voltage is enough
        {
            blink_interval[i] = 0; // no blinking
        }
    }

    int current_time_led = millis();
    for (int j = 0; j < 2; j++)
    {
        interval_LED[j] += current_time_led - last_time_led;
        if (blink_interval[j] != 0)
        {
            if (interval_LED[j] > blink_interval[j])
            {
                led[j] = !led[j];    //flip the true/false
                interval_LED[j] = 0; // reset count up of the interval
            }

            if (led[j] == true)
            {
                pixels.setPixelColor(j, pixels.Color(colors[0], colors[1], colors[2]));
                pixels.show();
            }
            else
            {
                pixels.setPixelColor(j, pixels.Color(0, 0, 0));
                pixels.show();
            }
        }

        else
        {
            pixels.setPixelColor(j, pixels.Color(colors[0], colors[1], colors[2]));
            pixels.show();
        }
    }
    last_time_led = current_time_led;
    // Serial.println(led[0]);
}