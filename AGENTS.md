The code should be minimal, as simple and readable as it gets. no verbose comments. code is the documentation.

UI is simple and light. calm and spacey. zen but not tacky

-

Open Source tDCS

2mA capable with saline electrodes 

for mental health 

Electronics is xiao BLE sense with nrf52840 + 4x cr2025 battery + BJT current source with pwm + rc filter.
PWM sets voltage to Vrsense + 0.7V (Vbe) for current control.

adc measures:
- voltage over 1kΩ current sensing resistor
- battery voltage via 5.7x divider
- output voltage via 5.7x divider

impedance = (Vbat - Vout) / current

arduino sketch controls it over BLE

readings characteristic (5x 16bit): set current, measured current, battery voltage, output voltage, impedance

timer characteristic (4x 16bit): target current, time remaining, ramp up, ramp down

RGB led: green=powered, blue=connected, red=session active

-

web gui (index.html) controls it over web bluetooth.

features:
- direct current slider for manual control
- timed sessions with duration, current, ramp settings
- live current and impedance graphs
- session history with expandable graphs
- notes per session
- audio feedback on session end
