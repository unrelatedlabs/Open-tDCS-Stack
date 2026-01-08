The code should be minimal, as simple and readable as it gets. no verbose comments. code is the documentation.

UI is simple and light. calm and spacey. zen but not tacky

-

Open Source tDCS

2mA capable with saline electrodes 

for mental health 

Electronics is xiao BLE sense with nrf52840 + 4x cr2025 battery  + simple current source with pwm + rc filter driving opamp based constant current.
we also measure voltage and current for measuring electrode impedance and detecting wearing 

so adc has
- voltage over a 180ohm current sensing resistor for 360mV at 2mA

- voltage of the battery from a resistor divider 12V divided by 6 for 2V

- voltage from output of current source divided by 6

the diff is electrode voltage

.

arduino sketch controls it over BLE

there is a characteristic with 5x 16 bit 
 - set current 
 - measured current
 - measured battery voltage 
 - measured current source voltage
 - measured impedance

- there is a characteristic for setting/reading a 16bit second countdown timer. with target current, total time, ramp up time, ramp down time. ( each 16bit)

PWM output dutty cycle controls current with 4mA full output range 

RGB led shows status with Blue for connected and Green for powered and red for session in progress. 

-

react native app controls it over ble.

it allows starting a session with duration current and ramp setting. there’s is a current and impedance graph 
+ area for notes
. full list is displayed in main screen.  with duration time and set current 

there is a chart with date on x and duration on y

there is audio feedback on session end 

session can run in background with live activity displaying the timer 

first build is for ios only. but should be android an mac. compatible too.

