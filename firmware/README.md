\# Firmware — Pi 5 / ESP32



\## What's been done



\- Raspberry Pi 5 set up with Python and basic GPIO/serial libraries installed

\- Tested a basic serial connection between the Pi 5 and a spare ESP32 dev board

&#x20; as a proof of concept — confirmed the two can communicate over serial



\## What's next



Blocked on confirming with Craig:

\- Exact servo count/spec per leg

\- Whether one ESP32 drives a whole leg or one per motor

\- Power/battery budget

\- How cloud compute access will be provided



Once confirmed, next step is a minimal end-to-end test:

Pi 5 sends a command → ESP32 receives it → PWM signal → motor moves.

