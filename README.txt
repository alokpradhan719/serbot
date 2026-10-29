SAFEPATH - AUTONOMOUS PORTER ROBOT
===================================

FOLDERS
  firmware/safepath_auto/safepath_auto.ino  -> upload to the ESP32 (needs ESP32Servo library)
  dashboard/index.html                      -> open on the laptop (double-click). Works offline.

The robot only executes simple commands and brakes by itself.
The laptop does all thinking: mapping, path planning, obstacle detours, shuttle.
Wiring is exactly the same as before. Nothing new to buy.

QUICK START
 1. Upload safepath_auto.ino (USB), then unplug USB and switch the battery on.
 2. Laptop: join Wi-Fi "SafePath_Robot" (password SafePath123).
 3. Open dashboard/index.html in Chrome or Edge, click Connect.
    If the browser asks to allow access to local network devices, click Allow.
    If Connect still fails: open a terminal in the dashboard folder, run
        python -m http.server 8000
    and open http://localhost:8000
 4. Calibrate (Calibration & settings): "Test: drive 2 s" -> measure cm -> Apply.
    "Test: turn 90°" -> enter how many degrees it really turned -> Apply.
 5. Mark a HOME spot on the floor with tape. Put the robot there facing "ahead".
 6. Click "Initiate mapping". The robot explores, then comes back home.
 7. Start = home, End = picked automatically (or click Set start / Set end on the map).
 8. Set trips, click "Start shuttle". STOP button halts everything at any time.

TIPS
 - Use a small, tidy area first (about 3 m x 3 m) with straight walls or cardboard walls.
 - Without wheel sensors, position is estimated from time and corrected with the walls.
   If the robot gets confused: STOP, put it on the home mark facing ahead,
   click "Robot is at home" (Map file section), then continue.
 - Save the map (Map file > Save map) so you do not need to map again next time.
 - Recalibrate when the battery is getting low (the robot gets slower).
 - The phone page at http://192.168.4.1 still works for manual driving.
