# T-LoRa Pager simulator

The host build (`web/`) laid out as the LilyGo T-LoRa Pager: 480x222, no touch,
the QWERTY keyboard and the rotary encoder as the only input, and the UI's own
`TLORA_PAGER` branches compiled in. It is for reviewing and scripting the Pager's
screens without flashing a device after every layout change.

```bash
web/fetch-deps.sh
cmake -S web -B web/build-pager -G Ninja -DCMAKE_BUILD_TYPE=Release -DWADA_SIM_PAGER=ON
cmake --build web/build-pager --target wadamesh-host

# interactive (SDL window, 2x)
WADA_DATA_DIR=/tmp/pager-data web/build-pager/wadamesh-host --size 480x222 --scale 2

# headless screenshots
WADA_DATA_DIR=/tmp/pager-data web/build-pager/wadamesh-host --headless --size 480x222 \
  --script web/sim_pager/tour.wada --shots /tmp/pager-shots
```

The simulator works inside `WADA_DATA_DIR` (the Pager's tile partition has no
directory of its own on the host, and its cache eviction would otherwise clean up
the working directory), so pass absolute paths for `--script` and `--shots`.

Screenshots are written at the canvas size (800x480); the Pager's screen is the
top-left 480x222.

## Input

| Host key | Pager |
|---|---|
| Up / Down | turn the knob one detent |
| Right | click the knob |
| Left / Esc | hold the knob ~1.2 s (Back) |
| Enter, Backspace, letters | the keyboard's own keys |

Alt/Fn chords and press-and-hold letters (Hold S, Space to lock) are not
simulated.

## What is stubbed

`SimPager.cpp` and the headers here stand in for the Pager's hardware: the
ES8311 codec and I2S (silent), the DRV2605 motor (prints the pattern), the
BHI260AP (lying flat), the charger and gauge (76 %, on battery), no microSD.
The mesh is the web build's demo backend.
