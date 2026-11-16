# ERO PAD OS v2.24 — Complete Crosstalk Update

## Included files
- `ERO_PAD_OS_v2_24_COMPLETE_WEB_AND_OLED_UPDATE.ino` — complete ESP32-S3 firmware source with the updated web page embedded.
- `ERO_PAD_OS_v2_24_index.html` — standalone copy of the same updated HTML page.
- `ERO_PAD_OS_v2_24_README.md` — this guide.

## Crosstalk update
- OLED: Advanced Tune → select pad → Crosstalk Level. Encoder steps through 50%, 60%, 70%, 80%; press encoder to save.
- Web UI: Settings → Crosstalk Matrix → Roland-Style Crosstalk Cancel Level. Select a pad and level, then apply and save.
- The web control updates only the selected pad's matrix row. Other pad rows are not changed.
- Existing detailed matrix editing and Quick Set controls remain available.
- Existing API routes are retained.

## Install
1. Back up your current working firmware.
2. Open the `.ino` file in Arduino IDE with the same ESP32-S3 board, libraries, and build settings used by your existing project.
3. Compile before uploading. Upload only if compilation succeeds.
4. Connect your phone to ERO PAD Wi-Fi and open `http://192.168.4.1/`.
5. Test OLED navigation, web matrix load/save, pad triggering, SD persistence, audio, and MIDI.

## Important
The ESP32 serves the HTML embedded in the `.ino` file. The separate HTML file is an identical source copy for GitHub version control. Uploading the HTML to GitHub Pages alone does not replace the ESP32's embedded page or automatically route relative API calls to the ESP32.

This package has not been compiled or hardware-tested in this environment.
