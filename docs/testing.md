# Offline checks

From the firmware root, use a native C/C++ compiler, binutils, gzip, Python 3 and CMake. These commands run without a board.

```sh
tests/run_host_tests.sh
CFLAGS_EXTRA='-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -no-pie' tests/run_host_tests.sh
python3 tools/check_layers.py
python3 tools/check_config_macros.py --before tests/data/app_config_before.h
python3 tools/gen_lcd_zh_font.py --check
SOURCE_DATE_EPOCH=1763700000 LCD_SCREENSHOTS=/tmp/lcdshots/current tests/run_host_tests.sh test_lcd_ui test_lcd_ui_43
```

The protocol differential test compares against a fixed compressed oracle generated with the pre-refactor driver. That driver is outside the public repository; this checkout alone cannot regenerate the reference. It exercises 100,000 input sets across four motor ranges. The committed fixture also runs in a source archive.

For control changes run the folded-plant sweep and the seven-axis contact, drag and release comparisons. The seven-axis contact sweep is optional. It requires a separately supplied simulator and its Python dependencies; it prints SKIP when the simulator is absent. The shoulder-push host regression contains its own plant equations. The mode transition replay checks a fixed reference length and SHA-256; it runs offline with the files in this release.

```sh
python3 tools/test_admittance_coupled.py
python3 tools/test_admittance_ff_contact.py --sim-dir /path/to/optional-simulator --out build/contact
python3 tools/test_legacy_control_trace.py
```
