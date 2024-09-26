# config code

from vex import *
import json

# Brain should be defined by default
brain = Brain()

def main():
    pass

if brain.sdcard.is_inserted():
    main()
else:
    # warn the user that there is no SD card inserted
    brain.screen.set_fill_color(Color(255, 0, 0))
    brain.screen.draw_rectangle(0, 0, 480, 240)
    for i in range(4):
        brain.screen.set_font(FontType.PROP60)
        brain.screen.print("     NO SD CARD")
        brain.screen.new_line()