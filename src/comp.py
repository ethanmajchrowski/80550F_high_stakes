# competition code :)
from vex import *

#region variables

# Brain should be defined by default
brain=Brain()

#endregion variables

def auton():
    pass

def driver():
    pass

if brain.sdcard.is_inserted():
    comp = Competition(driver, auton)
else:
    while True:
        brain.screen.set_cursor(0, 0)
        # warn the user that there is no SD card inserted
        brain.screen.set_fill_color(Color(255, 0, 0))
        brain.screen.draw_rectangle(0, 0, 480, 240)
        for i in range(4):
            brain.screen.set_font(FontType.PROP60)
            brain.screen.print("     NO SD CARD")
            brain.screen.new_line()
        
        wait(400)

        brain.screen.clear_screen()

        wait(400)