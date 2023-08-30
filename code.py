from machine import I2C, Pin
from time import sleep
from pico_i2c_lcd import I2cLcd
from collections import OrderedDict
from rotary_irq_rp2 import RotaryIRQ  

rtc = machine.RTC()

# buzzer

buzzer = Pin(16, Pin.OUT, value=0)

# buttons

light_button = Pin(5, Pin.IN, Pin.PULL_UP)
time_button = Pin(9, Pin.IN, Pin.PULL_UP)
alarm_button = Pin(13, Pin.IN, Pin.PULL_UP)
stop_alarm_button = Pin(28, Pin.IN, Pin.PULL_UP)

# display

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

I2C_ADDR = i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)

# rotary

SW=Pin(20,Pin.IN,Pin.PULL_UP)  
r = RotaryIRQ(pin_num_dt=19,  
    pin_num_clk=18,  
    min_val=0,  
    reverse=False,
    half_step=True,
    range_mode=RotaryIRQ.RANGE_UNBOUNDED)  
val_old = r.value()

# vars

white_arrow = bytearray([0x10, 0x18, 0x1C, 0x1E, 0x1E, 0x1C, 0x18, 0x10])
lcd.custom_char(0, white_arrow)

global time_clock
time_clock = OrderedDict()

global alarm_clock
alarm_clock = OrderedDict()

global minute_before
minute_before = 0

# funcs

def rotary_pressed():
    if SW.value()==0:  
        return True
        while SW.value()==0:  
            continue
    return False    

def rotary_turn():
    global val_old, val_new
    val_new = r.value()
    if val_old != val_new:  
        inc = val_new - val_old
        val_old = val_new
        return inc
    else:
        return 0

def pressed_button(B):
    if B.value() == 0:
        return True
    else:
        return False
        
def pause_until_button_released(B):
    while B.value() == 0:
        pass

def set_backlight(flag):
    if flag:
        lcd.backlight_off()
        flag = False
    else:
        lcd.backlight_on()
        flag = True
    pause_until_button_released(light_button)
    return flag

def reset_time_clock():
    global time_clock
    time_clock['hours'] = rtc.datetime()[4]
    time_clock['minutes'] = rtc.datetime()[5]
    time_clock['day'] = rtc.datetime()[2]
    time_clock['month'] = rtc.datetime()[1]
    time_clock['year'] = rtc.datetime()[0] - 2000

def reset_alarm_clock():
    global alarm_clock
    alarm_clock['hours'] = 0
    alarm_clock['minutes'] = 0
    alarm_clock['mode'] = 1

def update_rtc():
    global time_clock
    rtc.datetime((time_clock['year'] + 2000, time_clock['month'], time_clock['day'], 0, time_clock['hours'], time_clock['minutes'], 0, 0))

def display_clock(clock,selection=""):
    if clock == 'time':
        c = time_clock.copy()
    if clock == 'alarm':
        c = alarm_clock.copy()
        # define alarm mode
        if c['mode'] == 0:
            c['mode'] = 'Off'
        if c['mode'] == 1:
            c['mode'] = 'On' 
    # put a zero before one-digit numbers
    if c['hours'] < 10:
        c['hours'] = '0'+str(c['hours'])
    if c['minutes'] < 10:
        c['minutes'] = '0'+str(c['minutes'])
    # if a value is selected put the selection arrow character before selected value 
    if selection:
        c[selection] = f'{chr(0)}' + str(c[selection])
    # display clocks in the lcd screen
    if clock == 'time':
        (hours,minutes,day,month,year) = (c['hours'],c['minutes'],c['day'],c['month'],c['year'])
        lcd.putstr(f'{hours}:{minutes} {day}/{month}/{year}')
    if clock == 'alarm':          
        (hours,minutes,mode) = (c['hours'],c['minutes'],c['mode'])
        lcd.putstr(f'\n{hours}:{minutes} {mode}')

def number_loop_correction():
    global time_clock, alarm_clock
    time_clock['hours'] %= 24
    time_clock['minutes'] %= 60
    time_clock['day'] %= 31
    time_clock['month'] %= 12
    alarm_clock['hours'] %= 24
    alarm_clock['minutes'] %= 60
    alarm_clock['mode'] %= 2

def set_clock(clock, move_button):
    global time_clock, alarm_clock
    if clock == 'time':
        c = time_clock
    if clock == 'alarm':
        c = alarm_clock
    for var in c:
        clock_is_set = False
        lcd.clear()
        display_clock(clock,var)
        pause_until_button_released(move_button)
        while True:
            step = rotary_turn()
            if step != 0:
                c[var] = c[var] + step
                number_loop_correction()
                lcd.clear()
                display_clock(clock,var)
                update_rtc()
            if pressed_button(move_button):
                break
            if rotary_pressed():
                clock_is_set = True
                break
        if clock_is_set:
            break
    lcd.clear()
    display_clock('time')
    display_clock('alarm')

def alarm_time():
    if (time_clock['hours'],time_clock['minutes'],rtc.datetime()[6]) == (alarm_clock['hours'],alarm_clock['minutes'],0):
        return True

def fire_alarm():
    s = 0
    while s < 30:
        set_backlight(False)
        buzzer.value(1)
        sleep(0.5)
        set_backlight(True)
        buzzer.value(0)
        sleep(0.5)
        s += 1
        if pressed_button(stop_alarm_button):
            set_backlight(False)
            buzzer.value(0)
            break

def main():
    
    global minute_before
    
    lcd_backlight_state = True
    reset_alarm_clock()

    while True:
        
        # reset
        
        reset_time_clock()
        
        # update display
        
        if time_clock['minutes'] != minute_before:
            lcd.clear()
            display_clock('time')
            display_clock('alarm')
            minute_before = time_clock['minutes']
        
        # lcd turn on/off light
        
        if pressed_button(light_button):
            lcd_backlight_state = set_backlight(lcd_backlight_state)
        
        # set clock
        
        if pressed_button(time_button):
            set_clock('alarm',time_button)
        if pressed_button(alarm_button):
            set_clock('time',alarm_button)
        
        # fire alarm
        
        if alarm_clock['mode'] == 1 and alarm_time():
            fire_alarm()
        
        sleep(0.5)        
                    
        
if __name__ == "__main__":
    main()
