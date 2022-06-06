# greenseeker packet message
Header header

# rgba values to show
LEDRGBA rgba

# a short hand to use for common alert colors
# this doesn't define alert colours
# it just says what kind of alert the message represents
# this can be used with LEDRGBA message constants
uint8 alert
# alert types
uint8 NONE = 0
uint8 IDLE = 1
uint8 SUCCESS = 2
uint8 WARNING = 3
uint8 ERROR = 4

# light on pulse duration in milliseconds
uint16 duration

# whether to flash the light during the pulse duration
# and how long the is the flash length
uint16 flash

# the notification's lifetime validity
# once this has passed the notification is not valid
duration validity
