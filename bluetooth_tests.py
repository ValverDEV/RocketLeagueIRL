import serial
# Serial setup to use in Arduino
evenement=""
print("Start")
port="/dev/rfcomm0"
bluetooth=serial.Serial(port, 9600) # Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick

def send_error(error):

    """Send error value through bluetooth

    Parameters
    ----------
    error: number
        error in car position and expected trajectory
    """

    # Convert number to string and then encode to ascii
    error_bytes = str(error).encode('ascii')
    bluetooth.write(error_bytes)
    return

while True:
    err = input()
    err = int(err)
    send_error(err)