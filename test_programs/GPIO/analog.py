# Libraries
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Software SPI
CLK = 18
MISO = 23
MOSI = 24
CS = 25
mcp = Adafruit_MCP3008.MCP3008(clk=CLK,cs=CS,miso=MISO,mosi=MOSI)

# GPIO Mode (BOARD / BCM)
#GPIO.setmode(GPIO.BCM)

print("Reading values")

 
if __name__ == '__main__':
    try:
        
        while True:

            values = [0]*8
            for i in range(8):
                values[i] = mcp.read_adc(i)

            print(' | {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
            time.sleep(0.5)
            
            

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        #GPIO.cleanup()
