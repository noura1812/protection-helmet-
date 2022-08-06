#firebase & sensors imports
import RPi.GPIO as GPIO
import board
import busio as io
import time
import Adafruit_DHT
import pyrebase
import sys
from w1thermsensor import W1ThermSensor
import os
from datetime import datetime
#prints the date and time for each send
print (sys.getdefaultencoding())

#initialization for the pins ,time,sensors

#date&time
MAX_FILE_SIZE_B = 1024
#sensors
#DHT22
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 17
humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
#dangerous casae 
over_heated=0
danger_data=0
high_temp=0
#mcp3008
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8
#mq5
smokesensor_dpin = 19
smokesensor_apin = 1
#mq2
mq2_dpin = 26
mq2_apin = 2
#ds18b20
sensor=W1ThermSensor()
    
#flatter
i2c = io.I2C(board.SCL, board.SDA, frequency=100000)

config = {
  "apiKey": "FDdYIWn6nMEniz7FjJDBdXj98u4pdziNLgOganbe",
  "authDomain": "minners-1e0c7.firebaseapp.com",
  "databaseURL": "https://minners-1e0c7-default-rtdb.firebaseio.com",
  "storageBucket": "project-id.appspot.com"
}
firebase = pyrebase.initialize_app(config)
db = firebase.database()

#/////////////////////////////////
#ai imports & initialization
import numpy as np
import os
import tensorflow as tf    
from keras.applications.mobilenet import preprocess_input
from tensorflow.keras.preprocessing.image import load_img, img_to_array
import cv2

class_names = ['coal', 'cracks']
class_names2 = ['dangerous', 'normal','safe']
trained_model = tf.keras.models.load_model('/home/pi/Shot/Pro_Model.h5')  # Load Model
cracks_model = tf.keras.models.load_model('/home/pi/Shot/Cracks_Model.h5')  # Load Model
dataset = '/home/pi/Shot/seg_pred2'  # Unlabeled datasets
files = os.listdir(dataset)

camera = cv2.VideoCapture(0)
dangerr=0
normall=0
safee=0

#date & time function
def log_current_time(log_path):
    now = datetime.now()
    timestamp = now.strftime('%Y-%m-%d_%H-%M-%S')               # ex: '2021-10-28_16-59-59'
    write_mode = 'a'                                            # assume we're going to append to the output file

    if os.path.isfile(log_path):                                # if the output file already exists
        size = os.path.getsize(log_path)                        # get file size in bytes
        if size >= MAX_FILE_SIZE_B:                             # if the file is too large, overwrite it entirely
            write_mode = 'w'

    with open(log_path, mode=write_mode) as output_file:
        output_file.write(timestamp + '\n')
        print(timestamp)


#function for setting initial values for the pins to synchronize 
def init():
         GPIO.setwarnings(False)
         GPIO.cleanup()			#clean up at the end of your script
         GPIO.setmode(GPIO.BCM)		#to specify whilch pin numbering system
         # set up the SPI interface pins
         GPIO.setup(SPIMOSI, GPIO.OUT)
         GPIO.setup(SPIMISO, GPIO.IN)
         GPIO.setup(SPICLK, GPIO.OUT)
         GPIO.setup(SPICS, GPIO.OUT)
         GPIO.setup(smokesensor_dpin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
         GPIO.setup(mq2_dpin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#read SPI data from MCP3008(or MCP3204) chip,8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
        if ((adcnum > 7) or (adcnum < 0)):
                return -1
        GPIO.output(cspin, True)	

        GPIO.output(clockpin, False)  # start clock low
        GPIO.output(cspin, False)     # bring CS low

        commandout = adcnum
        commandout |= 0x18  # start bit + single-ended bit
        commandout <<= 3    # we only need to send 5 bits here
        for i in range(5):
                if (commandout & 0x80):
                        GPIO.output(mosipin, True)
                else:
                        GPIO.output(mosipin, False)
                commandout <<= 1
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)

        adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
        for i in range(12):
                GPIO.output(clockpin, True)
                GPIO.output(clockpin, False)
                adcout <<= 1
                if (GPIO.input(misopin)):
                        adcout |= 0x1

        GPIO.output(cspin, True)
        
        adcout >>= 1       # first bit is 'null' so drop it
        return adcout
#call the synchronization for thr=e pins
init()
#the main code for taking pictures
while True:
    #calling and setting time & date
    log_path = '/home/pi/Code/test/output.txt'
    log_current_time(log_path)
    for i in range(1):
        #taking a picture
        return_value, image = camera.read()
        #wait 1sec
        time.sleep(3)
        #set a path for saving the image
        cv2.imwrite('/home/pi/Shot/opencv1.png', image)
        path='/home/pi/Shot/opencv1.png'
        #resetting image layout and convert it into an array
        x = load_img(path = path, target_size = (224, 224))
        x = img_to_array(x)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        y = trained_model.predict(x)
        # Confidence
        for i in np.argsort(y[0])[::-1]:
            coall=y[0][0]*100
            crackss=y[0][1]*100
            
            if y[0][1]*100 >= 1.00000 and class_names[i] == 'cracks':
                y2 = cracks_model.predict(x)
                for j in np.argsort(y2[0])[::-1]:
                
                    dangerr=y2[0][0]*100
                    normall=y2[0][1]*100
                    safee=y2[0][2]*100
                    if(round(dangerr)>=50):
                        danger_data=1
                    else:
                        danger_data=0
                    
            #chick if the there are dangrous cracks
            
            #send the data of the images to the firebase
            data = {"isCoal":str(round(coall,2))+"  %              ",
                "isCracks":str(round(crackss,2))+" %              ",
                "dangerious_ratio":str(round(dangerr,2))+"  %          ",
                "normal_ratio":str(round(normall,2))+" %           ",
                "safe_ratio":str(round(safee,2))+" %               ",
                   }
            db.child("ratioF").child("2-push").set(data)
            #del(camera)
        #returning values from sensors
        smokelevel=readadc(smokesensor_apin, SPICLK, SPIMOSI, SPIMISO, SPICS)
        COlevel=readadc(mq2_apin, SPICLK, SPIMOSI, SPIMISO, SPICS)
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
        temp_human=sensor.get_temperature()

        #check if overheated
        if (humidity>=37.5):
                over_heated=1
        else:
                over_heated=0
        if (temperature>50):
                high_temp=1
        else:
                high_temp=0
        #set the configration of the sensor reads
        HumanTemp =str(round(temp_human,1))+" °C          "
        ambient_temperature =str(round( temperature,1))+" °C          "
        Humidityy =str( round(humidity,2))+" %          "
        smoke=str(round((smokelevel/1024.)*5,2))+" ppm          "
        Co = str(round((COlevel/1024.)*3.3,2))+" ppb          "
        #send the data of the sensors to the firebase
        data = {
        "human Temp": HumanTemp,
        "ambient Temp": ambient_temperature,
        "Humidity": Humidityy,
        "Methane level": smoke,
        "Co Level": Co,
        }
        db.child("Records").child("1-set").set(data)
        #send dangerous data to firebase
        
        data = {
                "worker overheated":over_heated,
                   }
        db.child("alert").child("sensors1").set(data)
        data = {
              
                "mine overheated":high_temp,
                   }
        db.child("alert").child("sensors2").set(data)
        data = {
                "danger":danger_data,
                   }
        db.child("alert").child("module").set(data)

        time.sleep(2)
       



