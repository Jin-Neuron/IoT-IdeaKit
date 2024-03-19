import machine
from machine import Pin, SPI, reset
import json
import time
import ssd1306
import img_lib
import framebuf
import network
import ubinascii
from umqtt.simple import MQTTClient

#spi設定
spi = SPI( 1,baudrate = 100000, sck = Pin(10), mosi = Pin(11))

#mqtt設定
iot_core_endpoint = 'endpoint'
topic = 'topic'

#Wi-Fi設定
ssid = 'SSID'
password = 'PW'

#pin設定
oled_cs = Pin(7,Pin.OUT)
dc = Pin(6,Pin.OUT)
rst = Pin(5,Pin.OUT)
enable = Pin(14, Pin.OUT)
left = Pin(15, Pin.OUT)
right = Pin(17, Pin.OUT)
red = Pin(4, Pin.OUT)
blue = Pin(18, Pin.OUT)

display = ssd1306.SSD1306_SPI(128, 64, spi, dc, rst, oled_cs)
fb = framebuf.FrameBuffer(img_lib.techring, 74, 64, framebuf.MONO_HLSB)
state = ""

#起動アニメーションの表示
display.fill(0)
display.blit(fb, int((128-74) / 2), 0)
display.show()

#Wi-Fiに接続
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

#接続を待機
max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

#接続できなかった場合はエラーを表示
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
#接続できた場合はIPアドレスを表示
else:
    print('Connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )

keyfile = '/certs/private.der'
with open(keyfile, 'rb') as f:
    key = f.read()
certfile = "/certs/certificate.der"
with open(certfile, 'rb') as f:
    cert = f.read()    
ssl_params = {'key': key, 'cert': cert, 'server_side': False}

def mqtt_subscribe(topic, msg):
    print("on received...")
    global state
    state = json.loads(msg.decode())["state"]
    print("state {}".format(state))

try:
    #mqtt接続
    mqtt_client_id = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
    mqtt = MQTTClient(
        mqtt_client_id,
        iot_core_endpoint,
        port = 8883,
        keepalive = 10000,
        ssl = True,
        ssl_params = ssl_params)

    print("Connecting to AWS IoT...")
    mqtt.connect()
    print("Connected.")

    mqtt.set_callback(mqtt_subscribe)
    mqtt.subscribe(topic)

except:
    print("Unable to connect to MQTT.")
    
f = open("./index.html", 'r')
html = f.read()
f.close()

while True:
    try:
        #mqtt受信処理
        mqtt.check_msg()

        if state == "TurnRight":
            display.fill(0)
            display.text("TurnRight",0,0,1)
            display.show()
            #モータードライバー出力
            enable.value(1)
            left.value(1)
            right.value(0)
            #LED出力
            blue.value(1)
            red.value(0)
        elif state == "TurnLeft":
            display.fill(0)
            display.text("TurnLeft",0,0,1)
            display.show()
            #モータードライバー出力
            enable.value(1)
            left.value(0)
            right.value(1)
            #LED出力
            blue.value(1)
            red.value(0)
        elif state == "Stop":
            display.fill(0)
            display.blit(fb, int((128-74) / 2), 0)
            display.show()
            #モータードライバー出力
            enable.value(0)
            #LED出力
            blue.value(0)
            red.value(1)

    except Exception as e:
        print(e)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        reset()

    time.sleep_ms(500)