# ESP32 upgrade to the ESP8266 2WD robot

 Plug replaceable upgrade of the ESP8266 2WD robot using an ESP32 and a custom adaptor PCB. By 'plug replaceable' it means that the ESP8266 microcontroller module can be simply removed from the motor controller and replaced by a ESP32 mounted on a custom PCB that acts as an adaptor between the motor controller connectors, that are designed for a ESP8266, and the 38-pin connector of the ESP32 module.

The 3D printed components for the robot are exactly the same as the original ESP8266 project and the designs can be downloaded from [here](https://www.printables.com/model/67808-esp8266-2wd-robot-components). Full details of the upgrade project are documented [here](https://onlinedevices.org.uk/ESP32_upgrade_to_ESP8266_2WD_Robot) and the images below show the overall build in various stages:

<img src="images\ESP32adapt_PCB03_20210310_111946210_900w.jpg" width="207" height="180"> &nbsp; &nbsp; <img src="images\ESP32adapt_PCB03_20210310_112217977.PORTRAIT_900w.jpg" width="230" height="180">  &nbsp; &nbsp; <img src="images\ESP32adapt_PCB03_20210310_112425991_900w.jpg" width="162" height="180">

Details of the custom PCB are shown in the images below, and the KiCAD design as gerber files can be downloaded from the 'PCB_design_files' folder.

<img src="images\ESP32adapt_PCB03_annotated_900w.jpg" width="276" height="250"> &nbsp; &nbsp; <img src="images\ESP32adapt_PCB03_front_600w.jpg" width="162" height="250">  &nbsp; &nbsp; <img src="images\ESP32adapt_PCB03_back2_600w.jpg" width="166" height="250">

The ESP32 code, downloadable from the 'ESP32_code' folder, was developed on the Arduino IDE and like the original ESP8266 version it uses the 3 slide switches to 'set' a particular operational mode and uses a set of key parameters held in SPIFFS files. These key parameters are all 'set' in the data folder but the folder in the repository is called  'data-public-version', where the details of the optional SSID's and their passwords are just dummy values. These text files therefore need to be updated to the required usage values and the folder renamed as just 'data'.
