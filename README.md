# Firmware-Development-of-aBluetooth-Low-Energy-Applicationfor-Audio-Recording-of-a-Songbird
In this repository the code for an in-vivo audio sensor based on a nordic nrf 52832 SoC gets presented. The sensor is able to send data via BLE to a  central device which is also based on the nrf52832 SoC. The code for the central is also presented in this repository. 
To get a better understanding of the hardware and firmware implemented I recommend taking a look at the thesis which will also be uploaded to the repo.
Additionally you can find some python code which converts the over bluetooth received pcm-data into a wav file which is playable on almost every device.
Get the  central code to run with SEGGER embedded studio:
    -download the main.c 
    -make sure you have SDK 17.02 installed on your PC
    -copy the folder ble_app_uart_c 
    -replace the main file with the main file presented in the central folder of this repo
Get the peripheral code to work with SEGGER embedded studio:
    -download the main.c ble_custom.c and ble_custom.h
    -make sure you have SDK 17.02 installed on your PC
    -copy the folder ble_app_uart in examples/ble_peripheral
    -replace the main file with the main file presented in the peripheral folder of this repo
    -copy the ble_custom.c and the ble_custom.h into the ble_app_uart
    -replace the ble_nus.c file in the SEEGGER project explorer with the ble_custom.c file
    -in the nrf drivers folder include the nrfx_pdm.c file
    -enable nrfx_pdm via the CMSIS configuration wizzard 
    -build the solution 
    -if you get an error locate the nrfx_glue file and uncomment line 57 (dont ask why! it works now be happy)
