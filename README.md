# NineLight ##

NineLight is a media streaming setup for the team 900 robot cameras. We use MediaMTX (https://github.com/bluenviron/mediamtx) to stream data from a Raspberry Pi camera to a web server. 

## Installation ##
First, download the MediaMTX binary (https://github.com/bluenviron/mediamtx/releases). 
### Camera Setup ###
We have to disable the legacy Raspberry Pi camera stack. To disable the stack, type `sudo raspi-config`, then go to `Interfacing options`, `enable/disable legacy camera support`, choose `no`. Reboot the system.
