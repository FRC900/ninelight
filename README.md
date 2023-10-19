# NineLight ##

NineLight is a media streaming setup for the team 900 robot cameras. We use MediaMTX (https://github.com/bluenviron/mediamtx) to stream data from a Raspberry Pi camera to a web server. 

## Installation ##
First, download the MediaMTX binary (https://github.com/bluenviron/mediamtx/releases). 
### Camera Setup ###
We have to disable the legacy Raspberry Pi camera stack. To disable the stack, type `sudo raspi-config`, then go to `Interfacing options`, `enable/disable legacy camera support`, choose `no`. Reboot the system. Then install the packages `libcamera0` and `libfreetype6`. Download the server executable and download the mediamtx.yml file from this github page. To start the server, just run `./mediamtx`

### The Rest ###
What do we use as the client? How do we connect to the camera
How would we specify specific ports for the camera to use, and how do we make the cliet use them?
I remember we had to disable IPv6 for some reason, why and how did we do that?
