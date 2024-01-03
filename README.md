# NineLight ##

NineLight is a media streaming setup for the Team 900 robot cameras. We use MediaMTX (https://github.com/bluenviron/mediamtx) to stream data from a Raspberry Pi camera to a web server. 

## Installation ##
First, download the MediaMTX binary (https://github.com/bluenviron/mediamtx/releases). 
### Camera Setup ###

#### Building rpiboot ####
Install or build the `rpiboot` utility by following the instructions at https://github.com/raspberrypi/usbboot#building. Don't worry about messages like "Cannot open file", as long as it says "second stage boot server done" the build was succesful. 
#### Flashing the OS Image ####
Run `rpi-imager` (https://github.com/raspberrypi/rpi-imager) with the camera plugged in. Select the 64-bit light version. Make sure to enable SSH with username `ubuntu` and password `ubuntu`, set the timezone to `us/new york`, and set the hostname as `ninelight`. 
#### Configuring SSH ####
Disable IPv6 on the computer, plug the camera into the robot, connect to the robot's wifi, and SSH into the robot with username `ubuntu@ninelight.local` and password `ubuntu`. Type `sudo nano /etc/ssh/sshd_config`, scroll to the line that says `port 22`, and change it to say `port 5801`. 
#### Configuring the Overlay ####
On the local computer, reconnect to the actual wifi and run `sudo wget https://datasheets.raspberrypi.org/cmio/dt-blob-cam1.bin -O /boot/dt-blob.bin`. That will download the file to `/boot/dt-blob.bin`, so move it to the home directory with `sudo mv /boot/dt-blob.bin .` in the home directory. Next, connect to the robot wifi and run `sftp ubuntu@ninelight.local`. Then run `put dt-blob.bin` to transfer the file to the camera. Exit the session, and reconnect with `ssh ubuntu@ninelight.local`. Run `sudo mv dt-blob.bin /boot/` and type `y` when it asks to replace the file. Run `sudo nano /boot/config.txt` and add the line `dtoverlay=ov9281`. 


### The Rest ###
Are these all the things we need to do to setup up the Raspberry Pi?
What do we use as the client? How do we connect to the camera
How would we specify specific ports for the camera to use, and how do we make the cliet use them?
I remember we had to disable IPv6 for some reason, why and how did we do that?

### Real Notes ###
- get rpi-boot utility
    - compile this
    - https://docs.kubesail.com/guides/pibox/rpiboot/
    - https://github.com/raspberrypi/usbboot#building
    - dont worry about it saying thins like "cannot open file", as long as it says "second stage boot server done" you're all good
- use rpi-imager utility
    - select 64-bit light
    - run rpi-boot, you the should see something in the storage tab
    - preconfigure ssh enabled (user/pass)
        - ubuntu/ubuntu
    - timezone/locale
        - us/new york
    - set hostname
        - ninelight
- flash with rasbian-64-os lite
- ssh into it
    - disable IPv6
    - connect to robot wifi
    - connect ninelight to robot
    - ssh ubuntu@ninelight.local
    - remember: pwd is also ubuntu
- add '5801' as a special port for ssh
    - nano /etc/ssh/sshd_config
    - port 5801
- download special overlay file
    - set correct overlay in /boot/txt
    - run sudo wget https://datasheets.raspberrypi.org/cmio/dt-blob-cam1.bin -O /boot/dt-blob.bin ON LOCAL COMPUTER
    - that will download it to /boot/dt-blob.bin, move that to your home directory
    - then sftp into the ninelight
    - and put dt-blob.bin 
    - then ssh into ninelight
    - and move dt-blob.bin to /boot/ (say y to replace when it asks)
    -   

    - edit /boot/config.txt and add dtoverlay=ov9281 (or 5647?)
- set static IP address
    - set it to the ip that it currently is
    - 10.9.0.39
    - 10.9.0.1
    - 10.9.0.1
    - sudo nano /etc/dhcpcd.conf

        interface eth0
        static_routers=10.9.0.1
        static domain_name_servers=10.9.0.1
        static ip_address=10.9.0.39/24


- reboot
    - when ssh'ing now, use -p 5801
- download mediamtx server
    - don't need to disable legacy camera, it's not there
- get the yml file
- send both of those ^ files to the ninelight
- add '5802' and '5803' as ports for the camera
- follow [these instructions] (https://github.com/bluenviron/mediamtx#start-on-boot)
- create shortcut to camera website


