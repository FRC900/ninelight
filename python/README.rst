Driver Camera Server
=============

Running
-------

First install the required packages:

clone this: https://github.com/marshallmassengill/aiortc
You may need to install a bunch of pre-reqs for this - check the repo for build instructions.
"pip install ." from the direct
also run "pip install aiohttp"

This will install a hacked up version of the aiortc library that restricts ports to the 5800-5810 range.

Then run the driver camera server with:
"python ninelight.py --host <host IP address> --video-codec video/H264"

You can then browse to the following page with your browser:

http://<host IP address>:1180

Once you click `Start` the server will send video from its webcam to the
browser.

.. warning:: This has only really been tested with Chrome
