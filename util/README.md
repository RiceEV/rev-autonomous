# Beaglebone Utilities
This directory contains some useful scripts that will help with setting up your Beagle Bone

# List of scripts and how to use them
## usb_internet
This script is used to enable the board to use your computer's network adapter when plugged in over USB. You will likely have to run this script each time you reboot or change network connections so I would recommend adding this script to your PATH (I'll show you how to do that later). 

### Enabling network adapter sharing on Windows
In order to use this script you must first enable sharing on your primary network adapter. On Windows this can be done by going to Control Panel > View network and status and selecting your primary connection (the one giving you internet access on your PC).
[IMAGE HERE]

Next, select properties and then the sharing tab in the window that appears. You will want to select the option that says "Allow other network users to connect..." and deselect the option that says "Allow other network users to control or disable the shared Internet connection". Next, select the device that corresponds to your beagle bone in the drop down menu (my connection is called "Ethernet" but depending on if you are using WiFi or not it may be something like Ethernet2 or 3).
[IMAGE HERE]

Finally, go back to the control panel networking screen and select the network that corresponds to your Beagle Bone. Select properties again and this time scroll down in the Networking tab until you find an item labeled "Internet Protocol Version 4 (TCP/IPv4)". Select this and in the window that appears check the "Obtain an IP address automatically" and "Obtain DNS server address automatically" boxes.
[IMAGE HERE]

Now your Beagle Bone will have access to your computer's network adapter to connect to the internet!

### Using the script
Now we have to actually use the script to enable use of the computer's adapter.
TODO

### Adding to PATH
TODO
