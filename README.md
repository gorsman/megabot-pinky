Installation instructions
-------------------------
In order to avoid configuration problems with CMake please install everything
as outlined below (particularly mind the directories as it might be a problem
for CMake to locate this software).

1. Arduino SDK
   Download Arduino SDK from the official web-site:
     http://arduino.cc/en/Main/Software
   ... and unpack it to /usr/share/

   For example:
     /usr/share/arduino-1.0.4/


2. Eclipse
   Download "Eclipse IDE for C/C++ Developers" from the official web-site:
     http://www.eclipse.org/downloads/
   ... ant unpack it to /usr/share/

   So that the location of your Eclipse installation is:
     /usr/share/eclipse/

   Add Eclipse to $PATH. For example create /etc/profile.d/eclipse.sh
   Put the following content into the file:
     #!/bin/bash
     ECLIPSE_DIR=/usr/share/eclipse
     PATH=$PATH:$ECLIPSE_DIR
     export PATH
   Mark the file as executable: sudo chmod a+rx /etc/profile.d/eclipse.sh
