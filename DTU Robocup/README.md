# Coco code
This is the code to control the Coco robot, which will perform in the DTU Robocup and potentially destroy all of the humanity.

## Connecting to Coco
The connection is done by SSH. Normally the PuTTY terminal is used to for SSH, but we agreed that PuTTY is Putin, and we don't like that dictator (I hope). So, we will instead use the Remote - SSH in Microsoft Visual Code Studio.

### Setting up the SSH extension
1. Download Visual Studio Code from Microsoft and set it up.
2. Install the extension Remote - SHH from Microsoft.
3. Click on the lower left blue icon icon, which looks like two triangles pointing at each other.
4. Click on "Connect to Host..." and thereafter "+Add New SSH Host..."
5. Write "local@10.197.218.61", hit enter and click on "<filepath>\.ssh\config"

### Open the connection
1. Click on the lower left blue icon icon, which looks like two triangles pointing at each other.
2. Click on "Connect to Host..." and thereafter "10.197.218.61".
3. A new window opens up and if this is the first time, and choose "Linux".
4. Write the code as "grenen".
5. Open the terminal in "Terminal" and then "New Terminal"

## Open the PI camera
The camera is setup while booting up. The PI camera can be accessed on the browser (not Bowser) by going to "10.197.218.61:7123". If that doesn't work, then you should cry in the corner of the room and reflect how you can't do something as simple as accessing a link.