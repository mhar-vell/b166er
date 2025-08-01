# Orientation for b1-66er Robot with ROS on macOS
This document provides instructions for setting up the P3AT robot with ROS on macOS, including
installation of the necessary packages, configuration of the robot, and troubleshooting steps.

Before ```catkin_make``` the following command must be run to fix the Aria library path:

Check the current library path of the RosAria node:
```shell
# Check the library dependencies of the RosAria node
otool -L ~/p3at_ws/devel/lib/rosaria/RosAria
```
Probably it will show something like this:
```bash
# Output of otool -L command
/Users/marcoreis/p3at_ws/devel/lib/rosaria/RosAria:
        @rpath/libtf.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/libroscpp.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/librosconsole.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/libdynamic_reconfigure_config_init_mutex.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/libroscpp_serialization.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/librostime.dylib (compatibility version 0.0.0, current version 0.0.0)
        lib/libAria.dylib (compatibility version 0.0.0, current version 0.0.0)
        @rpath/libc++.1.dylib (compatibility version 1.0.0, current version 1.0.0)
        /usr/lib/libSystem.B.dylib (compatibility version 1.0.0, current version 1351.0.0)
```
If the output shows `lib/libAria.dylib`, it means that the RosAria node is looking for the Aria library in a relative path, which may not work correctly on macOS.
To fix this issue, you need to change the path of the Aria library to an absolute path. The Aria library is typically installed in `/usr/local/Aria`, so you need to ensure that the library is located there. If you have installed the Aria library in a different location, you will need to adjust the commands accordingly.
If you have the Aria library installed in `/usr/local/Aria`, you can create a symbolic link to the expected location using the following command:
```bash
# Create a symbolic link to the Aria library in the expected location
sudo ln -s /usr/local/Aria/lib/libAria.dylib /usr/local/lib/libAria.dylib
```
This command creates a symbolic link from the Aria library in `/usr/local/Aria/lib` to the expected location in `/usr/local/lib`. This allows the RosAria node to find the Aria library without needing to modify the binary directly.
Make sure to install it in the `/usr/local/Aria` directory or adjust the paths accordingly in the commands below.
If you have the Aria library installed in a different location, you can create a symbolic link
to the expected location using the following command:
```bash
# Create a symbolic link to the Aria library in the expected location
sudo ln -s /path/to/your/Aria/lib/libAria.dylib /usr/local/Aria/lib/libAria.dylib
```
If necessary, you can create the folder structure for the Aria library using the following command:
```bash
# Create the Aria library directory structure if it does not exist
sudo mkdir -p /usr/local/Aria/lib
```
This command creates the `/usr/local/Aria/lib` directory if it does not already exist. 

Make sure to replace `/path/to/your/Aria/lib/libAria.dylib` with the actual path where the Aria library is installed on your system. This command creates a symbolic link from the Aria library in your specified location to the expected location in `/usr/local/Aria/lib/libAria.dylib`. 
This allows the RosAria node to find the Aria library without needing to modify the binary directly.
After creating the symbolic link, you need to update the RosAria node to use the absolute path of the Aria library. 
You can do this by using the `install_name_tool` command to change the library path in the RosAria node
binary.
First, check the current library dependencies of the RosAria node:
```bash# Check the library dependencies of the RosAria node
otool -L ~/p3at_ws/devel/lib/rosaria/RosAria
```
If the output shows `lib/libAria.dylib`, it means that the RosAria node is looking for the Aria library in a relative path, which may not work correctly on macOS. 
To fix this, you need to change the path of the Aria library to an absolute path. 
Assuming you have installed the Aria library in `/usr/local/Aria`, you can change the path of the Aria library in the RosAria node using the `install_name_tool` command:
```bash
# Change the Aria library path in the RosAria node
install_name_tool -change lib/libAria.dylib /usr/local/Aria/lib/libAria.dylib ~/p3at_ws/devel/lib/rosaria/RosAria
```
This command updates the RosAria node binary to use the absolute path of the Aria library, ensuring that it can find the library correctly when running.

> **Note:** If you have installed the Aria library in a different location, you can change the path in the command above to match your installation directory. 
For example, if you have the Aria library installed in `/opt/Aria`, you would run:          
```bash
# Change the Aria library path in the RosAria node for a different installation directory
install_name_tool -change lib/libAria.dylib /opt/Aria/lib/libAria.dylib ~/p3at_ws/devel/lib/rosaria/RosAria
```

After that, you need to re-sign the binary to avoid issues with the code signature: 
```bash
# Re-sign the RosAria node binary
codesign --force --sign - ~/p3at_ws/devel/lib/rosaria/RosAria
```
Finally, you need to set the `DYLD_LIBRARY_PATH` environment variable to include the Aria library path:
```bash
# Set the DYLD_LIBRARY_PATH to include the Aria library path
export DYLD_LIBRARY_PATH=/usr/local/Aria/lib:$DYLD_LIBRARY_PATH 
```

After running the above commands, you can proceed with building your workspace using:
```bash
# Build the workspace with catkin_make
cd ~/p3at_ws
source devel/setup.bash
# Run catkin_make to build the workspace    
catkin_make
``` 
Make sure to add the `DYLD_LIBRARY_PATH` export command to your shell configuration file (e.g., `.bashrc`, `.zshrc`) to ensure it is set every time you open a new terminal session.    
If you encounter any issues with the Aria library not being found, you can also try running the following command to update the library cache:
```bash
sudo update_dyld_shared_cache
```
This command may require administrative privileges, so you might need to enter your password.
If you are using a different version of the Aria library or have it installed in a different location, make sure to adjust the paths accordingly in the commands above.

If errors persist, ensure that the Aria library is correctly installed and that the paths specified in the commands match your system's configuration.
Aria library is typically installed in `/usr/local/Aria`, but if you have it installed elsewhere, you will need to adjust the paths accordingly.
If you have Aria library in different location, you can change the path in the commands above to match your installation directory. Rosaria node requires the Aria library to be correctly linked for proper functionality. 
Rosaria expects the Aria library to be located at `/usr/local/Aria/lib/libAria.dylib`, so if you have it installed in a different location, you will need to adjust the commands accordingly. If you not have the Aria library installed, you can create a symbolic link to the correct location using the following command:
```bash
sudo ln -s /usr/local/lib/libAria.dylib /usr/local/Aria/lib/libAria.dylib
```
This command creates a symbolic link from the Aria library in `/usr/local/lib` to the expected location in `/usr/local/Aria/lib`.
Make sure to run the above commands in a terminal with the appropriate permissions. If you encounter any issues with permissions, you may need to use `sudo` to run the commands as an administrator.
To ensure that the `DYLD_LIBRARY_PATH` is set correctly every time you open a new terminal session, you can add the export command to your shell configuration file. For example, if youare using `bash`, you can add the following line to your `~/.bashrc` or `~/.bash_profile` file:
```bash
export DYLD_LIBRARY_PATH=/usr/local/Aria/lib:$DYLD_LIBRARY_PATH
```
After adding the line, save the file and run the following command to apply the changes:
```bash
source ~/.bashrc
``````bash
# or
source ~/.bash_profile
```
This will ensure that the `DYLD_LIBRARY_PATH` is set correctly in all new terminal sessions.
If you are using `bash`, you can add the export command to your `~/.bashrc` or `~/.bash_profile` file to make the change persistent across terminal sessions. Here's how you can do it:
```bash
echo 'export DYLD_LIBRARY_PATH=/usr/local/Aria/lib:$DYLD_LIBRARY_PATH'
>> ~/.bashrc
```

If you are using a different shell (e.g., zsh), make sure to adjust the commands accordingly, as the syntax for exporting environment variables may vary slightly.
