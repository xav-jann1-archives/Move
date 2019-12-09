# Copy your current .gazebo folder (if exist and not a symbolic link)
cp -R ~/.gazebo ~/.gazebo.sav2

# Copy the current .gazebo to the /tmp
cp -R /sync/Robotic/gazebo/.gazebo /tmp/.

# Remove your previous .gazebo
rm -R ~/.gazebo

# Create a symbolic link to /tmp/.gazebo
ln -s /tmp/.gazebo ~/.gazebo

# Create link to model
ln -s ~/Bureau/Move/src/scara_cpe_gazebo/models/kinect_ros/ ~/.gazebo/models/