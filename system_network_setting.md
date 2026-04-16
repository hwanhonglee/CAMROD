# Set Network
# Create File
sudo nano /etc/sysctl.d/10-cyclone-max.conf
# Add cmd line
# Increase the maximum receive buffer size for network packets
net.core.rmem_max=2147483647  # 2 GiB, default is 208 KiB

# IP fragmentation settings
net.ipv4.ipfrag_time=3  # in seconds, default is 30 s
net.ipv4.ipfrag_high_thresh=134217728  # 128 MiB, default is 256 KiB

#### check
user@pc$ sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh
net.core.rmem_max = 2147483647
net.ipv4.ipfrag_time = 3
net.ipv4.ipfrag_high_thresh = 134217728

################################################
# if use cyclonedds? you need to install 
# Choose your ROS distribution
rosdistro=humble  # or jazzy

# RMW implementation (see ./defaults/main.yaml for the current default)
rmw_implementation=rmw_cyclonedds_cpp 

sudo apt update
sudo apt install ros-${rosdistro}-${rmw_implementation//_/-}

# (Optional) You set the default RMW implementation in the ~/.bashrc file.
echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc

# AND, Enable multicast for lo
sudo ip link set lo multicast on

###########################################

# Set DDS  
# Create File 
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <!-- <NetworkInterface autodetermine="false" name="lo" priority="default" multicast="default" /> -->
        <!-- <NetworkInterface name="eth0" multicast="default" /> -->
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="10MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
# Add cmd line (in ~/.bashrc)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export CYCLONEDDS_URI=file:///absolute/path/to/cyclonedds.xml
# Replace `/absolute/path/to/cyclonedds.xml` with the actual path to the file.
# Example: export CYCLONEDDS_URI=file:///home/user/cyclonedds.xml



###########################3
