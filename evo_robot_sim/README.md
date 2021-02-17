# Install Basic Dependencies
```
sudo apt install gazebo9 \
                 libgazebo9-dev \
                 ros-melodic-gazebo-ros-pkgs \
                 ros-melodic-gazebo-ros-control \
                 ros-melodic-joint-state-controller \
                 ros-melodic-effort-controllers \
                 ros-melodic-position-controllers \
                 ros-melodic-velocity-controllers
```

# Install Launch File Dependencies
```
sudo apt install -y ros-melodic-tf2-eigen \
                    libann-dev \
                    ros-melodic-map-server \
                    ros-melodic-twist-mux \
                    ros-melodic-move-base \
                    ros-melodic-teb-local-planner \
                    ros-melodic-global-planner \
                    libflann-dev
```

Install evocortex lib:
```
git clone https://github.com/evocortex/evocortex &&
cd evocortex/platform &&
./make_release_amd64.sh &&
cd ../releases &&
sudo dpkg -i libevocortex-0.0.1-amd64.deb
```

Install evocortex catkin packages:
```
git clone https://github.com/evocortex/evo_robot_sim
```
```
git clone https://github.com/evocortex/evo_ros_apps

rm -rf evo_ros_apps/evo_localizer &&
rm -rf evo_ros_apps/evo_tsd_slam
```
```
git clone https://dev.azure.com/evocortex/_git/evo_localizer
```
```
git clone https://dev.azure.com/evocortex/_git/evo_tsd_slam
```

evo_ros_apps will have a faulty package (evo_udp_bridge). Compile using `catkin build -c` to ignore failing packages or delete it using `rm -r`.

# Compile

catkin build evo_robot_sim

# Launch
```
killall gzserver gzclient rosmaster
roslaunch evo_robot_sim gazebo.launch
```

# Cache leeren
Gazebo erkennt oft nicht, dass Dateien sich geändert haben und nutzt den alten Cache. Manuelles leeren hilft:
```
rm -rf /tmp/gazebo*
```

# Projektaufbau
config: 
- controllers: nicht ändern
- global properties: je nach Roboter anpassen

gazebo/models
- elm_ground_plane: ELM-Bodentextur
- example_heightmap: exemplarisches 3D-Modell einer Karte basierend auf einer Heightmap
- heightmap_willowgarage: exemplarische Heightmap-Karte, welche mittels der mit Gazebo mitgelieferten willowMap erstellt wurde

launch: 
- gazebo.launch: Parameter einlesen, Welt laden, Controller starten, Joy starten
- lift.py und control.py: allgemeine Joyknoten, empfangen cmd_vel und cmd_lift
- controllerspezifische Knoten, die cmd_vel und cmd_lift publishen

lib:
- libevoomni: Omnidirektionaler Antrieb in Gazebo, nur für die firmeninterne Nutzung

meshes:
- Roboter-CAD-Daten (Mesh = nur CAD, ganzes Objekt ist in Gazebo ein Model)

nodes:
- odometry: Twist im falschen Frame, muss korrigiert werden (eigener Node vs Gazebo vs. in Marcos Knoten)

scripts:
- Shellskripte zum Publishen von cmd_vel und cmd_lift

sdf:
- quick: funktioniert nicht wegen Mecanum

genutzte XML-Dateiformate:
- XACRO(= XML mit Macros)
- URDF(ROS, ros_control)
- SDF(Gazebo)
