# Udacity_Robotics_Software_Engineer_nanodegree

## Summary
This course aims to educate more about localization and navigation principles pertaining to autonomous robots, 
using a commonly used platform called Robot Operating System (ROS). It covers topics ranging from Gazebo simulation, 
communcation between ROS nodes, localization using Extended Kalman Filter or Particle Filter, Simultaneous 
Localization & Mapping (SLAM) and A* Path Planning. 

All of these techniques culminates into a final project where a home service robot capable of localization and navigation 
is developed. Instructions for compiling each project is available on each project's directory.

**Note**: These projects are developed in Ubuntu 16.04, ROS Kinetic and Gazebo 7.16.1. Please clone the master branch for 
latest version of all projects. 

## Gazebo World Segment
Gazebo is a useful simulation tool that can be used with ROS to render robots in a simulated environment. 
It comes with a model and world editor, along with presets models, that can allow for quick prototyping of a physical environment.

The main principles taught in this segment are:

1. Using model editor tool to render a robot with specified links & joints
2. Using World editor tool to render an environment (e.g. a house)
3. Running plugins during the launch of the Gazebo platform

The output of the word created in gazebo is shown bellow, where a office was buided using models in the gazebo library, some 
utilities were added from the library into the office and two robots were build in Gazebo, using the model capabilities of the 
that program.

![1-Gazebo_word_program_running](https://github.com/Photon-einstein/Udacity_Robotics_Software_Engineer_nanodegree/assets/31144077/da5a7709-314e-4232-a2c4-efb8d68067c3)

<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find my_robot)/urdf/my_robot.gazebo" />
 
  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <!-- Scaling factor -->
  <xacro:property name="scale" value="1.5" />

  <link name='chassis'>
    <pose>0 0 0.15 0 0 0</pose>

    <!-- Increase mass accordingly -->
    <inertial>
      <mass value="15.0 * scale"/>
      <origin xyz="0.0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="0.1 * scale" ixy="0" ixz="0"
          iyy="0.1 * scale" iyz="0"
          izz="0.1 * scale"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <!-- Scale size -->
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- Scale size -->
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>

    <!-- Use previous wheels color -->
    <gazebo reference="chassis">
      <material>Gazebo/Green</material>
    </gazebo>

    <!-- Apply scaling to caster links -->
    <xacro:property name="caster_scale" value="1.5" />

    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
      <!-- Scale size -->
      <geometry>
        <sphere radius="0.0499 * caster_scale"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy="0 0 0"/>
      <!-- Scale size -->
      <geometry>
        <sphere radius="0.05 * caster_scale"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
      <!-- Scale size -->
      <geometry>
        <sphere radius="0.0499 * caster_scale"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy="0 0 0"/>
      <!-- Scale size -->
      <geometry>
        <sphere radius="0.05 * caster_scale"/>
      </geometry>
    </visual>

  </link>

  <link name='left_wheel'>
    <pose>0 0 0 0 0 0</pose>

    <inertial>
      <mass value="5.0 * scale"/>
      <origin xyz="0.0 0 0" rpy="0 1.5707 1.5707"/>
      <inertia
          ixx="0.1 * scale" ixy="0" ixz="0"
          iyy="0.1 * scale" iyz="0"
          izz="0.1 * scale"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/> 
      <!-- Scale size -->
      <geometry>
        <cylinder radius="0.1 * scale" length="0.05 * scale"/>
      </geometry>
    </collision>

    <visual name='left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <!-- Scale size -->
      <geometry>
        <cylinder radius="0.1 * scale" length="0.05 * scale"/>
      </geometry>
    </visual>

    <!-- Use previous chassis color -->
    <gazebo reference="left_wheel">
      <material>Gazebo/Blue</material>
    </gazebo>
  </link>

  <link name='right_wheel'>
    <pose>0 0 0 0 0 0</pose>

    <inertial>
      <mass value="5.0 * scale"/>
      <origin xyz="0.0 0 0" rpy="0 1.5707 1.5707"/>
      <inertia
          ixx="0.1 * scale" ixy="0" ixz="0"
          iyy="0.1 * scale" iyz="0"
          izz="0.1 * scale"
      />  
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/> 
      <!-- Scale size -->
      <geometry>
        <cylinder radius="0.1 * scale" length="0.05 * scale"/>
      </geometry>
    </collision>

    <visual name='right_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <!-- Scale size -->
      <geometry>
        <cylinder radius="0.1 * scale" length="0.05 * scale"/>
      </geometry>
    </visual>

    <!-- Use previous chassis color -->
    <gazebo reference="right_wheel">
      <material>Gazebo/Blue</material>
    </gazebo>

  </link>

  <joint type="continuous" name="left_wheel_hinge">
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <child link="left_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <joint type="continuous" name="right_wheel_hinge">
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <child link="right_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

 <link name='camera'>
    <pose>0 0 0 0 0 0</pose>

    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0 0" rpy="0.0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0.0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <visual name='camera_visual'>
      <origin xyz="0.0 0 0" rpy="0.0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint type="fixed" name="camera_joint">
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <child link="camera"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

 <link name='hokuyo'>
    <pose>0 0 0 0 0 0</pose>

    <inertial>
      <mass value="1e-5"/>
      <origin xyz="0.0 0 0" rpy="0.0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0.0 0 0"/>
      <geometry>
	<box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>

    <visual name='hokuyo_visual'>
      <origin xyz="0.0 0 0" rpy="0.0 0 0"/>
      <geometry>
	<mesh filename="package://my_robot/meshes/hokuyo.dae"/>
      </geometry>
    </visual>
  </link>

  <joint type="fixed" name="hokuyo_joint">
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <child link="hokuyo"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>

  <gazebo reference="left_wheel">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="chassis">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="camera">
    <material>Gazebo/Red</material>
  </gazebo>

</robot>
