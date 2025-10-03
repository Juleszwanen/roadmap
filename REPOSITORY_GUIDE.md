# Roadmap Repository - Comprehensive Guide

## Table of Contents
1. [Repository Overview](#repository-overview)
2. [Repository Structure](#repository-structure)
3. [Launch File Initialization](#launch-file-initialization)
4. [ROS Node Initialization Flow](#ros-node-initialization-flow)
5. [Core Components Deep Dive](#core-components-deep-dive)
6. [Function Interactions in roadmap.cpp](#function-interactions-in-roadmapcpp)
7. [File Connections and Dependencies](#file-connections-and-dependencies)
8. [Data Flow Through the System](#data-flow-through-the-system)
9. [Message Definitions](#message-definitions)
10. [Configuration System](#configuration-system)

---

## Repository Overview

The **roadmap** repository is a ROS Noetic package that converts waypoint definitions (from files or external sources) into road polylines with proper lane representations. It reads map data, fits splines through waypoints using Clothoid and/or Cubic spline fitting algorithms, and publishes the resulting road network as ROS messages.

**Key Capabilities:**
- Read waypoints from XML, YAML, or OSM files
- Receive waypoints dynamically via ROS topics
- Fit smooth splines (Clothoid + Cubic) through waypoints
- Generate multiple lane types (roads, sidewalks, crosswalks, road markings)
- Publish road polylines and reference paths
- Visualize maps in RViz

---

## Repository Structure

```
roadmap/
├── roadmap/                    # Main package
│   ├── CMakeLists.txt         # Build configuration
│   ├── package.xml            # Package manifest
│   ├── config/
│   │   └── settings.yaml      # Configuration parameters
│   ├── launch/
│   │   ├── roadmap.launch     # Main launch file
│   │   └── roadmap_test.launch # Test launch file
│   ├── include/roadmap/
│   │   ├── roadmap.h          # Main node class header
│   │   ├── configuration.h    # Configuration class
│   │   ├── reader.h           # File reader class
│   │   ├── spline_converter.h # Spline fitting class
│   │   ├── types.h            # Data structure definitions
│   │   └── helpers.h          # Logging macros
│   ├── src/
│   │   ├── roadmap_node.cpp   # ROS node entry point
│   │   ├── roadmap.cpp        # Main node implementation
│   │   ├── configuration.cpp  # Configuration implementation
│   │   ├── reader.cpp         # File reader implementation
│   │   └── spline_converter.cpp # Spline fitting implementation
│   ├── maps/                  # Map definition files
│   │   ├── test_map.xml       # Example XML map
│   │   └── ...                # Various test maps
│   └── rviz/                  # RViz configurations
│
└── roadmap_msgs/              # Message definitions package
    ├── CMakeLists.txt
    ├── package.xml
    └── msg/
        ├── RoadPolyline.msg   # Single polyline definition
        └── RoadPolylineArray.msg # Array of polylines
```

---

## Launch File Initialization

### What Happens When `roadmap/launch/roadmap.launch` is Used

The launch file is located at: `roadmap/roadmap/launch/roadmap.launch`

```xml
<?xml version="1.0"?>
<launch>
    <arg name="map_file_name" default="maps/mobile_robotics_lab/straight.xml" />
    <arg name="map_package_name" default="roadmap" />
    <arg name="config_file" default="$(find roadmap)/config/settings.yaml" />

    <rosparam command="load" file="$(arg config_file)"/>
    
    <param name="roadmap/map_file_name" value="$(arg map_file_name)"/>    
    <param name="roadmap/map_package_name" value="$(arg map_package_name)"/>    

    <node pkg="roadmap" type="roadmap_node" name="roadmap_node" respawn="false" output="screen"/>
</launch>
```

### Launch Sequence:

1. **Parse Arguments** (Lines 3-5)
   - `map_file_name`: Specifies which map file to load (default: `maps/mobile_robotics_lab/straight.xml`)
   - `map_package_name`: Package containing the map (default: `roadmap`)
   - `config_file`: Path to settings YAML file

2. **Load Configuration** (Line 7)
   - Loads all parameters from `settings.yaml` into the ROS parameter server
   - Sets default values for debug output, update frequency, spline settings, etc.

3. **Override Parameters** (Lines 10-11)
   - Overrides the `map_file_name` and `map_package_name` from the launch file arguments
   - This allows runtime customization without editing `settings.yaml`

4. **Start Node** (Line 13)
   - Launches the `roadmap_node` executable
   - Node name: `roadmap_node`
   - Output: screen (prints to console)
   - No auto-respawn on failure

---

## ROS Node Initialization Flow

### Entry Point: `roadmap_node.cpp`

```cpp
int main(int argc, char **argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  
  Roadmap roadmap;  // Constructor does all initialization
  ros::spin();       // Process callbacks
  
  return 0;
}
```

**Execution Steps:**

1. **ROS Initialization** (`ros::init`)
   - Initializes ROS system
   - Sets up node name from parameter server

2. **Roadmap Object Construction** (`Roadmap roadmap;`)
   - This single line triggers the entire initialization chain
   - See detailed constructor flow below

3. **Event Loop** (`ros::spin()`)
   - Enters ROS event loop
   - Processes timer callbacks (Poll function at 1 Hz by default)
   - Handles incoming messages from subscribers

---

## Core Components Deep Dive

### 1. Roadmap Class (`roadmap.h` / `roadmap.cpp`)

**Purpose:** Main orchestrator that coordinates all components

**Constructor Initialization Sequence:**

```cpp
Roadmap::Roadmap()
{
    // Step 1: Initialize configuration
    config_.reset(new RoadmapConfig());
    config_->initialize();

    // Step 2: Create file reader
    reader_.reset(new Reader(config_.get()));

    // Step 3: Initialize visualization system
    VISUALS.init(&nh_);

    // Step 4: Set up subscribers
    waypoints_sub_ = nh_.subscribe(config_->external_waypoint_topic_, 1, 
                                   &Roadmap::WaypointCallback, this);
    reset_sub_ = nh_.subscribe("/lmpcc/reset_environment", 1, 
                               &Roadmap::ResetCallback, this);
    reverse_sub_ = nh_.subscribe("/roadmap/reverse", 1, 
                                 &Roadmap::ReverseCallback, this);
    offset_sub_ = nh_.subscribe("roadmap/offset", 1, 
                                &Roadmap::OffsetCallback, this);

    // Step 5: Set up publishers
    map_pub_ = nh_.advertise<roadmap_msgs::RoadPolylineArray>(
                   "roadmap/road_polylines", 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>("roadmap/reference", 1);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/roadmap/goal", 1);

    // Step 6: Initialize spline converter
    spline_converter_.Initialize(nh_, config_.get());

    // Step 7: Read map from file
    ReadFromFile();

    // Step 8: Initialize state
    is_reversed_ = false;

    // Step 9: Create timer for periodic publishing
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_->update_frequency_), 
                             &Roadmap::Poll, this);
    
    ROADMAP_WARN("Initialization completed");
}
```

### 2. RoadmapConfig Class (`configuration.h` / `configuration.cpp`)

**Purpose:** Manages all configuration parameters from ROS parameter server

**Key Parameters:**
- `debug_output`: Enable/disable debug logging
- `update_frequency`: Publishing rate (Hz)
- `map_package_name`: Package containing map files
- `map_file_name`: Specific map file to load
- `fit_clothoid`: Whether to fit Clothoid before cubic splines
- `spline_sample_distance`: Distance between sampled waypoints on spline
- `minimum_waypoint_distance`: Ignore waypoints closer than this
- `external_waypoint_topic`: Topic for external waypoint input

**Initialization:**
```cpp
bool RoadmapConfig::initialize()
{
  ros::NodeHandle nh;
  
  retrieveParameter(nh, "roadmap/debug_output", debug_output_, true);
  retrieveParameter(nh, "roadmap/map_package_name", map_package_name_);
  retrieveParameter(nh, "roadmap/map_file_name", map_file_name_);
  retrieveParameter(nh, "roadmap/update_frequency", update_frequency_, 10.);
  // ... more parameters
  
  return true;
}
```

### 3. Reader Class (`reader.h` / `reader.cpp`)

**Purpose:** Reads map data from various file formats or ROS topics

**Key Methods:**

- **`Read()`**: Reads file specified in configuration
  - Determines file type from extension (.xml, .yaml, .osm)
  - Calls appropriate parser

- **`ReadXML(const std::string &file)`**: Parses XML map files
  - Reads `<way>` elements containing waypoints
  - Each `<nd>` defines a waypoint (x, y, theta)
  - Each `<lane>` defines a lane type and width
  - Creates `Way` objects with `Lane` children

- **`ReadYAML(const std::string &file)`**: Parses YAML files (legacy support)

- **`ReadOSM(const std::string &file)`**: Parses OpenStreetMap files
  - Converts lat/lon to local coordinates
  - Builds road network from OSM data

- **`WaypointCallback(const nav_msgs::Path &msg)`**: Receives waypoints via ROS
  - Creates map from incoming Path message
  - Alternative to file-based maps

**Data Storage:**
```cpp
Map map_;  // Stores the loaded map (Vector of Ways, each containing Lanes)
```

### 4. SplineConverter Class (`spline_converter.h` / `spline_converter.cpp`)

**Purpose:** Fits smooth splines through discrete waypoints

**Key Methods:**

- **`ConvertMap(Map &map)`**: Main conversion function
  - Iterates through all Ways and Lanes
  - Calls `FitSplineOnLane` for each lane
  - Returns converted map with fitted splines

- **`FitSplineOnLane(Lane &lane)`**: Fits spline to a single lane
  - Extracts waypoints from lane nodes
  - Calls `FitSplineOnWaypoints`
  - Updates lane nodes with spline-sampled points

- **`FitSplineOnWaypoints(...)`**: Core spline fitting logic
  - Step 1: Optionally fit Clothoid curve for smooth transitions
  - Step 2: Fit cubic splines on the result
  - Samples points along the spline at regular intervals

- **`FitClothoid(...)`**: Fits Clothoid spiral curve
  - Provides smooth curvature transitions
  - Better for vehicle path planning

- **`FitCubicSpline(...)`**: Fits cubic splines
  - Uses `tk::spline` library
  - Creates smooth interpolation between waypoints

### 5. Data Structures (`types.h`)

**Core Types:**

```cpp
struct Node {
    double x, y, theta;  // Position and heading
};

struct Lane {
    int id, type;                      // Lane identifier and type
    std::vector<Node> nodes;           // Waypoints defining the lane
    bool spline_fit;                   // Has spline been fitted?
    double length;                     // Total lane length
    tk::spline spline_x, spline_y;     // Fitted splines
};

struct Way {
    std::vector<Lane> lanes;           // Multiple lanes (road, sidewalk, etc.)
    std::vector<Node> nodes;           // Center line waypoints
    double plus_offset, minus_offset;  // Lane offsets
};

struct Map {
    std::vector<Way> ways;             // Collection of ways
};
```

**Lane Types** (from `RoadPolyline.msg`):
- `LANECENTER_FREEWAY (1)`: Highway/freeway lane center
- `LANECENTER_SURFACESTREET (2)`: Surface street lane center
- `ROADLINE_BROKENSINGLEWHITE (6)`: Broken white line
- `ROADEDGEBOUNDARY (15)`: Road edge
- `CROSSWALK (18)`: Pedestrian crossing
- And more...

---

## Function Interactions in roadmap.cpp

### Main Workflow Functions

#### 1. `ReadFromFile()`
```
ReadFromFile()
    ├── reader_->Read()           // Read map from file
    │   ├── Determine file type
    │   └── Call ReadXML/ReadYAML/ReadOSM
    └── ConvertMap()              // Process the map
        └── (see ConvertMap below)
```

Called: Once during initialization, and when offset changes

#### 2. `ConvertMap()`
```
ConvertMap()
    ├── spline_converter_.ConvertMap(reader_->GetMap())
    │   ├── For each Way:
    │   │   └── For each Lane:
    │   │       └── FitSplineOnLane(lane)
    │   │           ├── Extract waypoints
    │   │           ├── FitClothoid (optional)
    │   │           └── FitCubicSpline
    │   └── Return converted_map_
    │
    ├── Clear message buffers
    ├── reader_->GetMap().ToMsg(road_msg_)    // Convert to RoadPolylineArray
    └── spline_converter_.converted_map_.ToMsg(ref_msg_)  // Convert to Path
```

Called: After reading files or receiving waypoints

#### 3. `Poll(const ros::TimerEvent &event)` - The Main Loop
```
Poll()  [Called at update_frequency_ Hz]
    ├── Increment run counter
    ├── map_pub_.publish(road_msg_)           // Publish road polylines
    ├── reference_pub_.publish(ref_msg_)      // Publish reference path
    ├── spline_converter_.VisualizeInputData()  // Draw input waypoints
    └── spline_converter_.VisualizeMap()        // Draw fitted splines
```

Called: Periodically by timer (default 1 Hz)

#### 4. Callback Functions

**`WaypointCallback(const nav_msgs::Path &msg)`**
```
WaypointCallback()
    ├── reader_->WaypointCallback(msg)  // Store waypoints
    │   ├── Clear existing map
    │   ├── Create new Way from path
    │   └── Add default lanes (road + sidewalk)
    ├── ConvertMap()                     // Process waypoints
    └── reference_pub_.publish(ref_msg_) // Immediate publish
```

**`ReverseCallback(const std_msgs::Empty &msg)`**
```
ReverseCallback()
    ├── Toggle is_reversed_ flag
    ├── Re-initialize config, reader, spline_converter
    ├── reader_->Read()              // Read map again
    ├── If reversed: reader_->GetMap().Reverse()  // Reverse direction
    ├── ConvertMap()                 // Process reversed map
    └── goal_pub_.publish(...)       // Publish new goal
```

**`OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)`**
```
OffsetCallback()
    ├── reader_->OffsetCallback(msg)  // Store offset
    └── ReadFromFile()                // Re-read with new offset
```

**`ResetCallback(const std_msgs::Empty &msg)`**
- Currently commented out (no-op)
- Intended for resetting the map to initial state

---

## File Connections and Dependencies

### Dependency Graph

```
roadmap_node.cpp (main entry)
    │
    └── roadmap.h/cpp (orchestrator)
            ├── configuration.h/cpp (parameters)
            │       └── helpers.h (logging)
            │
            ├── reader.h/cpp (file I/O)
            │       ├── configuration.h (parameters)
            │       ├── types.h (data structures)
            │       └── External: rapidxml (XML parsing)
            │
            ├── spline_converter.h/cpp (spline fitting)
            │       ├── configuration.h (parameters)
            │       ├── types.h (data structures)
            │       └── External: tk::spline, Clothoid library
            │
            └── types.h (data structures)
                    └── roadmap_msgs (ROS messages)
```

### Include Dependencies

**roadmap.h includes:**
- `ros/ros.h` - ROS core
- `geometry_msgs/PoseStamped.h` - Pose messages
- `std_msgs/Empty.h` - Empty messages
- `roadmap_msgs/RoadPolylineArray.h` - Custom messages
- `reader.h` - File reading
- `types.h` - Data structures
- `spline_converter.h` - Spline fitting

**configuration.h includes:**
- `roadmap/helpers.h` - Logging macros
- `ros/ros.h` - ROS parameter access

**reader.h includes:**
- `rapidxml_utils.hpp` - XML parsing
- `roadmap/types.h` - Data structures
- `roadmap/configuration.h` - Parameters
- `nav_msgs/Path.h` - Path messages
- `geometry_msgs/Pose.h` - Pose messages

**spline_converter.h includes:**
- `configuration.h` - Parameters
- `types.h` - Data structures
- External spline libraries

**types.h includes:**
- `roadmap_msgs/RoadPolyline.h` - Message definitions
- `nav_msgs/Path.h` - Path messages
- `geometry_msgs/PoseStamped.h` - Pose messages
- `ros_tools/convertions.h` - Utility functions
- `ros_tools/spline.h` - Spline utilities

### External Dependencies

From `package.xml` and `CMakeLists.txt`:

**ROS Packages:**
- `roscpp` - ROS C++ client
- `roslib` - ROS library utilities
- `geometry_msgs` - Geometry message types
- `std_msgs` - Standard message types
- `nav_msgs` - Navigation message types

**Custom Packages:**
- `roadmap_msgs` - Custom message definitions
- `ros_tools` - Utility library (conversions, visualization, logging)
- `asr_rapidxml` - XML parsing library

**Third-party Libraries:**
- `tk::spline` - Cubic spline implementation
- Clothoid library - Clothoid curve fitting

---

## Data Flow Through the System

### Initialization Flow

```
1. Launch File
   ↓ (loads parameters)
2. ROS Parameter Server
   ↓ (main() creates)
3. Roadmap Constructor
   ↓ (creates)
4. RoadmapConfig::initialize()
   ↓ (reads parameters from server)
5. Reader Constructor
   ↓ (ready to read files)
6. Roadmap::ReadFromFile()
   ↓
7. Reader::Read()
   ↓ (parses file)
8. Map Structure (in reader_)
   ↓
9. SplineConverter::ConvertMap()
   ↓ (fits splines)
10. Converted Map (in spline_converter_)
    ↓
11. ConvertMap() → ToMsg()
    ↓
12. ROS Messages (road_msg_, ref_msg_)
```

### Runtime Publishing Flow

```
Timer Triggers (at update_frequency_)
    ↓
Poll() Function
    ↓
    ├── Publish road_msg_ → /roadmap/road_polylines
    │       (RoadPolylineArray with all lanes)
    │
    ├── Publish ref_msg_ → /roadmap/reference
    │       (nav_msgs::Path with reference trajectory)
    │
    ├── Visualize Input Data (markers in RViz)
    │
    └── Visualize Map (splines in RViz)
```

### Dynamic Waypoint Flow

```
External Source
    ↓ (publishes)
/carla/ego_vehicle/waypoints (nav_msgs::Path)
    ↓ (subscriber)
WaypointCallback()
    ↓
Reader::WaypointCallback()
    ↓ (creates Map from Path)
New Map Structure
    ↓
ConvertMap()
    ↓
Spline Fitting
    ↓
Immediate Publish (ref_msg_)
```

### Map Format Conversion Chain

```
File (XML/YAML/OSM)
    ↓ (Reader::ReadXML/YAML/OSM)
Raw Map Data (Ways with Lanes, discrete waypoints)
    ↓ (SplineConverter::ConvertMap)
Fitted Map (Lanes with splines and dense waypoints)
    ↓ (Map::ToMsg)
ROS Messages
    ├── RoadPolylineArray (all lanes with types)
    └── nav_msgs::Path (reference trajectory only)
```

---

## Message Definitions

### RoadPolyline.msg

```
int32 id                      # Unique lane identifier

# Lane type constants
uint8 LANECENTER_FREEWAY=1
uint8 LANECENTER_SURFACESTREET=2
uint8 LANECENTER_BIKELANE=3
uint8 ROADLINE_BROKENSINGLEWHITE=6
uint8 ROADLINE_SOLIDSINGLEWHITE=7
# ... (see types.h for full list)

uint8 type                    # Type from constants above
geometry_msgs/Point[] coords  # Polyline waypoints
```

### RoadPolylineArray.msg

```
Header header                          # Timestamp and frame_id
roadmap_msgs/RoadPolyline[] road_polylines  # Array of polylines
```

**Published Topics:**
- `/roadmap/road_polylines` (RoadPolylineArray): All lanes with type information
- `/roadmap/reference` (nav_msgs::Path): Reference trajectory for planning
- `/roadmap/goal` (geometry_msgs::PoseStamped): Goal pose (published on reverse)

**Subscribed Topics:**
- Configured via `external_waypoint_topic` (default: `/carla/ego_vehicle/waypoints`)
- `/lmpcc/reset_environment` (std_msgs::Empty): Reset map
- `/roadmap/reverse` (std_msgs::Empty): Reverse map direction
- `roadmap/offset` (geometry_msgs::PoseWithCovarianceStamped): Offset map

---

## Configuration System

### Configuration File: `config/settings.yaml`

```yaml
roadmap:
  # High level settings
  debug_output: false              # Enable debug logging
  update_frequency: 1              # Publishing rate (Hz)
  map_package_name: 'roadmap'      # Package with map files
  map_file_name: 'maps/mobile_robotics_lab/straight.xml'
  
  # Spline settings
  spline:
    fit_clothoid: true                    # Fit Clothoid before cubic spline
    minimum_waypoint_distance: 0.5        # Minimum distance between waypoints
    spline_sample_distance: 2.0           # Sample spacing on spline
    clothoid_point_per_xm: 0.25           # Clothoid point density
  
  # Visual settings
  scale: 1.0                             # Visualization scale
  
  # Topics
  external_waypoint_topic: '/carla/ego_vehicle/waypoints'
```

### Parameter Override Hierarchy

1. **Default values** in `configuration.cpp`
2. **settings.yaml** loaded by launch file
3. **Launch file arguments** override specific parameters
4. **Runtime** parameter changes via ROS parameter server

---

## Example Map File Format

### XML Format (Preferred)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<way>
  <nd x="-10" y="0" theta="0"/>
  <nd x="20" y="0" theta="0"/>
  <nd x="30" y="0" theta="0"/>
  <nd x="50" y="10" theta="1.5708"/> 
  <nd x="50" y="30" theta="1.5708"/>
  
  <!-- Define lanes offset from center -->
  <lane type="road" width="4.0" two_way="1"/>
  <lane type="sidewalk" width="2.0" two_way="1"/>
</way>

<!-- Additional way for crosswalk -->
<way>
  <nd x="44" y="30" theta="0."/>
  <nd x="58" y="30" theta="0."/>
  <lane type="crosswalk" width="2.0" two_way="0"/>
</way>
```

**Elements:**
- `<way>`: Defines a road segment
- `<nd>`: Node/waypoint with x, y position and theta heading (radians)
- `<lane>`: Lane definition
  - `type`: "road", "sidewalk", "crosswalk"
  - `width`: Lane width in meters
  - `two_way`: "1" for bidirectional, "0" for one-way

---

## Summary of Key Points

### When `roadmap.launch` is executed:

1. **Parameters are loaded** from `settings.yaml` to ROS parameter server
2. **Launch arguments** override map file and package name
3. **roadmap_node executable** starts
4. **ROS initialization** occurs
5. **Roadmap constructor** executes complete initialization:
   - Loads configuration from parameter server
   - Creates Reader to handle file I/O
   - Sets up ROS publishers/subscribers
   - Initializes SplineConverter
   - **Reads map file** specified in configuration
   - **Fits splines** through waypoints
   - **Creates ROS messages** with polyline data
   - **Starts timer** for periodic publishing
6. **Event loop** begins processing callbacks
7. **Poll() function** publishes map data at configured rate

### How functions in roadmap.cpp work together:

- **Constructor** sets up entire system and performs initial map loading
- **ReadFromFile()** loads map from disk via Reader, then processes it
- **ConvertMap()** orchestrates spline fitting and message creation
- **Poll()** periodically publishes pre-computed messages and visualizations
- **Callbacks** handle dynamic inputs (waypoints, reverse, offset, reset)
- All functions coordinate through shared state:
  - `reader_`: holds raw map data
  - `spline_converter_`: holds fitted splines
  - `road_msg_`, `ref_msg_`: pre-computed ROS messages

### File Connections:

The system has a layered architecture:
- **Main layer**: `roadmap_node.cpp` → `roadmap.cpp/h`
- **Configuration layer**: `configuration.cpp/h`
- **Data layer**: `types.h`, `roadmap_msgs`
- **Processing layer**: `reader.cpp/h`, `spline_converter.cpp/h`
- **Utilities**: `helpers.h`, `ros_tools` package

Each component has a single responsibility, making the system modular and maintainable.
