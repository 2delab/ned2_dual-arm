╔════════════════════════════════════════════════════════════════════════════════╗
║                 MARS: MULTI-ARM ROBOTIC SYSTEM - COMPLETE ARCHITECTURE         ║
║                        End-to-End Pipeline with Data Flow                       ║
╚════════════════════════════════════════════════════════════════════════════════╝
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LAYER 0: PHYSICAL HARDWARE                                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌──────────────────┐                          ┌──────────────────┐            │
│  │  Niryo NED2      │                          │  Niryo NED2      │            │
│  │   ARM 1          │                          │   ARM 2          │            │
│  │ (6-DOF)          │  Niryo Protocol          │ (6-DOF)          │            │
│  │ X = -0.35m       │◄────────TCP─────────►    │ X = +0.35m       │            │
│  │ IP: 192.168.8.142│  Position Commands       │ IP: 192.168.8.149│            │
│  │                  │  Joint States            │                  │            │
│  │ Gripper          │  Feedback                │ Gripper          │            │
│  └──────────────────┘                          └──────────────────┘            │
│         ▲                                                ▲                       │
│         │                                                │                       │
│         └────────────────────┬─────────────────────────┘                       │
│                              │                                                  │
│                    ROS2 Middleware (DDS)                                       │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: HARDWARE INTERFACE LAYER                                              │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌─────────────────────────────────┐    ┌─────────────────────────────────┐  │
│  │   ROS2 DRIVER ARM 1             │    │   ROS2 DRIVER ARM 2             │  │
│  │   Namespace: /arm_1             │    │   Namespace: /arm_2             │  │
│  ├─────────────────────────────────┤    ├─────────────────────────────────┤  │
│  │ Publishes:                      │    │ Publishes:                      │  │
│  │ • /arm_1/joint_states           │    │ • /arm_2/joint_states           │  │
│  │   [joint_1...joint_6]           │    │   [joint_1...joint_6]           │  │
│  │   (40 Hz)                       │    │   (40 Hz)                       │  │
│  │                                 │    │                                 │  │
│  │ • /arm_1/joint_states_diff      │    │ • /arm_2/joint_states_diff      │  │
│  │ • /arm_1/tf_static              │    │ • /arm_2/tf_static              │  │
│  │                                 │    │                                 │  │
│  │ Subscribes to:                  │    │ Subscribes to:                  │  │
│  │ • /arm_1/controller/            │    │ • /arm_2/controller/            │  │
│  │   follow_joint_trajectory       │    │   follow_joint_trajectory       │  │
│  │   (Position commands)           │    │   (Position commands)           │  │
│  │                                 │    │                                 │  │
│  │ Gripper Interface:              │    │ Gripper Interface:              │  │
│  │ • Tool action server            │    │ • Tool action server            │  │
│  │ • OPEN/CLOSE commands           │    │ • OPEN/CLOSE commands           │  │
│  └─────────────────────────────────┘    └─────────────────────────────────┘  │
│         ▲                                           ▲                           │
│         │ Niryo Protocol                           │ Niryo Protocol           │
│         └──────────────────────┬────────────────────┘                         │
│                                │                                               │
└─────────────────────────────────┼───────────────────────────────────────────────┘
                                  │
        Data Flow: Joint states flow UP, trajectory commands flow DOWN
        
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: STATE MANAGEMENT LAYER                                                │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │              JOINT STATE AGGREGATOR NODE                                  │  │
│  │         ros2_driver_state_aggregator / joint_state_prefixer              │  │
│  ├──────────────────────────────────────────────────────────────────────────┤  │
│  │                                                                            │  │
│  │  INPUTS (Subscribers):                                                   │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐    │  │
│  │  │ /arm_1/joint_states (JointState)  [40 Hz]                      │    │  │
│  │  │ /arm_2/joint_states (JointState)  [40 Hz]                      │    │  │
│  │  │ Format: name=[joint_1..6], position=[...], velocity, effort   │    │  │
│  │  └─────────────────────────────────────────────────────────────────┘    │  │
│  │                                                                            │  │
│  │  INTERNAL PROCESSING (Thread-Safe):                                      │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐    │  │
│  │  │ ┌─────────────────────────────────────────────────────────────┐ │    │  │
│  │  │ │ State Dictionary (Protected by Mutex):                      │ │    │  │
│  │  │ │ {                                                           │ │    │  │
│  │  │ │   "arm_1": JointState(...),  # unprefixed names            │ │    │  │
│  │  │ │   "arm_2": JointState(...)   # unprefixed names            │ │    │  │
│  │  │ │ }                                                           │ │    │  │
│  │  │ └─────────────────────────────────────────────────────────────┘ │    │  │
│  │  │                                                                    │    │  │
│  │  │ Callback Group: MutuallyExclusiveCallbackGroup                   │    │  │
│  │  │ (Ensures serial state updates, prevents race conditions)         │    │  │
│  │  │                                                                    │    │  │
│  │  │ Timer: publish_combined_states() @ 40 Hz                         │    │  │
│  │  │ ┌─────────────────────────────────────────────────────────────┐ │    │  │
│  │  │ │ NAMESPACE PREFIXING:                                        │ │    │  │
│  │  │ │   arm_1 joint_1  ──►  arm_1_joint_1                         │ │    │  │
│  │  │ │   arm_1 joint_2  ──►  arm_1_joint_2                         │ │    │  │
│  │  │ │   ...                                                        │ │    │  │
│  │  │ │   arm_2 joint_1  ──►  arm_2_joint_1                         │ │    │  │
│  │  │ │   arm_2 joint_2  ──►  arm_2_joint_2                         │ │    │  │
│  │  │ │   ...                                                        │ │    │  │
│  │  │ └─────────────────────────────────────────────────────────────┘ │    │  │
│  │  └─────────────────────────────────────────────────────────────────┘    │  │
│  │                                                                            │  │
│  │  OUTPUTS (Publishers):                                                   │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐    │  │
│  │  │ /joint_states (JointState)  [40 Hz]                            │    │  │
│  │  │ Format: name=[arm_1_joint_1, arm_1_joint_2, ...,              │    │  │
│  │  │                 arm_2_joint_1, arm_2_joint_2, ...]            │    │  │
│  │  │         position=[θ₁₁, θ₁₂, ..., θ₂₁, θ₂₂, ...]             │    │  │
│  │  │                                                                 │    │  │
│  │  │ /tf_static (TransformStamped) - Arm base frames               │    │  │
│  │  │ /tf (TransformStamped) - Link transforms [15 Hz]             │    │  │
│  │  └─────────────────────────────────────────────────────────────────┘    │  │
│  │                                                                            │  │
│  │  TRAJECTORY PROXY (Bidirectional):                                       │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐    │  │
│  │  │ Action Servers (from MoveIt2):                                  │    │  │
│  │  │ • /arm_1/follow_joint_trajectory_prefixed (FollowJointTrajectory)   │    │  │
│  │  │ • /arm_2/follow_joint_trajectory_prefixed (FollowJointTrajectory)   │    │  │
│  │  │   Input: [arm_1_joint_1, arm_1_joint_2, ..., arm_2_joint_...]      │    │  │
│  │  │   [PREFIXED NAMES]                                              │    │  │
│  │  │                                                                 │    │  │
│  │  │ NAMESPACE STRIPPING:                                           │    │  │
│  │  │   arm_1_joint_1  ──►  joint_1                                  │    │  │
│  │  │   arm_1_joint_2  ──►  joint_2                                  │    │  │
│  │  │   ...                                                           │    │  │
│  │  │   arm_2_joint_1  ──►  joint_1                                  │    │  │
│  │  │   arm_2_joint_2  ──►  joint_2                                  │    │  │
│  │  │   ...                                                           │    │  │
│  │  │                                                                 │    │  │
│  │  │ Action Clients (to Hardware):                                  │    │  │
│  │  │ • /arm_1/niryo_robot_follow_joint_trajectory_controller/      │    │  │
│  │  │   follow_joint_trajectory (FollowJointTrajectory)              │    │  │
│  │  │ • /arm_2/niryo_robot_follow_joint_trajectory_controller/      │    │  │
│  │  │   follow_joint_trajectory (FollowJointTrajectory)              │    │  │
│  │  │   Output: [joint_1, joint_2, ..., joint_6] [UNPREFIXED]       │    │  │
│  │  │                                                                 │    │  │
│  │  │ Callback Group: ReentrantCallbackGroup                        │    │  │
│  │  │ (Allows concurrent trajectory execution)                       │    │  │
│  │  │                                                                 │    │  │
│  │  │ Execution Monitoring:                                          │    │  │
│  │  │ • Timeout: 120 seconds                                         │    │  │
│  │  │ • Feedback relay: position error, velocity tracking           │    │  │
│  │  │ • Result propagation: success/failure status                  │    │  │
│  │  └─────────────────────────────────────────────────────────────────┘    │  │
│  │                                                                            │  │
│  │  EXECUTOR CONFIGURATION:                                                 │  │
│  │  • MultiThreadedExecutor with 8 threads                                 │  │
│  │  • Two Callback Groups:                                                 │  │
│  │    - MutuallyExclusive: Joint state subs + publisher                   │  │
│  │    - Reentrant: Action servers for trajectory execution                │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
        ▲                                                          ▲
        │ Unified 12-DOF State (arm_1_joint_1...arm_2_joint_6)     │
        │ 40 Hz Publication                                        │
        │                                                          │
        │ Prefixed Trajectories                                   │
        │ Asynchronous routing                                    │
        │                                                          │
        
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3: PLANNING & COORDINATION LAYER                                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                ROBOT STATE PUBLISHER                                     │  │
│  ├──────────────────────────────────────────────────────────────────────────┤  │
│  │ Subscribes: /joint_states (40 Hz)                                        │  │
│  │ Publishes:  /tf (Transform tree) [15 Hz]                                │  │
│  │             /tf_static (Fixed transforms)                               │  │
│  │                                                                          │  │
│  │ Computes forward kinematics for all links:                              │  │
│  │ • arm_1_base_link ──► arm_1_shoulder_link ──► ... ──► arm_1_tool_link  │  │
│  │ • arm_2_base_link ──► arm_2_shoulder_link ──► ... ──► arm_2_tool_link  │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│         ▲                                                         ▼              │
│         │                                             TF Tree (world frame)      │
│         │                                                         ▼              │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                 MOVEIT2 PLANNING SCENE                                   │  │
│  │          (Centralized Collision-Aware Planning Infrastructure)           │  │
│  ├──────────────────────────────────────────────────────────────────────────┤  │
│  │                                                                          │  │
│  │  SCENE COMPONENTS:                                                      │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │  │
│  │  │ URDF Robot Model (niryo_ned2_dual.urdf):                        │   │  │
│  │  │                                                                  │   │  │
│  │  │  ┌─────────────────────────────────────────────────────────┐   │   │  │
│  │  │  │ ARM 1 KINEMATIC CHAIN (7 links):                        │   │   │  │
│  │  │  │ arm_1_base_link (fixed)                                 │   │   │  │
│  │  │  │  └─[arm_1_joint_1]─► arm_1_shoulder_link               │   │   │  │
│  │  │  │     └─[arm_1_joint_2]─► arm_1_arm_link                 │   │   │  │
│  │  │  │        └─[arm_1_joint_3]─► arm_1_elbow_link            │   │   │  │
│  │  │  │           └─[arm_1_joint_4]─► arm_1_forearm_link       │   │   │  │
│  │  │  │              └─[arm_1_joint_5]─► arm_1_wrist_link      │   │   │  │
│  │  │  │                 └─[arm_1_joint_6]─► arm_1_hand_link    │   │   │  │
│  │  │  │                    └─► arm_1_tool_link                 │   │   │  │
│  │  │  │                    └─► arm_1_gripper (base_gripper_1,  │   │   │  │
│  │  │  │                         mors_1, mors_2, camera_link)   │   │   │  │
│  │  │  └─────────────────────────────────────────────────────────┘   │   │  │
│  │  │                                                                  │   │  │
│  │  │  ┌─────────────────────────────────────────────────────────┐   │   │  │
│  │  │  │ ARM 2 KINEMATIC CHAIN (7 links):                        │   │   │  │
│  │  │  │ arm_2_base_link (fixed at X=+0.35m)                    │   │   │  │
│  │  │  │  └─[arm_2_joint_1]─► arm_2_shoulder_link               │   │   │  │
│  │  │  │     └─[arm_2_joint_2]─► arm_2_arm_link                 │   │   │  │
│  │  │  │        └─[arm_2_joint_3]─► arm_2_elbow_link            │   │   │  │
│  │  │  │           └─[arm_2_joint_4]─► arm_2_forearm_link       │   │   │  │
│  │  │  │              └─[arm_2_joint_5]─► arm_2_wrist_link      │   │   │  │
│  │  │  │                 └─[arm_2_joint_6]─► arm_2_hand_link    │   │   │  │
│  │  │  │                    └─► arm_2_tool_link                 │   │   │  │
│  │  │  │                    └─► arm_2_gripper (base_gripper_1,  │   │   │  │
│  │  │  │                         mors_1, mors_2, camera_link)   │   │   │  │
│  │  │  └─────────────────────────────────────────────────────────┘   │   │  │
│  │  │                                                                  │   │  │
│  │  │  ┌─────────────────────────────────────────────────────────┐   │   │  │
│  │  │  │ STATIC COLLISION OBJECTS:                               │   │   │  │
│  │  │  │ • Ground plane (3m × 3m platform)                      │   │   │  │
│  │  │  │ • Workspace boundaries                                 │   │   │  │
│  │  │  │ • (Optional: dynamic obstacles added via planning)     │   │   │  │
│  │  │  └─────────────────────────────────────────────────────────┘   │   │  │
│  │  └─────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  COLLISION GEOMETRIES WITH PADDING:                                    │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │  │
│  │  │ Per-Link Padding: 5 cm (conservative safety margin)             │   │  │
│  │  │                                                                  │   │  │
│  │  │ Padded Links:                                                   │   │  │
│  │  │ ARM 1: base_link, shoulder_link, arm_link, elbow_link,         │   │  │
│  │  │        forearm_link, wrist_link, hand_link                     │   │  │
│  │  │        (+ gripper: base_gripper_1, mors_1, mors_2)             │   │  │
│  │  │                                                                  │   │  │
│  │  │ ARM 2: base_link, shoulder_link, arm_link, elbow_link,         │   │  │
│  │  │        forearm_link, wrist_link, hand_link                     │   │  │
│  │  │        (+ gripper: base_gripper_1, mors_1, mors_2)             │   │  │
│  │  │                                                                  │   │  │
│  │  │ Padding Effect: 10cm minimum separation between arms            │   │  │
│  │  │ (5cm padding × 2 approaching links)                             │   │  │
│  │  └─────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  COLLISION MATRIX (SRDF - Semantic Robot Description):                 │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐   │  │
│  │  │ Disabled Collision Pairs (~143):                                │   │  │
│  │  │                                                                  │   │  │
│  │  │ ADJACENT LINKS (connected by joint - cannot collide):           │   │  │
│  │  │ • arm_1_shoulder_link ↔ arm_1_base_link (via joint_1)          │   │  │
│  │  │ • arm_1_arm_link ↔ arm_1_shoulder_link (via joint_2)           │   │  │
│  │  │ • ... (similar for all adjacent pairs in both arms)            │   │  │
│  │  │                                                                  │   │  │
│  │  │ NEVER-COLLIDING LINKS (separated by kinematic distance):       │   │  │
│  │  │ • arm_1_base_link ↔ arm_1_elbow_link                           │   │  │
│  │  │ • arm_1_arm_link ↔ arm_1_hand_link                             │   │  │
│  │  │ • ... (similar non-interfering pairs)                          │   │  │
│  │  │                                                                  │   │  │
│  │  │ INTER-ARM BASE LINKS (fixed separation):                       │   │  │
│  │  │ • arm_1_base_link ↔ arm_2_base_link (70cm apart, fixed)        │   │  │
│  │  │                                                                  │   │  │
│  │  │ GRIPPERS (when niryo_ned2_dual_gripper_moveit_config):         │   │  │
│  │  │ • arm_1 gripper pairs                                           │   │  │
│  │  │ • arm_2 gripper pairs                                           │   │  │
│  │  │                                                                  │   │  │
│  │  │ Performance: ~75% reduction in active collision checks          │   │  │
│  │  │ (from ~200 potential pairs to ~50 active pairs)                │   │  │
│  │  └─────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  SCENE UPDATE FREQUENCY: 15 Hz                                         │  │
│  │  (Updates from /joint_states → collision-free path generation)         │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│         ▲                                           ▼                           │
│         │                              Planning Requests                       │
│         │ /joint_states                                                       │
│         │ (12-DOF state)              Planning Results (Trajectories)          │
│         │                                                                      │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │              MOVEIT2 PLANNING INFRASTRUCTURE                             │  │
│  │                (move_group node)                                         │  │
│  ├──────────────────────────────────────────────────────────────────────────┤  │
│  │                                                                          │  │
│  │  THREE COORDINATION MODES - PLANNING INTERFACES:                        │  │
│  │                                                                          │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │  │ MODE 1: SYNCHRONOUS COORDINATION (Dual Group Planning)          │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Group: "dual" (all 12 joints)                           │   │  │
│  │  │ ┌─────────────────────────────────────────────────────────────┐ │   │  │
│  │  │ │ Planning Request:                                           │ │   │  │
│  │  │ │ • Start State: current joint positions                      │ │   │  │
│  │  │ │ • Goal State: target [θ₁₁...θ₁₆, θ₂₁...θ₂₆] (12-DOF)     │ │   │  │
│  │  │ │ • Planning Group: "dual"                                    │ │   │  │
│  │  │ │ • Timeout: 5 seconds                                        │ │   │  │
│  │  │ │ • Planner: RRTConnect (bidirectional RRT)                  │ │   │  │
│  │  │ │ • Max Planning Attempts: 10                                │ │   │  │
│  │  │ │ • Velocity Scaling: 30%                                    │ │   │  │
│  │  │ │ • Acceleration Scaling: 30%                                │ │   │  │
│  │  │ └─────────────────────────────────────────────────────────────┘ │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Pipeline:                                               │   │  │
│  │  │   Request ──► Adapters ──► OMPL Planner ──► Adapters ──► Result │   │  │
│  │  │                                                                   │   │  │
│  │  │   Request Adapters:                                              │   │  │
│  │  │   • ResolveConstraintFrames                                     │   │  │
│  │  │   • ValidateWorkspaceBounds                                     │   │  │
│  │  │   • CheckStartStateBounds                                       │   │  │
│  │  │   • CheckStartStateCollision ◄─── Collision Check              │   │  │
│  │  │                                                                   │   │  │
│  │  │   Core OMPL Planner:                                             │   │  │
│  │  │   • RRTConnect (default): Bidirectional search from start/goal   │   │  │
│  │  │   • Alternatives: RRT, RRTstar, PRM, KPIECE, BKPIECE           │   │  │
│  │  │   • Configuration Space: 12-DOF                                  │   │  │
│  │  │   • Collision Checking: At each sampled configuration            │   │  │
│  │  │   • Sample Count: ~1000s per planning query                      │   │  │
│  │  │                                                                   │   │  │
│  │  │   Response Adapters:                                             │   │  │
│  │  │   • AddTimeOptimalParameterization ──► Velocity/Accel           │   │  │
│  │  │   • ValidateSolution ◄─── Final Collision Check                │   │  │
│  │  │   • DisplayMotionPath (RViz visualization)                      │   │  │
│  │  │                                                                   │   │  │
│  │  │ Output: Trajectory with synchronized timing                      │   │  │
│  │  │   ┌─────────────────────────────────────────────────────────┐   │   │  │
│  │  │   │ Trajectory:                                             │   │   │  │
│  │  │   │ • joint_names: [arm_1_joint_1...arm_1_joint_6,         │   │   │  │
│  │  │   │                 arm_2_joint_1...arm_2_joint_6]         │   │   │  │
│  │  │   │ • waypoints: [{positions, velocities, accelerations,   │   │   │  │
│  │  │   │              time_from_start}, ...]                    │   │   │  │
│  │  │   │ • Timing: Both arms reach waypoints SIMULTANEOUSLY     │   │   │  │
│  │  │   └─────────────────────────────────────────────────────────┘   │   │  │
│  │  │                                                                   │   │  │
│  │  │ Use Cases: Bimanual grasping, synchronized assembly              │   │  │
│  │  └──────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │  │ MODE 2: HYBRID COORDINATION (Dual Planning, Parallel Execution) │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Phase: SAME AS MODE 1                                   │   │  │
│  │  │ • Uses "dual" group (12-DOF planning)                            │   │  │
│  │  │ • Produces synchronized 12-joint trajectory                      │   │  │
│  │  │                                                                   │   │  │
│  │  │ Execution Phase: DIFFERENT FROM MODE 1                           │   │  │
│  │  │ ┌─────────────────────────────────────────────────────────────┐ │   │  │
│  │  │ │ Trajectory Splitting:                                       │ │   │  │
│  │  │ │                                                              │ │   │  │
│  │  │ │ Unified Trajectory:                                         │ │   │  │
│  │  │ │ [wp0: arm_1_joint_1..6 + arm_2_joint_1..6, t=0.0s]         │ │   │  │
│  │  │ │ [wp1: arm_1_joint_1..6 + arm_2_joint_1..6, t=0.5s]         │ │   │  │
│  │  │ │ [wp2: arm_1_joint_1..6 + arm_2_joint_1..6, t=1.2s]         │ │   │  │
│  │  │ │ ...                                                          │ │   │  │
│  │  │ │                                                              │ │   │  │
│  │  │ │                         ↓ SPLIT                              │ │   │  │
│  │  │ │                                                              │ │   │  │
│  │  │ │ ARM 1 Trajectory:                                            │ │   │  │
│  │  │ │ [wp0: arm_1_joint_1..6, t=0.0s]                            │ │   │  │
│  │  │ │ [wp1: arm_1_joint_1..6, t=0.5s]                            │ │   │  │
│  │  │ │ [wp2: arm_1_joint_1..6, t=1.2s]                            │ │   │  │
│  │  │ │                                                              │ │   │  │
│  │  │ │ ARM 2 Trajectory:                                            │ │   │  │
│  │  │ │ [wp0: arm_2_joint_1..6, t=0.0s]                            │ │   │  │
│  │  │ │ [wp1: arm_2_joint_1..6, t=0.5s]                            │ │   │  │
│  │  │ │ [wp2: arm_2_joint_1..6, t=1.2s]                            │ │   │  │
│  │  │ │                                                              │ │   │  │
│  │  │ │ Note: Timestamps preserved, names un-prefixed              │ │   │  │
│  │  │ └─────────────────────────────────────────────────────────────┘ │   │  │
│  │  │                                                                   │   │  │
│  │  │ Use Cases: Multi-phase assembly, workspace handoff               │   │  │
│  │  └──────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │  │ MODE 3: ASYNCHRONOUS COORDINATION (Independent Planning)        │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Group 1: "arm_1" (6 joints)                             │   │  │
│  │  │ Planning Group 2: "arm_2" (6 joints)                             │   │  │
│  │  │ No cross-arm collision checking during planning                  │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Request (ARM 1):                                         │   │  │
│  │  │ • Start State: current ARM 1 joint positions                     │   │  │
│  │  │ • Goal State: target [θ₁₁...θ₁₆] (6-DOF)                       │   │  │
│  │  │ • Planning Group: "arm_1"                                        │   │  │
│  │  │ → Lower dimensional search → Faster planning                     │   │  │
│  │  │                                                                   │   │  │
│  │  │ Planning Request (ARM 2):                                         │   │  │
│  │  │ • Start State: current ARM 2 joint positions                     │   │  │
│  │  │ • Goal State: target [θ₂₁...θ₂₆] (6-DOF)                       │   │  │
│  │  │ • Planning Group: "arm_2"                                        │   │  │
│  │  │                                                                   │   │  │
│  │  │ Execution: Parallel, non-blocking, callback-based                │   │  │
│  │  │                                                                   │   │  │
│  │  │ Use Cases: Parallel pick-and-place, separate workspace zones     │   │  │
│  │  └──────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  INVERSE KINEMATICS (IK) SOLVER:                                        │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │  │ For Cartesian Goals (end-effector poses):                       │   │  │
│  │  │                                                                   │   │  │
│  │  │ IK Request:                                                      │   │  │
│  │  │ • Target Pose: [x, y, z, qx, qy, qz, qw]                       │   │  │
│  │  │ • End-effector Link: arm_1_tool_link (or arm_2_tool_link)       │   │  │
│  │  │ • Seed State: current joint positions                           │   │  │
│  │  │                                                                   │   │  │
│  │  │ Solver: KDL (Kinematics and Dynamics Library)                    │   │  │
│  │  │ • Method: Jacobian-based iterative solving                       │   │  │
│  │  │ • Timeout: 5 milliseconds per attempt                           │   │  │
│  │  │ • Search Resolution: 0.005 radians (~0.3 degrees)              │   │  │
│  │  │ • Attempts: 3 (with perturbed seeds on failure)                │   │  │
│  │  │                                                                   │   │  │
│  │  │ IK Output: Joint configuration [θ₁, θ₂, ..., θ₆]              │   │  │
│  │  │ • Collision-aware: Solutions in collision are rejected           │   │  │
│  │  │ • Seeded search: Prefer solutions near current state             │   │  │
│  │  └──────────────────────────────────────────────────────────────────┘   │  │
│  │                                                                          │  │
│  │  TRAJECTORY TIME PARAMETERIZATION:                                      │  │
│  │  ┌──────────────────────────────────────────────────────────────────┐   │  │
│  │  │ Input: Geometric path (sequence of collision-free waypoints)    │   │  │
│  │  │                                                                   │   │  │
│  │  │ Process:                                                          │   │  │
│  │  │ 1. For each waypoint, compute maximum safe velocity             │   │  │
│  │  │    based on:                                                     │   │  │
│  │  │    • Joint velocity limits (reduced by velocity_scaling: 30%)   │   │  │
│  │  │    • Path curvature (tighter curves → lower velocities)         │   │  │
│  │  │                                                                   │   │  │
│  │  │ 2. Compute acceleration profile:                                │   │  │
│  │  │    • Max acceleration = joint_accel_limit × accel_scaling (30%) │   │  │
│  │  │    • Trapezoidal profile: ramp-up, constant, ramp-down          │   │  │
│  │  │                                                                   │   │  │
│  │  │ 3. Generate trajectory with velocities/accelerations             │   │  │
│  │  │                                                                   │   │  │
│  │  │ Output: Time-parameterized trajectory                            │   │  │
│  │  │ [wp0: pos=[...], vel=[...], accel=[...], t=0.0s]                │   │  │
│  │  │ [wp1: pos=[...], vel=[...], accel=[...], t=0.3s]                │   │  │
│  │  │ [wp2: pos=[...], vel=[...], accel=[...], t=0.8s]                │   │  │
│  │  │ ...                                                               │   │  │
│  │  └──────────────────────────────────────────────────────────────────┘   │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
        ▲                                                          ▼
        │ Trajectory Requests (joint/Cartesian goals)             │
        │ arm_1 group, arm_2 group, or dual group                │
        │                                                         │
        │ Trajectory Results (prefixed joint names)              │
        │ With synchronized or independent timing               │
┌─────────────────────────────────────────────────────────────────────────────────┐
│ LAYER 4: EXECUTION LAYER                                                        │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌──────────────────────────────────────────────────────────────────────────┐  │
│  │                     EXECUTION DISPATCHER                                 │  │
│  ├──────────────────────────────────────────────────────────────────────────┤  │
│  │                                                                          │  │
│  │  Routes trajectories to appropriate controllers based on:               │  │
│  │  • Coordination mode (Sync/Hybrid/Async)                               │  │
│  │  • Joint names in trajectory (prefix determines target arm)             │  │
│  │  • Execution policy (blocking vs. asynchronous)                        │  │
│  │                                                                          │  │
│  │  Decision Logic:                                                        │  │
│  │  if all joints from both arms → Send to Dual Controller (Mode 1)       │  │
│  │  if split by arm + non-blocking → Send to Arm Controllers (Mode 2/3)   │  │
│  │  else → Error: Invalid trajectory configuration                        │  │
│  └──────────────────────────────────────────────────────────────────────────┘  │
│         │                                           │                          │
│         ▼                                           ▼                          │
│  ┌────────────────────────────────┐      ┌────────────────────────────────┐  │
│  │  CONTROLLER ARM 1              │      │  CONTROLLER ARM 2              │  │
│  │  (JointTrajectoryController)   │      │  (JointTrajectoryController)   │  │
│  ├────────────────────────────────┤      ├────────────────────────────────┤  │
│  │ Input Action Server:           │      │ Input Action Server:           │  │
│  │ /arm_1/follow_joint_trajectory │      │ /arm_2/follow_joint_trajectory │  │
│  │ _prefixed (FollowJointTraj.)   │      │ _prefixed (FollowJointTraj.)   │  │
│  │                                │      │                                │  │
│  │ Trajectory Processing:         │      │ Trajectory Processing:         │  │
│  │ • Accepts: arm_1_joint_1..6    │      │ • Accepts: arm_2_joint_1..6    │  │
│  │ • Strips prefixes              │      │ • Strips prefixes              │  │
│  │ • Creates commands for         │      │ • Creates commands for         │  │
│  │   joint_1..6                   │      │   joint_1..6                   │  │
│  │                                │      │                                │  │
│  │ Control Loop (100 Hz):         │      │ Control Loop (100 Hz):         │  │
│  │ ┌──────────────────────────┐   │      │ ┌──────────────────────────┐   │  │
│  │ │ for each 10ms interval:  │   │      │ │ for each 10ms interval:  │   │  │
│  │ │                          │   │      │ │                          │   │  │
│  │ │ 1. Current Time          │   │      │ │ 1. Current Time          │   │  │
│  │ │ 2. Interpolate Trajectory│   │      │ │ 2. Interpolate Trajectory│   │  │
│  │ │    cubic_spline(t_now)   │   │      │ │    cubic_spline(t_now)   │   │  │
│  │ │    → target_position[6]  │   │      │ │    → target_position[6]  │   │  │
│  │ │ 3. Read Current Position │   │      │ │ 3. Read Current Position │   │  │
│  │ │    from joint_states     │   │      │ │    from joint_states     │   │  │
│  │ │ 4. Compute PID Control   │   │      │ │ 4. Compute PID Control   │   │  │
│  │ │    error = target - current   │      │ │    error = target - current   │  │
│  │ │ 5. Publish Position      │   │      │ │ 5. Publish Position      │   │  │
│  │ │    Commands to Hardware  │   │      │ │    Commands to Hardware  │   │  │
│  │ │ 6. Publish Feedback      │   │      │ │ 6. Publish Feedback      │   │  │
│  │ │    (position error,      │   │      │ │    (position error,      │   │  │
│  │ │     velocity tracking)   │   │      │ │     velocity tracking)   │   │  │
│  │ │                          │   │      │ │                          │   │  │
│  │ └──────────────────────────┘   │      │ └──────────────────────────┘   │  │
│  │                                │      │                                │  │
│  │ Output Action Server:          │      │ Output Action Server:          │  │
│  │ /arm_1/niryo_robot_follow_...  │      │ /arm_2/niryo_robot_follow_...  │  │
│  │ _trajectory_controller/        │      │ _trajectory_controller/        │  │
│  │ follow_joint_trajectory        │      │ follow_joint_trajectory        │  │
│  │                                │      │                                │  │
│  │ Sends: [joint_1..6] commands   │      │ Sends: [joint_1..6] commands   │  │
│  │ Receives: Feedback & Results   │      │ Receives: Feedback & Results   │  │
│  │                                │      │                                │  │
│  │ Execution Monitoring:          │      │ Execution Monitoring:          │  │
│  │ • Position Error Tracking      │      │ • Position Error Tracking      │  │
│  │ • Velocity Profile Monitoring  │      │ • Velocity Profile Monitoring  │  │
│  │ • Timeout: 120 seconds         │      │ • Timeout: 120 seconds         │  │
│  │                                │      │                                │  │
│  └────────────────────────────────┘      └────────────────────────────────┘  │
│         │                                           │                         │
│         └───────────────┬──────────────────────────┘                         │
│                         ▼                                                     │
│  ┌────────────────────────────────────────────────────────────────────────┐  │
│  │              DUAL CONTROLLER (Mode 1 - Synchronous Only)               │  │
│  ├────────────────────────────────────────────────────────────────────────┤  │
│  │ Input Action Server:                                                   │  │
│  │ /arm_1/follow_joint_trajectory_prefixed (12-joint trajectory)          │  │
│  │ /arm_2/follow_joint_trajectory_prefixed (12-joint trajectory)          │  │
│  │                                                                         │  │
│  │ NOTE: When dual trajectory arrives, it's split internally              │  │
│  │ and sent to both arm controllers SIMULTANEOUSLY                        │  │
│  │                                                                         │  │
│  │ Synchronized Execution:                                                │  │
│  │ ARM 1: ════════════════════► (0 - 2.5 seconds)                        │  │
│  │ ARM 2: ════════════════════► (0 - 2.5 seconds)  [SYNCHRONIZED]        │  │
│  │                                                                         │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│         │                                           │                         │
│         └───────────────┬──────────────────────────┘                         │
│                         ▼                                                     │
└─────────────────────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────────────────────┐
│ HARDWARE EXECUTION (Back to Layer 0)                                            │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  Controller interpolates trajectory → Position commands → Hardware Controller    │
│  Hardware follows position commands with its own servo loops                    │
│                                                                                  │
│  ┌──────────────────┐                          ┌──────────────────┐            │
│  │  Niryo NED2      │                          │  Niryo NED2      │            │
│  │   ARM 1          │                          │   ARM 2          │            │
│  │                  │  ◄─────Position Cmd─────  │                  │            │
│  │ Servo @ 100Hz    │  ────Actual Position───►  │ Servo @ 100Hz    │            │
│  │                  │  Gripper Command/Status   │                  │            │
│  │                  │  ◄─────────────────────►  │                  │            │
│  │                  │                          │                  │            │
│  └──────────────────┘                          └──────────────────┘            │
│                                                                                  │
│  Physical motion of arms executing the planned trajectory                       │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
═════════════════════════════════════════════════════════════════════════════════════
SUMMARY: COMPLETE DATA FLOW
1. HARDWARE PUBLISHES STATE (40 Hz):
   /arm_1/joint_states [joint_1..6]   ─┐
   /arm_2/joint_states [joint_1..6]   ─┤
                                        └──► Joint State Aggregator
                                        
2. STATE AGGREGATION (40 Hz):
   Joint State Aggregator ──► Applies namespace prefixes
   /joint_states [arm_1_joint_1..6, arm_2_joint_1..6]
   
3. PLANNING SCENE UPDATE (15 Hz):
   /joint_states ──► Robot State Publisher ──► /tf (transform tree)
   /joint_states ──► MoveIt Planning Scene (collision checking)
   
4. MOTION PLANNING (On Request):
   Planning Request (goal + mode) ──► Motion Planner
   ├─ Mode 1 (Sync): "dual" group (12-DOF)
   ├─ Mode 2 (Hybrid): "dual" group (12-DOF) + split for execution
   └─ Mode 3 (Async): "arm_1" & "arm_2" groups (6-DOF each)
   
   Planning Pipeline:
   Request Adapters ──► OMPL Planner ──► Response Adapters
   ├─ Collision checking at each sample
   ├─ Time-optimal parameterization
   └─ Final trajectory validation
   
5. TRAJECTORY EXECUTION:
   
   MODE 1 (SYNCHRONOUS):
   Trajectory ──► Split → Arm 1 Controller ──┐
                                              ├──► Parallel execution (synchronized)
                        Arm 2 Controller ──┘
   
   MODE 2 (HYBRID):
   Trajectory ──► Split → Arm 1 Controller ──┐
                                              ├──► Parallel execution (independent timing)
                        Arm 2 Controller ──┘
   
   MODE 3 (ASYNCHRONOUS):
   Trajectory (Arm 1) ──► Arm 1 Controller ──► Asynchronous execution
   Trajectory (Arm 2) ──► Arm 2 Controller ──► (via callbacks)
   
6. TRAJECTORY PROXY ROUTING:
   Prefixed Trajectory ──► Trajectory Proxy
   ├─ Extract joint names
   ├─ Determine target arms
   ├─ Strip namespace prefixes
   ├─ Route to appropriate controller(s)
   └─ Monitor execution
   
7. CONTROLLER EXECUTION (100 Hz):
   For each 10ms period:
   ├─ Interpolate target position from trajectory
   ├─ Read current position from hardware
   ├─ PID Control: compute correction
   ├─ Send position commands to hardware
   └─ Publish feedback (position error, velocities)
   
8. HARDWARE RESPONSE (100+ Hz):
   Hardware servo loops follow position commands
   Actual joint positions fed back to state publishers (40 Hz)
   → Loop returns to step 1
═════════════════════════════════════════════════════════════════════════════════════
KEY RATES AND FREQUENCIES:
Hardware Driver Publishing (Step 1):        40 Hz
Joint State Aggregation (Step 2):           40 Hz
Transform Tree Update (Step 3):             15 Hz
Trajectory Controller Execution (Step 7):  100 Hz
Hardware Servo Loops (Step 8):            100+ Hz
Planning Frequency:                        On-demand (typically 0.2-2 Hz for complex tasks)
Planning Timeout:                          5 seconds maximum
═════════════════════════════════════════════════════════════════════════════════════
COLLISION AVOIDANCE INTEGRATION:
At planning time:
• Collision checking in request adapters (start state validation)
• Collision checking during OMPL exploration (each sampled config)
• Collision checking in response adapters (final path validation)
• Link padding (5cm) ensures safety margins
At execution time:
• Phase-based execution decomposes complex tasks
• Each phase validated before execution begins
• No runtime dynamic replanning (design decision for determinism)
═════════════════════════════════════════════════════════════════════════════════════