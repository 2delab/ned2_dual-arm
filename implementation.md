# Chapter 4: Implementation

## 4.1 System Architecture Overview

The Multi-Arm Robotic System (MARS) implements a dual-arm coordination framework using two Niryo NED2 6-DOF manipulators operating within a shared workspace. The system is built on ROS2 (Humble/Jazzy compatible) and leverages MoveIt2 for collision-aware motion planning.

### 4.1.1 Hardware Configuration

The physical system consists of two Niryo NED2 robotic arms positioned 70 centimeters apart along the X-axis. Arm 1 is located at X=-0.35m and Arm 2 at X=+0.35m relative to the world frame origin. The arms are oriented to face each other with a 180-degree yaw difference, facilitating collaborative manipulation tasks within the shared central workspace.

Each manipulator provides 6 degrees of freedom and is equipped with gripper end-effectors capable of torque-controlled grasping operations. The arms connect to the control system via network interfaces, with Arm 1 at IP address 192.168.8.142 and Arm 2 at 192.168.8.149. This configuration creates an overlapping workspace envelope suitable for bimanual coordination tasks such as object handoff, synchronized assembly, and collaborative manipulation.

### 4.1.2 Software Architecture

The software architecture implements a centralized planning approach with distributed execution. The system consists of three primary layers:

**Layer 1: Hardware Interface**
Individual ROS2 driver nodes (`ros2_driver_arm_1`, `ros2_driver_arm_2`) communicate with the physical robots through the Niryo robot protocol. Each driver operates within its own namespace (`/arm_1`, `/arm_2`) to maintain logical isolation and prevent topic/service name collisions. The drivers publish unprefixed joint state messages (e.g., `joint_1`, `joint_2`, ..., `joint_6`) on their respective namespaced topics.

**Layer 2: State Aggregation & Coordination**
A custom Joint State Prefixer node aggregates joint states from both arms, adds namespace prefixes to joint names, and publishes a unified 12-dimensional state vector on the global `/joint_states` topic. This node also serves as a trajectory proxy, routing prefixed trajectory commands from MoveIt2 to the appropriate hardware controllers after stripping namespace prefixes. The implementation employs thread-safe mechanisms and a MultiThreadedExecutor with 8 threads to handle concurrent state updates and trajectory execution requests.

**Layer 3: Motion Planning**
A single MoveIt2 move_group node performs centralized motion planning for the complete 12-DOF system. The planning scene maintains awareness of all links across both manipulators, enabling collision-aware trajectory generation. The Robot State Publisher consumes the unified joint states and generates a complete TF tree representing the spatial relationships of all robot links in real-time.

### 4.1.3 Component Interaction Flow

The data flow through the system follows this sequence:

```
Physical Robots
    ↓ [Niryo Protocol]
ROS2 Drivers (Namespaced)
    ↓ [/arm_X/joint_states - unprefixed]
Joint State Prefixer
    ↓ [/joint_states - prefixed: arm_1_joint_1, arm_2_joint_1, ...]
Robot State Publisher
    ↓ [/tf - unified transform tree]
MoveIt2 Planning Scene
    ↓ [Collision-aware planning]
Trajectory Commands (prefixed)
    ↓ [/arm_X/follow_joint_trajectory_prefixed]
Joint State Prefixer (Proxy)
    ↓ [Strip prefixes, route to hardware]
Hardware Controllers
    ↓ [Position commands]
Physical Robots
```

This architecture ensures that MoveIt2 perceives a single unified robot model while maintaining the ability to execute trajectories on independent hardware controllers.

---

## 4.2 State Management & Synchronization

### 4.2.1 Joint State Aggregation

The Joint State Prefixer implements a thread-safe aggregation system that combines joint states from multiple robot namespaces into a unified state representation. The node subscribes to individual arm joint state topics and maintains an internal dictionary mapping namespaces to their most recent state messages.

**State Update Mechanism:**
```pseudocode
class JointStatePrefixer:
    joint_states: Dict[namespace → JointState]
    state_lock: Mutex
    
    function on_joint_state(msg, namespace):
        acquire(state_lock)
        joint_states[namespace] = msg
        release(state_lock)
    
    function publish_combined_states():
        acquire(state_lock)
        if any(joint_states.values() == None):
            release(state_lock)
            return  // Wait for all arms
        
        combined = JointState()
        combined.header.stamp = current_time()
        
        for namespace in sorted(joint_states.keys()):
            arm_state = joint_states[namespace]
            for joint_name in arm_state.name:
                combined.name.append(f"{namespace}_{joint_name}")
                combined.position.append(arm_state.position[i])
                combined.velocity.append(arm_state.velocity[i])
                combined.effort.append(arm_state.effort[i])
        
        publish(combined)
        release(state_lock)
```

The aggregation occurs at 40 Hz, providing real-time state updates to the planning system. The mutex-protected update mechanism ensures thread safety when handling concurrent state messages from multiple robot drivers.

### 4.2.2 Namespace Prefixing Strategy

The namespace prefixing approach solves the fundamental problem of integrating multiple robots into a single URDF model. Physical robot drivers publish joint states with generic names (`joint_1` through `joint_6`), while the unified URDF model defines distinct joints for each arm (`arm_1_joint_1`, `arm_1_joint_2`, ..., `arm_2_joint_1`, `arm_2_joint_2`, ..., `arm_2_joint_6`).

The prefixer performs bidirectional name translation:

**Forward Path (State Aggregation):**
- Input: `/arm_1/joint_states` with names `[joint_1, joint_2, ..., joint_6]`
- Transform: Prepend namespace → `[arm_1_joint_1, arm_1_joint_2, ..., arm_1_joint_6]`
- Output: Unified `/joint_states` with 12 joints

**Reverse Path (Trajectory Execution):**
- Input: `/arm_1/follow_joint_trajectory_prefixed` with names `[arm_1_joint_1, ...]`
- Transform: Strip prefix → `[joint_1, joint_2, ..., joint_6]`
- Output: `/arm_1/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory`

This bidirectional translation maintains compatibility between the unified planning representation and the physical hardware interface.

### 4.2.3 Trajectory Proxying Architecture

The trajectory proxying system enables MoveIt2 to command individual arms through a unified interface while maintaining compatibility with the hardware controllers. The system implements action servers that accept prefixed trajectory commands from MoveIt2 and forward them to hardware controllers after name translation.

**Proxy Implementation:**
```pseudocode
class TrajectoryProxy:
    servers: Dict[namespace → ActionServer]
    clients: Dict[namespace → ActionClient]
    
    function setup_proxy(namespace):
        // Server accepts prefixed trajectories from MoveIt
        servers[namespace] = ActionServer(
            topic=f"/{namespace}/follow_joint_trajectory_prefixed",
            execute_callback=lambda goal: execute_trajectory(goal, namespace)
        )
        
        // Client sends unprefixed trajectories to hardware
        clients[namespace] = ActionClient(
            topic=f"/{namespace}/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory"
        )
    
    function execute_trajectory(goal, namespace):
        // Strip namespace prefix from joint names
        unprefixed_trajectory = strip_prefix(goal.trajectory, namespace)
        
        // Forward to hardware controller
        hw_goal = FollowJointTrajectory.Goal()
        hw_goal.trajectory = unprefixed_trajectory
        
        future = clients[namespace].send_goal_async(hw_goal)
        
        // Relay feedback and result back to MoveIt
        while not future.done():
            feedback = await_feedback()
            send_feedback_to_moveit(feedback)
        
        return future.result()
    
    function strip_prefix(trajectory, namespace):
        prefix = f"{namespace}_"
        new_trajectory = JointTrajectory()
        new_trajectory.header = trajectory.header
        new_trajectory.points = trajectory.points
        
        for joint_name in trajectory.joint_names:
            if joint_name.startswith(prefix):
                new_trajectory.joint_names.append(
                    joint_name[len(prefix):]
                )
        
        return new_trajectory
```

The proxy system supports cancellation, feedback relay, and timeout handling (120-second default). This ensures robust execution while maintaining MoveIt2's expectation of unified robot control.

### 4.2.4 Concurrency Model

The Joint State Prefixer employs a MultiThreadedExecutor with 8 threads and two distinct callback groups:

**MutuallyExclusiveCallbackGroup**: Used for joint state subscriptions and the publishing timer. This ensures that state updates and publication occur atomically, preventing race conditions on the shared state dictionary.

**ReentrantCallbackGroup**: Used for action servers handling trajectory execution. This allows multiple trajectory execution requests to be processed concurrently, enabling parallel arm movement when using asynchronous execution modes.

This threading model provides the necessary concurrency for responsive state updates while preventing data races through appropriate callback group selection and mutex protection.

---

## 4.3 Collision Avoidance Implementation

### 4.3.1 Centralized Planning Scene

The collision avoidance strategy employs a single unified MoveIt2 planning scene that encompasses all links from both manipulators. This centralized approach treats the dual-arm system as a single 12-DOF robot, enabling the motion planner to reason about inter-arm collisions during trajectory generation.

The planning scene maintains collision geometries for all robot links and operates at 15 Hz to track the current system state. When planning trajectories, MoveIt2's collision checker validates each waypoint against the complete set of collision bodies, including:

- All links of Arm 1 (base, shoulder, arm, elbow, forearm, wrist, hand, gripper components)
- All links of Arm 2 (base, shoulder, arm, elbow, forearm, wrist, hand, gripper components)
- Static workspace obstacles (3m × 3m ground platform)
- Dynamic obstacles (can be added programmatically)

This unified representation ensures that any planned trajectory is collision-free with respect to both self-collision within each arm and inter-arm collisions.

### 4.3.2 Link Padding Strategy

To maintain a safety margin during execution, the system applies uniform link padding to critical components of both manipulators. Each of the following links receives 5 centimeters of padding:

**Arm 1**: base_link, shoulder_link, arm_link, elbow_link, forearm_link, wrist_link, hand_link
**Arm 2**: base_link, shoulder_link, arm_link, elbow_link, forearm_link, wrist_link, hand_link

When both arms include gripper assemblies (in the `niryo_ned2_dual_gripper_moveit_config` configuration), padding extends to:

**Arm 1 Gripper**: base_gripper_1, mors_1, mors_2, camera_link
**Arm 2 Gripper**: base_gripper_1, mors_1, mors_2, camera_link

The 5-centimeter per-link padding creates a minimum 10-centimeter separation buffer between arms when collision checking considers two padded links approaching each other (5cm from each link). This conservative safety margin accounts for:

- Execution tracking errors
- Model inaccuracies
- Dynamic effects during motion
- Hardware calibration tolerances

The padding is configured in the SRDF (Semantic Robot Description Format) file and is applied uniformly during all planning operations.

### 4.3.3 Collision Matrix Optimization

While the unified planning scene enables comprehensive collision checking, naive pairwise collision checking between all link combinations would be computationally expensive (O(n²) for n links). The system employs a disabled collision pairs matrix to optimize collision checking performance.

The SRDF defines 143+ collision pairs that are explicitly disabled based on three categories:

**Adjacent Links**: Parent-child links connected by joints physically cannot collide due to joint constraints. Examples:
- `arm_1_shoulder_link` ↔ `arm_1_base_link` (connected by joint_1)
- `arm_1_arm_link` ↔ `arm_1_shoulder_link` (connected by joint_2)

**Geometrically Impossible Collisions**: Links that are never close enough to collide regardless of joint configuration:
- `arm_1_base_link` ↔ `arm_1_elbow_link` (too far apart in kinematic chain)
- `arm_1_arm_link` ↔ `arm_1_hand_link` (separated by multiple joints)

**Inter-Arm Base Links**: The arm base links are fixed 70cm apart and flagged as adjacent in the URDF hierarchy:
- `arm_1_base_link` ↔ `arm_2_base_link`

This collision matrix reduces collision checking overhead while maintaining safety guarantees. Only geometrically plausible collision pairs are actively checked during planning.

### 4.3.4 Collision Checking During Planning

MoveIt2 integrates collision checking at multiple stages of the planning pipeline:

**Start State Validation**: Before planning begins, the system verifies that the current robot state is collision-free. This prevents planning from invalid initial configurations.

**Goal State Validation**: Target configurations are checked for collisions before planning attempts. Invalid goals are rejected immediately rather than attempting impossible plans.

**Trajectory Validation**: During planning, each candidate trajectory waypoint undergoes collision checking. Sampling-based planners (OMPL) generate random configurations and validate them before expanding the search tree. Only collision-free configurations are added to the exploration graph.

**Path Simplification**: After finding an initial collision-free path, MoveIt2's path smoothing algorithms attempt to shorten and optimize the trajectory while maintaining collision-free properties through continuous re-validation.

This multi-stage validation ensures that executed trajectories maintain safety throughout their duration.

### 4.3.5 Phase-Based Execution for Complex Scenarios

For scenarios involving narrow passages or complex maneuvers, the system employs a phase-based execution strategy rather than dynamic replanning. Complex tasks are decomposed into sequential phases where each phase represents a collision-free segment:

```pseudocode
function execute_complex_task():
    phases = [
        Phase1: Move both arms to intermediate poses
        Phase2: One arm reaches through shared space
        Phase3: Other arm moves to final position
        Phase4: Both return to home
    ]
    
    for phase in phases:
        plan = moveit.plan(
            start=current_state,
            goal=phase.target_state,
            group="dual"
        )
        
        if not plan.is_collision_free():
            return ERROR
        
        execute(plan)
        wait_for_completion()
    
    return SUCCESS
```

This approach ensures that each segment is validated for collision-free execution before proceeding, eliminating the need for real-time obstacle avoidance during trajectory execution. Test files such as `test6.py` demonstrate this multi-phase coordination pattern.

---

## 4.4 Multi-Mode Coordination Framework

The system implements three distinct coordination modes that balance between motion synchronization requirements and execution efficiency. Each mode addresses different classes of bimanual manipulation tasks.

### 4.4.1 Synchronous Mode: Dual Group Planning

**Purpose**: Fully coordinated bimanual tasks requiring precise temporal and spatial synchronization.

**Architecture**:
Synchronous mode treats both arms as a single 12-degree-of-freedom system during planning. The MoveIt2 planning component uses the `dual` planning group, which encompasses all joints from both manipulators in its configuration space.

**Planning Process**:
```pseudocode
function plan_synchronized_motion(target_state_12dof):
    dual_arm = moveit.get_planning_component("dual")
    dual_arm.set_start_state_to_current_state()
    
    // Create unified robot state with all 12 joint positions
    robot_state = RobotState(robot_model)
    robot_state.joint_positions = {
        "arm_1_joint_1": target[0],
        "arm_1_joint_2": target[1],
        "arm_1_joint_3": target[2],
        "arm_1_joint_4": target[3],
        "arm_1_joint_5": target[4],
        "arm_1_joint_6": target[5],
        "arm_2_joint_1": target[6],
        "arm_2_joint_2": target[7],
        "arm_2_joint_3": target[8],
        "arm_2_joint_4": target[9],
        "arm_2_joint_5": target[10],
        "arm_2_joint_6": target[11]
    }
    
    dual_arm.set_goal_state(robot_state=robot_state)
    
    // Plan in unified 12-DOF configuration space
    plan_result = dual_arm.plan()
    
    return plan_result
```

**Execution**:
The resulting trajectory contains waypoints for all 12 joints with synchronized timing. MoveIt2's trajectory execution manager routes the trajectory to the appropriate controllers:

```pseudocode
function execute_synchronized(trajectory):
    // MoveIt automatically splits trajectory to controllers
    moveit.execute(trajectory, controllers=[])
    
    // Joint State Prefixer proxies to hardware:
    // - arm_1_joint_* → /arm_1/controller
    // - arm_2_joint_* → /arm_2/controller
```

**Characteristics**:
- **Collision Awareness**: Planner considers inter-arm collision throughout entire trajectory
- **Temporal Harmony**: Both arms follow time-parameterized waypoints simultaneously
- **Coordinated Motion**: Arms maintain spatial relationships throughout execution
- **Planning Complexity**: Higher dimensional search space (12-DOF vs 6-DOF)

**Use Cases**:
- Bimanual object manipulation (both hands grasping same object)
- Synchronized assembly operations
- Coordinated Cartesian motions (both end-effectors moving in formation)

**Implementation Evidence**: Files `moveit4.py`, `moveit5.py`, `moveit6.py`, `test5.py` demonstrate dual group planning with various goal configurations.

### 4.4.2 Hybrid Mode: Coordinated Planning with Parallel Execution

**Purpose**: Collision-aware coordination without strict temporal synchronization requirements.

**Architecture**:
Hybrid mode performs unified planning using the `dual` group but executes trajectories independently on separate controllers. This provides collision-free guarantees during planning while allowing execution flexibility.

**Planning Phase**:
```pseudocode
function plan_coordinated_phase(targets_arm1, targets_arm2):
    // Plan with dual group for collision awareness
    goal = create_dual_goal(targets_arm1, targets_arm2)
    goal.group_name = "dual"
    
    move_group_client.send_goal_async(goal)
    trajectory = await_result()
    
    return trajectory  // Contains all 12 joints
```

**Trajectory Splitting**:
After planning, the unified trajectory is decomposed into arm-specific trajectories:

```pseudocode
function split_trajectory(unified_trajectory, namespace):
    arm_trajectory = JointTrajectory()
    
    // Extract joints belonging to this arm
    for joint_name in unified_trajectory.joint_names:
        if joint_name.startswith(f"{namespace}_"):
            arm_trajectory.joint_names.append(joint_name)
    
    // Copy waypoints for relevant joints
    for point in unified_trajectory.points:
        arm_point = JointTrajectoryPoint()
        for idx in arm_joint_indices:
            arm_point.positions.append(point.positions[idx])
            arm_point.velocities.append(point.velocities[idx])
        arm_point.time_from_start = point.time_from_start
        arm_trajectory.points.append(arm_point)
    
    return arm_trajectory
```

**Parallel Execution**:
```pseudocode
function execute_parallel(trajectory):
    traj_arm1 = split_trajectory(trajectory, "arm_1")
    traj_arm2 = split_trajectory(trajectory, "arm_2")
    
    // Queue both trajectories simultaneously
    client_arm1.send_goal_async(traj_arm1)  // Non-blocking
    client_arm2.send_goal_async(traj_arm2)  // Non-blocking
    
    // Controllers execute independently
```

**Characteristics**:
- **Collision-Free Planning**: Unified planning ensures safe trajectories
- **Independent Execution**: Arms follow trajectories without inter-controller synchronization
- **Flexible Timing**: Minor execution delays don't cascade between arms
- **Optimal Planning**: Single planning operation generates coordinated motion

**Use Cases**:
- Multi-phase assembly where phases are collision-free but don't require microsecond synchronization
- Pick-and-place operations in shared workspace
- Sequential handoff preparation and execution

**Implementation Evidence**: Files `test9.py` and `test10.py` implement this pattern with explicit trajectory splitting and parallel execution queuing.

### 4.4.3 Asynchronous Mode: Independent Operations

**Purpose**: Maximize throughput for independent tasks with different durations and no spatial interaction.

**Architecture**:
Asynchronous mode plans and executes each arm completely independently. Arms operate in separate configuration spaces with no shared planning or execution coordination.

**Independent Planning**:
```pseudocode
function plan_independent(arm_id, targets):
    arm = moveit.get_planning_component(arm_id)
    arm.set_start_state_to_current_state()
    
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions(arm_id, targets)
    arm.set_goal_state(robot_state=robot_state)
    
    // Plan in 6-DOF space (single arm only)
    plan_result = arm.plan()
    
    return plan_result
```

**Parallel Execution via AsyncExecutor**:
The system provides an `AsyncDualArmExecutor` library that manages concurrent trajectory execution:

```pseudocode
class AsyncDualArmExecutor:
    action_clients: Dict[arm_id → ActionClient]
    active_executions: Dict[arm_id → ExecutionState]
    callback_group: ReentrantCallbackGroup
    
    function execute_async(arm_id, trajectory, callback):
        // Apply velocity scaling if specified
        scaled_traj = apply_scaling(
            trajectory,
            velocity_factor=0.2,    // 20% speed
            accel_factor=0.2
        )
        
        // Send goal without blocking
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = scaled_traj
        
        future = action_clients[arm_id].send_goal_async(goal)
        
        // Register callback for completion
        future.add_done_callback(
            lambda f: handle_completion(arm_id, f, callback)
        )
        
        active_executions[arm_id] = ExecutionState.RUNNING
    
    function handle_completion(arm_id, future, user_callback):
        result = future.result()
        success = (result.error_code == SUCCESS)
        
        active_executions[arm_id] = ExecutionState.COMPLETE
        
        if user_callback:
            user_callback(arm_id, success, result.error_string)
```

**Characteristics**:
- **Maximum Throughput**: No waiting for slower arm to complete
- **Collision Oblivious**: Each arm plans without considering the other
- **Simple Planning**: Lower dimensional search space (6-DOF per arm)
- **Independent Timing**: Different task durations don't affect each other

**Safety Considerations**:
Asynchronous mode requires careful task design to avoid collisions. The workspace should be logically partitioned or tasks should be temporally separated to ensure arms don't interfere:

```pseudocode
function safe_async_execution():
    // Ensure spatial separation
    if not workspaces_separated(task1, task2):
        return ERROR
    
    // Execute independently
    executor.execute_async("arm_1", trajectory1, callback1)
    executor.execute_async("arm_2", trajectory2, callback2)
```

**Use Cases**:
- Parallel pick-and-place in separate workspace zones
- One arm performing task while other is idle/resetting
- High-throughput operations with pre-validated safe zones

**Implementation Evidence**: Files `test_async1.py` and `test_async2.py` demonstrate choreographed parallel execution with callback-based completion handling. The `async_executor_lib.py` module provides the infrastructure for this mode.

### 4.4.4 Mode Selection Guidelines

The appropriate coordination mode depends on task requirements:

| **Task Requirement** | **Mode** | **Rationale** |
|---------------------|----------|---------------|
| Both arms grasp same object | Synchronous | Requires precise spatial/temporal coordination |
| Sequential handoff in shared space | Hybrid | Need collision-free paths, loose timing |
| Parallel pick-and-place in separate zones | Asynchronous | Maximize throughput, no interaction |
| Coordinated assembly with phases | Hybrid | Phase-based collision avoidance sufficient |
| Bimanual Cartesian coordination | Synchronous | End-effector pose relationships critical |

The test suite includes examples of each mode across different scenarios, validating the flexibility of the multi-mode architecture.

---

## 4.5 Planning & Trajectory Execution

### 4.5.1 OMPL Planning Integration

The system employs the Open Motion Planning Library (OMPL) as the primary motion planning backend. OMPL provides a collection of sampling-based planning algorithms suitable for high-dimensional configuration spaces like the 12-DOF dual-arm system.

**Planning Pipeline Configuration**:
The planning pipeline consists of request adapters (pre-processing), the core planner, and response adapters (post-processing):

```pseudocode
Planning Request
    ↓
[Request Adapters]
    - ResolveConstraintFrames: Transform constraints to planning frame
    - ValidateWorkspaceBounds: Ensure goals within reachable space
    - CheckStartStateBounds: Verify start state within joint limits
    - CheckStartStateCollision: Validate collision-free initial state
    ↓
[OMPL Planner]
    - Configuration space sampling
    - Collision checking at sampled states
    - Graph/tree expansion until goal connection
    ↓
[Response Adapters]
    - AddTimeOptimalParameterization: Compute velocities/accelerations
    - ValidateSolution: Verify trajectory safety
    - DisplayMotionPath: Publish visualization markers
    ↓
Executable Trajectory
```

**Available Planners**:
The system supports multiple OMPL planners with different performance characteristics:

- **RRTConnect** (default): Bidirectional rapidly-exploring random tree, fast convergence for most scenarios
- **RRT**: Single-tree exploration, useful for constrained spaces
- **RRTstar**: Asymptotically optimal variant, longer planning time but better paths
- **PRM/PRMstar**: Probabilistic roadmap methods, efficient for repeated queries
- **KPIECE/BKPIECE**: Discretization-based planners, good for narrow passages

**Planning Parameters**:
Key planning parameters are configured to balance solution quality and computation time:

```pseudocode
planning_parameters = {
    max_planning_time: 5.0 seconds,
    planning_attempts: 10,
    goal_tolerance: {
        position: 0.0001 meters,
        orientation: 0.01 radians
    },
    velocity_scaling: 0.3,      // 30% of maximum velocity
    acceleration_scaling: 0.3,   // 30% of maximum acceleration
    planner_id: "RRTConnect"
}
```

The conservative velocity and acceleration scaling factors (30%) provide safety margins for execution on physical hardware.

### 4.5.2 Inverse Kinematics Solution

For Cartesian goal specifications (pose-based planning), the system requires inverse kinematics (IK) solutions to convert end-effector poses into joint configurations.

**KDL Kinematics Plugin**:
The system uses the Kinematics and Dynamics Library (KDL) plugin for IK solving:

```pseudocode
kinematics_config = {
    solver: "kdl_kinematics_plugin/KDLKinematicsPlugin",
    search_resolution: 0.005 radians,
    timeout: 0.005 seconds,     // 5 milliseconds
    attempts: 3
}
```

**IK Solving Process**:
```pseudocode
function solve_ik_for_pose(target_pose, arm_id):
    // Get current joint positions as seed
    seed_state = get_current_joint_positions(arm_id)
    
    // Iterative IK solution using Jacobian methods
    for attempt in 1..max_attempts:
        joint_solution = kdl_solver.solve(
            target_pose=target_pose,
            seed_state=seed_state,
            timeout=5ms
        )
        
        if joint_solution.valid:
            // Verify solution is collision-free
            if not in_collision(joint_solution):
                return joint_solution
        
        // Try different seed if solution invalid or in collision
        seed_state = perturb_seed(seed_state)
    
    return NO_SOLUTION
```

The 5-millisecond timeout ensures real-time performance, allowing rapid IK queries during planning without blocking the system.

### 4.5.3 Trajectory Time Parameterization

After finding a geometric path (sequence of collision-free waypoints), the system computes a time parameterization that assigns velocities and accelerations to each waypoint while respecting joint limits.

**Time-Optimal Parameterization**:
```pseudocode
function add_time_optimal_parameterization(path):
    trajectory = JointTrajectory()
    
    for i in range(len(path.waypoints)):
        point = JointTrajectoryPoint()
        point.positions = path.waypoints[i]
        
        if i == 0:
            point.velocities = [0.0] * num_joints
            point.accelerations = [0.0] * num_joints
            point.time_from_start = 0.0
        else:
            // Compute maximum safe velocity for segment
            max_vel = min(
                compute_velocity_limit(path.waypoints[i-1], path.waypoints[i]),
                joint_velocity_limits * velocity_scaling_factor
            )
            
            // Compute acceleration satisfying dynamic constraints
            max_accel = joint_acceleration_limits * acceleration_scaling_factor
            
            // Time parameterization using trapezoidal profile
            segment_time = compute_segment_time(
                distance=distance(path.waypoints[i-1], path.waypoints[i]),
                max_velocity=max_vel,
                max_acceleration=max_accel
            )
            
            point.time_from_start = trajectory.points[i-1].time_from_start + segment_time
            point.velocities = compute_velocities(path.waypoints[i-1], path.waypoints[i], segment_time)
            point.accelerations = compute_accelerations(point.velocities, segment_time)
        
        trajectory.points.append(point)
    
    return trajectory
```

This approach generates smooth trajectories that minimize execution time while respecting all kinematic and dynamic constraints.

### 4.5.4 Controller Architecture

The execution layer implements separate JointTrajectoryController instances for each arm, enabling independent trajectory following.

**Controller Configuration**:
```yaml
arm_1_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - arm_1_joint_1
    - arm_1_joint_2
    - arm_1_joint_3
    - arm_1_joint_4
    - arm_1_joint_5
    - arm_1_joint_6
  command_interfaces:
    - position
  state_interfaces:
    - position
    - velocity
  update_rate: 100  # Hz

arm_2_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - arm_2_joint_1
    - arm_2_joint_2
    - arm_2_joint_3
    - arm_2_joint_4
    - arm_2_joint_5
    - arm_2_joint_6
  command_interfaces:
    - position
  state_interfaces:
    - position
    - velocity
  update_rate: 100  # Hz

dual_controller:
  type: joint_trajectory_controller/JointTrajectoryController
  joints:
    - arm_1_joint_1
    - arm_1_joint_2
    - arm_1_joint_3
    - arm_1_joint_4
    - arm_1_joint_5
    - arm_1_joint_6
    - arm_2_joint_1
    - arm_2_joint_2
    - arm_2_joint_3
    - arm_2_joint_4
    - arm_2_joint_5
    - arm_2_joint_6
  command_interfaces:
    - position
  state_interfaces:
    - position
    - velocity
  update_rate: 100  # Hz
```

The 100 Hz update rate provides smooth trajectory following on the physical hardware.

**Controller Selection**:
MoveIt2's SimpleControllerManager maps planning groups to controllers:

```pseudocode
controller_mapping = {
    "arm_1": "arm_1_controller",
    "arm_2": "arm_2_controller",
    "dual": ["arm_1_controller", "arm_2_controller"]  // Parallel execution
}
```

When executing a dual-group trajectory, MoveIt2 automatically splits the trajectory and sends appropriate segments to each controller.

### 4.5.5 Trajectory Execution Flow

The complete execution process from planned trajectory to hardware motion:

```pseudocode
function execute_trajectory(planned_trajectory):
    // MoveIt sends trajectory to controller manager
    execution_future = moveit.execute_async(
        planned_trajectory,
        controllers=[]  // Auto-select based on group
    )
    
    // Controller manager routes trajectory
    if planned_trajectory.group == "dual":
        arm1_traj = extract_joints(planned_trajectory, "arm_1_*")
        arm2_traj = extract_joints(planned_trajectory, "arm_2_*")
        
        send_to_controller("/arm_1/follow_joint_trajectory_prefixed", arm1_traj)
        send_to_controller("/arm_2/follow_joint_trajectory_prefixed", arm2_traj)
    else:
        send_to_controller(f"/{group}/follow_joint_trajectory_prefixed", planned_trajectory)
    
    // Joint State Prefixer proxies to hardware
    for trajectory in trajectories:
        unprefixed = strip_namespace_prefix(trajectory)
        forward_to_hardware(unprefixed)
    
    // Hardware controllers execute
    for t in range(trajectory.duration):
        current_time = t * control_period
        
        // Interpolate target position at current time
        target = interpolate_trajectory(trajectory, current_time)
        
        // Send position command to hardware
        send_position_command(target.positions)
        
        // Read current state
        current_state = read_joint_states()
        
        // Publish state back to ROS
        publish_joint_states(current_state)
        
        sleep(control_period)
    
    return SUCCESS
```

**Error Handling**:
The execution system monitors for errors and handles them appropriately:

```pseudocode
function monitor_execution(execution_future):
    while not execution_future.done():
        feedback = execution_future.feedback()
        
        // Check for trajectory deviation
        if feedback.error.position > position_tolerance:
            execution_future.cancel()
            return EXECUTION_ERROR
        
        // Check for timeout
        if elapsed_time > max_execution_time:
            execution_future.cancel()
            return TIMEOUT_ERROR
        
        sleep(feedback_period)
    
    result = execution_future.result()
    return result.error_code
```

The 120-second execution timeout prevents indefinite blocking on stalled trajectories.

---

## 4.6 End-Effector Control

### 4.6.1 Cartesian Pose Planning

The system supports Cartesian goal specification for precise end-effector positioning tasks. Unlike joint-space goals that specify target joint angles directly, Cartesian goals specify target poses (position + orientation) in task space.

**Single-Arm Cartesian Planning**:
```pseudocode
function plan_cartesian_goal(arm_id, target_pose):
    arm = moveit.get_planning_component(arm_id)
    arm.set_start_state_to_current_state()
    
    // Create pose goal message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = f"{arm_id}_base_link"
    pose_goal.pose.position.x = target_pose.x
    pose_goal.pose.position.y = target_pose.y
    pose_goal.pose.position.z = target_pose.z
    pose_goal.pose.orientation.x = target_pose.qx
    pose_goal.pose.orientation.y = target_pose.qy
    pose_goal.pose.orientation.z = target_pose.qz
    pose_goal.pose.orientation.w = target_pose.qw
    
    // Set goal for end-effector link
    arm.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link=f"{arm_id}_tool_link"
    )
    
    // Plan - MoveIt solves IK internally
    plan_result = arm.plan()
    
    return plan_result
```

**Dual-Arm Cartesian Planning**:
For bimanual Cartesian coordination, both arms must reach specified end-effector poses simultaneously. The system validates this capability through the dual planning group:

```pseudocode
function plan_dual_cartesian(pose_arm1, pose_arm2):
    dual = moveit.get_planning_component("dual")
    dual.set_start_state_to_current_state()
    
    // Solve IK for both target poses
    robot_state = RobotState(robot_model)
    
    success_arm1 = robot_state.set_from_ik(
        group="arm_1",
        target_pose=pose_arm1,
        end_effector_link="arm_1_tool_link"
    )
    
    success_arm2 = robot_state.set_from_ik(
        group="arm_2",
        target_pose=pose_arm2,
        end_effector_link="arm_2_tool_link"
    )
    
    if not (success_arm1 and success_arm2):
        return NO_IK_SOLUTION
    
    // Set combined goal state
    dual.set_goal_state(robot_state=robot_state)
    
    // Plan with collision awareness for both arms
    plan_result = dual.plan()
    
    return plan_result
```

This approach ensures that both arms reach their target poses while maintaining collision-free motion throughout the trajectory.

### 4.6.2 Gripper Coordination

The system integrates gripper control through the Niryo robot tool interface, enabling synchronized grasping operations with the motion planning system.

**Gripper Command Interface**:
```pseudocode
function control_gripper(arm_id, command_type, torque_percent):
    // Create action client for tool commands
    tool_client = ActionClient(
        node,
        Tool,
        f"/{arm_id}/niryo_robot_tools_commander/action_server"
    )
    
    // Construct gripper command
    tool_cmd = ToolCommand()
    tool_cmd.cmd_type = command_type  // 1=OPEN, 2=CLOSE
    tool_cmd.tool_id = 11              // Gripper1
    tool_cmd.max_torque_percentage = torque_percent
    tool_cmd.hold_torque_percentage = torque_percent * 0.4
    
    goal = Tool.Goal()
    goal.cmd = tool_cmd
    
    // Send command asynchronously
    future = tool_client.send_goal_async(goal)
    
    return future
```

**Integrated Pick-and-Place Sequence**:
Combining motion planning with gripper control enables complete manipulation tasks:

```pseudocode
function pick_and_place(arm_id, pickup_pose, place_pose):
    // Phase 1: Approach pickup location
    plan1 = plan_cartesian_goal(arm_id, pickup_pose)
    execute(plan1)
    
    // Phase 2: Close gripper to grasp object
    control_gripper(arm_id, CLOSE, torque=80)
    wait(3.0)  // Allow gripper to fully close
    
    // Phase 3: Move to place location
    plan2 = plan_cartesian_goal(arm_id, place_pose)
    execute(plan2)
    
    // Phase 4: Open gripper to release object
    control_gripper(arm_id, OPEN, torque=80)
    wait(2.0)  // Allow gripper to fully open
```

**Dual-Arm Synchronized Grasping**:
For bimanual grasping tasks, both grippers must coordinate their actions:

```pseudocode
function dual_grasp(object_pose, grasp_offset):
    // Compute symmetric grasp poses
    pose_left = object_pose + translation(0, +grasp_offset, 0)
    pose_right = object_pose + translation(0, -grasp_offset, 0)
    
    // Plan synchronized approach
    plan = plan_dual_cartesian(pose_left, pose_right)
    execute(plan)
    
    // Close both grippers simultaneously
    future1 = control_gripper("arm_1", CLOSE, torque=80)
    future2 = control_gripper("arm_2", CLOSE, torque=80)
    
    wait_for_both(future1, future2)
```

This ensures both grippers close on the object simultaneously, providing stable bimanual grasping.

### 4.6.3 Frame Transformations

Cartesian planning requires proper frame transformations between different coordinate systems in the workspace.

**Reference Frames**:
- **World Frame**: Fixed reference at workspace origin (0, 0, 0)
- **Arm Base Frames**: `arm_1_base_link` at (-0.35, 0, 0), `arm_2_base_link` at (+0.35, 0, 0)
- **Tool Frames**: `arm_1_tool_link`, `arm_2_tool_link` at end-effector positions

**Transformation Example**:
When specifying a target pose in world frame that must be planned in arm base frame:

```pseudocode
function transform_pose(pose_world, target_frame):
    // Query TF tree for transformation
    transform = tf_buffer.lookup_transform(
        target_frame="arm_1_base_link",
        source_frame="world",
        time=current_time
    )
    
    // Apply transformation to pose
    pose_arm_frame = transform * pose_world
    
    return pose_arm_frame
```

The Robot State Publisher maintains the complete TF tree, enabling automatic frame transformations during planning.

### 4.6.4 Validation Through Demonstration Tasks

The implementation includes demonstration scripts that validate end-effector control capabilities:

**Demo 1** (`demo1.py`): Single-arm Cartesian motion with gripper coordination
- Move to pickup pose (0.35, 0.2, 0.2) in arm_1_base_link frame
- Open gripper
- Move to place pose (0.35, -0.2, 0.2)
- Close gripper

**Demo 2** (`demo2.py`): Multi-object pick-and-place loop
- Iterate through multiple pickup locations
- Execute coordinated Cartesian motion + gripper commands
- Stack objects at incrementing heights

These demonstrations confirm that single-arm Cartesian planning integrates correctly with the physical hardware and gripper control system. The validated dual-arm Cartesian coordination extends this capability to bimanual scenarios, enabling tasks such as:

- Synchronized bimanual grasping
- Object handoff between arms
- Cooperative assembly operations
- Coordinated Cartesian path following

---

## 4.7 Configuration & Deployment

### 4.7.1 MoveIt Configuration Structure

The system employs two primary MoveIt configuration packages optimized for different scenarios:

**niryo_ned2_dual_arm_moveit_config**: Basic dual-arm configuration without gripper models, suitable for motion planning validation and testing without end-effector complexity.

**niryo_ned2_dual_gripper_moveit_config**: Complete configuration including gripper collision geometries, camera links, and additional constraints. This configuration is used for manipulation tasks requiring gripper coordination.

**Configuration File Hierarchy**:
```
config/
├── niryo_ned2_dual.srdf              # Robot semantics
├── niryo_ned2_dual.urdf.xacro        # Robot kinematics model
├── joint_limits.yaml                  # Velocity/acceleration limits
├── kinematics.yaml                    # IK solver configuration
├── moveit_controllers.yaml            # MoveIt-controller mapping
├── ros2_controllers.yaml              # Hardware controller specs
├── ompl_planning_pipeline.yaml        # OMPL planner parameters
├── moveit_py_params.yaml             # Python API configuration
└── workspace_scene.yaml               # Static collision objects
```

### 4.7.2 SRDF Semantic Description

The Semantic Robot Description Format (SRDF) file defines planning groups, collision behavior, and predefined states:

**Planning Group Definitions**:
```xml
<group name="arm_1">
    <joint name="arm_1_joint_world"/>
    <joint name="arm_1_joint_1"/>
    <joint name="arm_1_joint_2"/>
    <joint name="arm_1_joint_3"/>
    <joint name="arm_1_joint_4"/>
    <joint name="arm_1_joint_5"/>
    <joint name="arm_1_joint_6"/>
</group>

<group name="arm_2">
    <joint name="arm_2_joint_world"/>
    <joint name="arm_2_joint_1"/>
    <joint name="arm_2_joint_2"/>
    <joint name="arm_2_joint_3"/>
    <joint name="arm_2_joint_4"/>
    <joint name="arm_2_joint_5"/>
    <joint name="arm_2_joint_6"/>
</group>

<group name="dual">
    <group name="arm_1"/>
    <group name="arm_2"/>
</group>
```

The `dual` group is defined as a composite of both arm groups, enabling unified 12-DOF planning.

**Predefined Poses**:
```xml
<group_state name="home" group="arm_1">
    <joint name="arm_1_joint_1" value="0"/>
    <joint name="arm_1_joint_2" value="0.2862"/>
    <joint name="arm_1_joint_3" value="0"/>
    <joint name="arm_1_joint_4" value="0"/>
    <joint name="arm_1_joint_5" value="-1.5166"/>
    <joint name="arm_1_joint_6" value="0"/>
</group_state>

<group_state name="test+pose" group="dual">
    <joint name="arm_1_joint_1" value="-1.2099"/>
    <!-- ... additional joints ... -->
    <joint name="arm_2_joint_6" value="0"/>
</group_state>
```

These predefined poses enable rapid testing and provide safe initialization configurations.

**Collision Configuration**:
Link padding and disabled collision pairs are specified as described in Section 4.3.

### 4.7.3 Joint Limits Configuration

Conservative joint limits ensure safe operation on physical hardware:

```yaml
joint_limits:
  arm_1_joint_1:
    has_velocity_limits: true
    max_velocity: 1.5708  # rad/s
    has_acceleration_limits: true
    max_acceleration: 2.0  # rad/s²
  arm_1_joint_2:
    has_velocity_limits: true
    max_velocity: 1.0472
    has_acceleration_limits: true
    max_acceleration: 0.8
  # ... additional joints ...

default_velocity_scaling_factor: 0.3
default_acceleration_scaling_factor: 0.3
```

The 30% scaling factors provide safety margins while maintaining reasonable motion speeds for manipulation tasks.

### 4.7.4 Launch File Architecture

The system employs a modular launch file structure:

**Main Launch File** (`ned2_dual_arm_moveit.launch.py`):
```pseudocode
function generate_launch_description():
    nodes = []
    
    // 1. Joint State Prefixer
    nodes.append(Node(
        package='niryo_ned2_dual_arm_moveit_config',
        executable='joint_state_prefixer.py',
        parameters=[{
            'robot_namespaces': ['arm_1', 'arm_2'],
            'publish_frequency': 40.0,
            'trajectory_timeout_sec': 120.0
        }]
    ))
    
    // 2. Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    ))
    
    // 3. MoveIt Move Group
    nodes.append(Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            moveit_config.to_dict(),
            {'publish_planning_scene_hz': 15.0}
        ]
    ))
    
    // 4. Workspace Scene Loader (optional)
    nodes.append(Node(
        package='niryo_ned2_dual_arm_moveit_config',
        executable='load_workspace_scene.py',
        parameters=[{'scene_file': workspace_scene_path}]
    ))
    
    // 5. RViz Visualization
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    ))
    
    return LaunchDescription(nodes)
```

This architecture ensures all components start in the correct order with appropriate dependencies.

### 4.7.5 Parameter Tuning Decisions

Several key parameters were tuned through empirical testing:

**State Publishing Frequency (40 Hz)**:
Selected to balance responsiveness and computational load. Higher frequencies (>50 Hz) showed diminishing returns while increasing CPU usage. Lower frequencies (<30 Hz) introduced noticeable lag in RViz visualization.

**Velocity/Acceleration Scaling (30%)**:
Conservative scaling ensures safe operation on physical hardware with modeling uncertainties. Test trajectories at 100% scaling exhibited occasional overshooting and oscillation, particularly on joint_5 and joint_6.

**Planning Timeout (5 seconds)**:
Sufficient for 95%+ of planning queries in typical scenarios. Complex narrow-passage scenarios occasionally require longer timeouts but are handled through multi-phase decomposition.

**Link Padding (5 cm)**:
Provides adequate safety margin given:
- Model accuracy: ±2mm typical error
- Execution tracking: ±5mm typical deviation
- Calibration drift: ±3mm over operational period
- Safety factor: 2× minimum required clearance

**Trajectory Timeout (120 seconds)**:
Accommodates slow velocities (20% scaling) over long paths. Maximum observed trajectory duration in testing was 85 seconds for complex multi-phase operations.

### 4.7.6 Deployment Workflow

The typical deployment sequence for physical robot operation:

```pseudocode
function deploy_system():
    // 1. Start hardware drivers
    launch("niryo_ned_ros2_driver", namespace="arm_1", ip="192.168.8.142")
    launch("niryo_ned_ros2_driver", namespace="arm_2", ip="192.168.8.149")
    wait_for_drivers_ready()
    
    // 2. Start MoveIt system
    launch("ned2_dual_arm_moveit.launch.py")
    wait_for_planning_scene_ready()
    
    // 3. Verify system state
    verify_joint_states_publishing()
    verify_tf_tree_complete()
    verify_move_group_responding()
    
    // 4. Move to known safe configuration
    plan_to_home_state("dual")
    execute_trajectory()
    
    // 5. System ready for task execution
    return READY
```

This workflow ensures all components are operational before task execution begins.

---

## 4.8 Implementation Summary

The implemented Multi-Arm Robotic System demonstrates a comprehensive solution to the challenges outlined in the theoretical framework:

**Shared Workspace Management**: Achieved through centralized MoveIt planning with link padding and optimized collision matrices, ensuring collision-free coordination of both manipulators in overlapping workspace.

**Task Coordination**: Implemented via three distinct modes (synchronous, hybrid, asynchronous) providing flexibility for different task requirements while maintaining safety guarantees.

**State Management**: Robust 12-DOF state aggregation with thread-safe mechanisms and trajectory proxying enables seamless integration of namespace-isolated hardware drivers with unified planning representation.

**Planning & Execution**: OMPL integration with time-optimal trajectory generation and separate controllers per arm balances planning quality with execution flexibility.

**End-Effector Control**: Validated Cartesian planning capabilities with gripper coordination enable complex manipulation tasks including bimanual grasping and pick-and-place operations.

The system has been validated through extensive testing across 15+ test scenarios demonstrating joint-space coordination, multi-phase operations, parallel execution, and collision avoidance. The implementation provides a solid foundation for empirical validation of bimanual Cartesian coordination tasks in Chapter 5.
