# Chapter 3: System Design

## 3.1 Design Overview

This chapter presents the architectural design of the Multi-Arm Robotic System (MARS), detailing the design decisions, rationale, and trade-offs that shaped the implementation. While Chapter 4 describes *how* the system is implemented through algorithms and technical mechanisms, this chapter focuses on *what* components comprise the system and *why* specific architectural choices were made.

The design process was guided by the four fundamental challenges identified in Chapter 1: shared workspace management, task coordination and synchronization, joint state management, and planning with trajectory execution. Each challenge influenced multiple aspects of the architecture, requiring careful consideration of component interactions, data flows, and coordination mechanisms.

### 3.1.1 Design Philosophy

The architectural design of MARS is founded on several core principles that guided decision-making throughout the development process:

**Flexibility Through Modularity**: Rather than optimizing for a single coordination paradigm, MARS provides multiple operational modes that can be selected based on task requirements. This flexibility is achieved through modular component design where coordination strategies are encapsulated and interchangeable.

**Centralized Planning with Distributed Execution**: The system employs a unified planning scene that reasons about all manipulators simultaneously while maintaining the ability to execute trajectories on independent hardware controllers. This hybrid approach combines the collision awareness benefits of centralized planning with the execution flexibility of distributed control.

**Separation of Concerns**: Clear boundaries between hardware interface, state management, planning, and execution layers enable each component to focus on its specific responsibilities without tight coupling to other subsystems. This separation facilitates independent testing, modification, and extension of individual components.

**Safety by Design**: Collision avoidance is integrated into the planning pipeline rather than treated as a post-hoc validation step. Safety margins are built into the geometric representation, and collision checking is performed continuously throughout the planning process.

**Real-Time Operation**: The architecture is designed to maintain responsiveness suitable for interactive manipulation tasks, with state updates, planning, and execution operating at frequencies that support smooth motion and rapid task completion.

### 3.1.2 Overall System Architecture

The MARS architecture consists of four primary layers that form a processing pipeline from hardware interfaces through planning to execution:

**Layer 1: Hardware Interface Layer**

This layer manages communication with physical robotic hardware, encapsulating the specific protocols and interfaces required by the manipulator controllers. Each arm has an independent driver that publishes joint states and accepts trajectory commands within its own namespace. The namespace isolation at this layer is crucial for maintaining logical separation between arms while enabling higher layers to reason about the complete system.

The hardware interface layer abstracts away hardware-specific details, presenting a consistent ROS2-based interface to higher layers regardless of the specific robot manufacturer or control protocol. This abstraction enables the system to work with different robotic platforms without modifying upper layers.

**Layer 2: State Management Layer**

The state management layer bridges between the namespace-isolated hardware interfaces and the unified planning representation required for centralized motion planning. This layer performs bidirectional translation: aggregating individual arm states into a complete system state for planning, and routing unified trajectory commands back to appropriate hardware controllers.

A critical design decision in this layer is the use of active aggregation rather than passive observation. Rather than expecting some external system to combine states, MARS includes dedicated components that actively subscribe to multiple namespaced topics, perform translation, and publish unified representations. This active approach provides explicit control over timing, synchronization, and error handling.

**Layer 3: Planning and Coordination Layer**

This layer contains the motion planning infrastructure and coordination logic. The planning component maintains a unified collision scene representing the complete multi-arm system and provides motion planning services that generate collision-free trajectories. The coordination mechanisms determine how planning is performed—whether arms are planned together as a unified system, independently with subsequent collision checking, or completely separately.

The design of this layer reflects the multi-mode coordination philosophy. Rather than hardcoding a single coordination approach, the layer provides multiple planning interfaces (single-arm groups and dual-arm group) and allows client code to select which interface to use based on task requirements.

**Layer 4: Execution Layer**

The execution layer manages trajectory following, monitoring execution progress, and providing feedback. This layer includes controller interfaces that accept trajectory commands and drive the physical hardware to follow planned paths. The execution layer can operate in different modes corresponding to the planning layer's coordination strategies: unified execution where a single trajectory is followed by multiple arms, parallel execution where independent trajectories are executed simultaneously, or sequential execution where arms move one at a time.

### 3.1.3 Key Architectural Decisions

Several high-level architectural decisions fundamentally shaped the system design:

**Decision 1: Centralized vs. Distributed Planning**

The choice to employ centralized planning rather than distributed coordination represents a fundamental architectural commitment. In distributed approaches, each arm has its own planner that generates trajectories independently, with coordination achieved through communication protocols that negotiate workspace access or resolve conflicts reactively.

MARS uses centralized planning where a single planning scene represents all manipulators and a unified planner generates trajectories that are collision-free by construction. This approach was chosen because:

- **Completeness**: Centralized planning can reason about inter-arm relationships globally, finding solutions that distributed approaches might miss when local planning horizons prevent arms from seeing collaborative opportunities.

- **Determinism**: Centralized planning produces consistent results for given inputs, whereas distributed approaches can exhibit emergent behaviors that are difficult to predict or debug.

- **Simplicity**: A single planning scene and unified collision checker is conceptually and computationally simpler than managing multiple planners with inter-planner communication protocols.

The primary trade-off is scalability—centralized planning in high-dimensional spaces becomes computationally expensive, and the approach becomes less tractable as more arms are added. However, for dual-arm systems, the twelve-dimensional configuration space remains within the capability of modern sampling-based planners.

**Decision 2: Multi-Mode Coordination**

Rather than providing a single coordination mechanism, MARS implements three distinct modes. This decision was driven by the observation that different manipulation tasks have fundamentally different coordination requirements:

- Tasks like bimanual grasping require tight spatial and temporal coupling
- Tasks like workspace handoff need spatial awareness but tolerate timing flexibility  
- Tasks like parallel pick-and-place in separate zones benefit from complete independence

Providing multiple modes increases architectural complexity but enables the same hardware platform to efficiently handle diverse scenarios. The alternative—optimizing for a single mode—would constrain system applicability to a narrow set of tasks.

**Decision 3: Namespace-Based Hardware Isolation**

The decision to maintain namespace isolation at the hardware layer rather than attempting to present both arms as a single unified robot from the hardware up was driven by practical deployment considerations. Physical manipulators typically expect exclusive namespace ownership, and maintaining this isolation:

- Simplifies hardware driver configuration and deployment
- Enables independent control of each arm for maintenance or testing
- Provides clear boundaries for debugging when hardware-level issues arise
- Allows arms to be powered up, configured, or calibrated independently

The cost of this decision is the need for explicit state aggregation and trajectory proxying logic in higher layers. However, these mechanisms prove valuable beyond simple name translation, providing natural integration points for monitoring, logging, and validation.

**Decision 4: Active State Aggregation**

The state management layer uses an active aggregation approach where dedicated components explicitly subscribe to multiple topics and publish unified representations. The alternative—relying on passive aggregation tools or expecting client code to manage multiple subscriptions—was rejected because:

- Active aggregation provides explicit control over timing and synchronization
- It enables centralized logging and monitoring of state consistency
- It creates a natural location for implementing state validation and error detection
- It presents a simpler interface to planning components that need complete system state

**Decision 5: Collision Safety Through Link Padding**

Rather than relying solely on geometric collision checking against nominal link models, MARS adds safety padding to robot links. This conservative approach was chosen because:

- Physical systems always have modeling errors, calibration drift, and execution tracking errors
- Padding provides a quantifiable safety margin that accounts for these real-world factors
- The computational cost of collision checking padded geometries is negligible with modern collision libraries
- Conservative collision checking prevents near-miss scenarios that might lead to contact due to accumulated errors

The trade-off is reduced workspace efficiency—padding reduces the effective reachable workspace and may make certain tightly-constrained manipulations infeasible. However, for the intended manipulation scenarios, the safety benefits outweigh the workspace reduction.

### 3.1.4 Design Pattern Applications

The MARS architecture employs several well-established design patterns that provide structure and clarity:

**Strategy Pattern for Coordination Modes**: The three coordination modes (synchronous, hybrid, asynchronous) exemplify the strategy pattern where different algorithms (planning strategies) can be selected at runtime to achieve the same goal (coordinated manipulation) through different means. This pattern enables mode selection based on task requirements without modifying core planning infrastructure.

**Proxy Pattern for Trajectory Routing**: The trajectory proxying mechanism implements the proxy pattern, where the proxy (trajectory proxy component) presents the same interface as the real subject (hardware controllers) while adding namespace translation and routing logic. This pattern enables transparent interposition between planning and execution layers.

**Observer Pattern for State Aggregation**: The state aggregation mechanism follows the observer pattern where multiple publishers (hardware drivers) notify subscribers (state aggregator) of state changes, which then notifies its own subscribers (planning components) of the aggregated state. This decoupling enables adding or removing state sources without modifying consumers.

**Facade Pattern for Planning Interface**: The planning layer presents a simplified facade over the complexity of motion planning libraries, collision checking, inverse kinematics, and trajectory parameterization. Client code interacts with a clean planning interface without needing to understand the intricacies of the underlying planning infrastructure.

### 3.1.5 Design Requirements

The architectural design was developed to satisfy both functional and non-functional requirements derived from the objectives:

**Functional Requirements:**

FR1: The system shall plan collision-free trajectories for dual six-degree-of-freedom manipulators operating in shared workspace

FR2: The system shall support three distinct coordination modes: synchronous (unified planning and execution), hybrid (unified planning with parallel execution), and asynchronous (independent planning and execution)

FR3: The system shall aggregate joint states from multiple namespace-isolated hardware interfaces into unified state representation

FR4: The system shall route trajectory commands from unified planning representation to appropriate namespace-specific hardware controllers

FR5: The system shall support both joint-space goals (target joint configurations) and Cartesian goals (target end-effector poses)

FR6: The system shall maintain minimum safety margins between manipulators during planned motion

FR7: The system shall provide gripper control integration for grasping operations

**Non-Functional Requirements:**

NFR1: State aggregation shall operate at sufficient frequency (≥30Hz) for real-time planning

NFR2: Planning shall complete within acceptable time bounds (≤10 seconds for typical scenarios)

NFR3: Trajectory execution shall maintain acceptable position accuracy for manipulation tasks

NFR4: The system shall scale to handle twelve-dimensional configuration space exploration

NFR5: Component interfaces shall be sufficiently modular to enable independent testing and future extension

NFR6: The architecture shall be compatible with standard robotics frameworks and tools

These requirements guided design decisions throughout the development process, with trade-offs made where requirements conflicted (e.g., balancing planning completeness against computation time).

---

## 3.2 Shared Workspace Management Design

### 3.2.1 Design Objectives

The workspace management design must satisfy several competing objectives: ensuring collision-free operation between manipulators, maximizing effective workspace utilization, maintaining computational efficiency for real-time planning, and providing quantifiable safety margins that account for real-world uncertainties.

The fundamental challenge is that naive collision checking in a dual-arm system would require evaluating every pair of links from both arms at every configuration sampled during planning. For two arms with seven major links each, this could require checking up to 49 collision pairs per configuration sample. With sampling-based planners potentially evaluating thousands of configurations per planning query, the computational burden would be substantial.

### 3.2.2 Unified Planning Scene Design

The core design decision for workspace management is the use of a unified planning scene that represents both manipulators as a single composite robot model. This scene maintains collision geometries for all links across both arms and provides collision checking services to the motion planner.

**Design Rationale:**

A unified planning scene was chosen over alternative approaches (separate scenes per arm, dynamic scene merging, distributed collision checking) because it provides:

- **Global Awareness**: The planner has complete information about both arms' configurations during trajectory generation, enabling it to reason about inter-arm relationships
- **Consistency**: A single scene representation ensures consistent collision checking throughout planning—there is no risk of distributed checkers having divergent views of system state
- **Integration**: Standard motion planning frameworks expect a single scene, making this approach compatible with existing tools

**Alternative Approaches Considered:**

*Separate Planning Scenes*: Maintaining independent scenes per arm with explicit synchronization was rejected because it complicates collision checking between arms and requires careful management of scene consistency when both arms are moving.

*Dynamic Scene Merging*: Building separate scenes and merging them on-demand was rejected due to the overhead of repeated merging operations and the complexity of managing multiple scene representations.

**Scene Update Strategy:**

The planning scene receives state updates from the state management layer and maintains a current configuration for all joints. The scene update frequency (15 Hz) was chosen to balance responsiveness against computational overhead. More frequent updates would provide marginal improvements in state freshness while consuming planning resources, while less frequent updates could result in stale state information during planning.

### 3.2.3 Collision Avoidance Through Link Padding

Safety margins are incorporated through link padding, where collision geometries are expanded beyond their nominal dimensions. Each critical link receives uniform padding, creating separation zones around robot components.

**Padding Magnitude Selection:**

The padding magnitude (5 cm per link) was selected through analysis of expected error sources:

- **Modeling Errors**: CAD models typically achieve ±2mm accuracy, but joint backlash and mechanical compliance can introduce ±3mm deviations
- **Calibration Drift**: Robot calibration can drift ±3mm over operational periods due to thermal effects and mechanical wear
- **Execution Tracking**: Trajectory following typically maintains ±5mm accuracy under dynamic conditions
- **Safety Factor**: A 2× safety factor on the sum of maximum expected errors yields approximately 26mm minimum padding

The 50mm padding provides nearly 2× this minimum requirement, offering conservative safety margins. The cost is reduced effective workspace—padded geometries make some configurations infeasible that would be geometrically possible with nominal link dimensions.

**Uniform vs. Adaptive Padding:**

Uniform padding (same magnitude for all links) was chosen over adaptive approaches (different padding per link, or dynamic padding based on velocity) for several reasons:

- **Simplicity**: Uniform padding is easy to configure, understand, and validate
- **Predictability**: Constant padding ensures consistent behavior across all configurations
- **Conservative Safety**: Using the maximum required padding everywhere errs on the side of caution

Adaptive padding could theoretically reduce workspace constraints, but would introduce complexity in determining appropriate padding factors and validating safety across all scenarios.

### 3.2.4 Collision Matrix Optimization

While link padding ensures safety, it does not address computational efficiency. Collision checking remains expensive if every link pair is tested at every configuration.

**Collision Matrix Design:**

The collision matrix explicitly encodes which link pairs should be checked for collision and which can be safely ignored. Three categories of disabled collision pairs reduce computational burden:

*Adjacent Links*: Links connected by a joint cannot collide due to kinematic constraints. For example, the shoulder link and upper arm link are connected by a revolute joint and their relative motion is constrained such that collision is geometrically impossible.

*Never-Colliding Links*: Links that are sufficiently separated in the kinematic chain that they cannot reach each other regardless of joint configuration. For example, the base link of one arm cannot possibly collide with the end-effector link of the same arm because multiple intermediate links separate them.

*Inter-Arm Base Links*: The base links of both arms are fixed in space with sufficient separation that collision is impossible. These are effectively treated as adjacent in the collision matrix despite not being directly connected kinematically.

**Matrix Generation Process:**

The collision matrix is generated through a combination of kinematic analysis and exhaustive configuration sampling:

1. Adjacent links are identified through kinematic chain analysis
2. Links separated by sufficient kinematic distance are identified heuristically
3. Random configuration sampling validates that heuristically-identified never-colliding pairs indeed never collide across the configuration space
4. Manual review ensures no geometrically feasible collision pairs are inappropriately disabled

This semi-automated approach balances computational efficiency of matrix generation against the need for validated safety.

**Performance Impact:**

The collision matrix typically reduces active collision checks by approximately 75% (from ~200 potential pairs to ~50 active pairs for a dual-arm system with grippers). This reduction translates directly to faster planning, enabling real-time performance for typical scenarios.

### 3.2.5 Phase-Based Execution Strategy

For complex manipulation tasks that involve narrow passages or require arms to pass close to each other, the system employs phase-based execution rather than attempting real-time dynamic obstacle avoidance.

**Design Rationale:**

Phase-based execution decomposes complex tasks into sequential segments where each segment is validated as collision-free before execution begins. This approach was chosen over dynamic replanning because:

- **Predictability**: Each phase is explicitly planned and validated, providing deterministic behavior
- **Simplicity**: No runtime replanning infrastructure is needed
- **Safety**: Each phase can be validated thoroughly before execution, reducing risk of runtime failures
- **Computational Efficiency**: Planning occurs offline (relative to execution) so computational cost does not impact execution responsiveness

**Phase Decomposition Strategy:**

Tasks are decomposed into phases based on workspace occupancy and motion characteristics. A typical decomposition might be:

- Phase 1: Move both arms from current configuration to intermediate poses that provide clearance
- Phase 2: Execute the primary manipulation motion for arm 1 while arm 2 remains stationary  
- Phase 3: Execute primary manipulation for arm 2 while arm 1 moves to final position
- Phase 4: Return both arms to home positions

Each phase boundary is selected to ensure collision-free motion within the phase and collision-free transitions between phases.

**Trade-offs:**

Phase-based execution sacrifices reactivity—the system cannot respond to unexpected obstacles during execution—in exchange for simplified design and deterministic behavior. For the intended manipulation scenarios where the workspace is controlled and obstacles are known in advance, this trade-off is acceptable.

**Alternative Approaches Considered:**

*Dynamic Replanning*: Continuously replanning during execution in response to sensor data was rejected due to the computational complexity and the lack of unpredictable dynamic obstacles in the target scenarios.

*Reactive Collision Avoidance*: Using potential fields or similar reactive methods was rejected because reactive approaches can fail to find solutions in constrained environments and lack the completeness guarantees of planning-based approaches.

---

## 3.3 Task Coordination and Synchronization Design

### 3.3.1 Design Objectives

The coordination design must support diverse manipulation scenarios with different temporal and spatial coupling requirements. Some tasks require precise synchronization where both arms must move in perfect temporal alignment. Other tasks need spatial awareness to avoid collisions but tolerate loose timing. Still others benefit from complete independence where arms operate without coordination overhead.

A key insight driving the design is that no single coordination strategy is optimal across all scenarios. The system must provide flexibility while maintaining simplicity and avoiding mode proliferation.

### 3.3.2 Multi-Mode Architecture Design

The coordination architecture provides three distinct operational modes that cover the spectrum from tight coupling to complete independence.

**Mode 1: Synchronous Coordination**

*Design Concept*: Treat both arms as a single unified twelve-degree-of-freedom robot. Planning and execution operate on this unified representation, ensuring both arms move in temporal alignment.

*Planning Mechanism*: The planning interface exposes a "dual" planning group that includes all joints from both arms. When planning for this group, the motion planner explores the twelve-dimensional configuration space and generates trajectories where both arms are always coordinated.

*Execution Mechanism*: The generated trajectory contains waypoints for all twelve joints with synchronized timestamps. The execution system routes the trajectory to both arm controllers simultaneously, and the controllers follow their respective segments in parallel.

*Design Rationale*: This mode was designed for scenarios where precise bimanual coordination is essential, such as grasping a single large object with both arms or performing assembly tasks that require specific relative positioning between end-effectors.

**Mode 2: Hybrid Coordination**

*Design Concept*: Plan using the unified representation to ensure collision-free motion, but execute trajectories independently on separate controllers to allow timing flexibility.

*Planning Mechanism*: Planning uses the same dual planning group as synchronous mode, ensuring the generated trajectory is collision-free with both arms moving.

*Execution Mechanism*: After planning, the unified trajectory is split into arm-specific segments. Each segment is sent to its respective controller independently. Controllers execute in parallel without synchronization.

*Design Rationale*: This mode targets scenarios where collision avoidance is critical but precise timing synchronization is unnecessary. The unified planning ensures arms don't collide, while independent execution allows timing variations without requiring inter-controller communication.

**Mode 3: Asynchronous Coordination**

*Design Concept*: Plan and execute each arm completely independently, with no inter-arm coordination during planning or execution.

*Planning Mechanism*: Each arm uses its own six-degree-of-freedom planning group. Planning for one arm does not consider the other arm's configuration or trajectory.

*Execution Mechanism*: Trajectories are sent to controllers independently using non-blocking action interfaces. Arms move in parallel without coordination.

*Design Rationale*: This mode maximizes throughput for scenarios where arms operate in separate workspace regions or where temporal sequencing provides sufficient coordination. The lack of coordination overhead enables minimum cycle times.

**Mode Selection Design:**

Mode selection is delegated to the client application layer rather than automated by the coordination system. This design decision reflects the observation that mode selection depends on task semantics that the coordination system cannot infer automatically. The client (which understands task requirements) selects the mode by choosing which planning interface to use and which execution method to invoke.

### 3.3.3 Planning Group Design

The planning infrastructure exposes three planning groups that correspond to the coordination modes:

**Group "arm_1"**: Six joints from the first manipulator (joint_1 through joint_6 with arm_1 prefix)

**Group "arm_2"**: Six joints from the second manipulator (joint_1 through joint_6 with arm_2 prefix)

**Group "dual"**: All twelve joints from both manipulators (composite of arm_1 and arm_2 groups)

This group structure leverages the hierarchical group capabilities of standard motion planning frameworks. The dual group is defined as a subgroup containing both arm groups, enabling the planning infrastructure to treat it as a single planning problem while maintaining the individual arm group definitions for independent planning.

**Design Benefits:**

- Natural mapping between planning groups and coordination modes
- Standard planning framework semantics—no custom group types required
- Clear specification of planning scope through group selection

**Alternative Approaches Considered:**

*Dynamic Group Formation*: Creating planning groups on-the-fly based on which arms are involved in a task was rejected due to the overhead of group creation and the potential for configuration errors.

*Single Unified Group Only*: Using only the dual group and relying on motion constraint specifications to achieve independent arm planning was rejected because it would complicate client code and obscure the coordination mode being used.

### 3.3.4 Trajectory Splitting Design

For hybrid mode execution, the system must split unified twelve-joint trajectories into arm-specific six-joint trajectories.

**Splitting Algorithm Design:**

The splitting process identifies which trajectory joints belong to each arm through name prefix matching. For each waypoint in the unified trajectory, arm-specific waypoints are created containing only the joints belonging to that arm. Timestamps are preserved from the unified trajectory, so both arm-specific trajectories maintain the same timing structure.

**Design Considerations:**

*Preserving Temporal Information*: The split trajectories maintain the same time-from-start values as the unified trajectory. This preserves the temporal relationships established during planning, even though execution will not enforce synchronization.

*Joint Ordering*: Arm-specific trajectories maintain the same joint ordering as the unified trajectory to avoid confusion and simplify validation.

*Metadata Preservation*: Velocity and acceleration constraints are preserved during splitting, ensuring execution safety is maintained.

### 3.3.5 Asynchronous Execution Framework Design

Asynchronous mode requires infrastructure to manage concurrent trajectory execution with callback-based completion notification.

**Executor Component Design:**

A dedicated executor component manages asynchronous execution state, tracking which arms are currently executing trajectories and providing callback mechanisms for completion notification.

**Action Client Architecture:**

The executor maintains action clients for each arm's trajectory controller. Action clients provide non-blocking goal submission through asynchronous interfaces, enabling parallel execution without thread blocking.

**Callback Management:**

Client code registers callback functions that are invoked upon trajectory completion. The callback mechanism enables reactive task sequencing where subsequent actions depend on completion of prior actions.

**Design Pattern:**

This design follows the asynchronous programming pattern common in robotic systems, where long-running operations (trajectory execution) should not block the main control thread. The callback mechanism provides loose coupling between execution completion and subsequent actions.

---

## 3.4 State Management Design

### 3.4.1 Design Objectives

State management must bridge the namespace-isolated hardware interfaces with the unified planning representation. The design must maintain real-time state updates, preserve temporal consistency across multiple asynchronous state sources, handle bidirectional information flow (state aggregation upward, command routing downward), and provide interfaces that are simple for client code to use.

### 3.4.2 State Aggregation Component Design

**Component Responsibilities:**

The state aggregator subscribes to joint state topics from multiple namespaced hardware interfaces, combines these states into a unified representation, publishes the unified state at a consistent rate, and maintains thread-safe internal state to handle concurrent updates.

**Namespace Translation Design:**

The core functionality of state aggregation is namespace translation. Incoming joint state messages use unprefixed joint names (joint_1, joint_2, etc.) because hardware drivers are unaware of the multi-arm context. The aggregator prepends namespace prefixes (arm_1_, arm_2_) to create unique joint names in the unified representation.

*Design Rationale*: Namespace prefixing at this layer rather than modifying hardware drivers maintains separation of concerns—hardware drivers remain simple and reusable, while the aggregation layer handles multi-arm-specific naming.

**Update Rate Design:**

The aggregator publishes unified state at a fixed rate (40 Hz) rather than reactively publishing whenever any arm's state updates. Fixed-rate publication was chosen because:

- Planning components benefit from predictable state update timing
- Fixed-rate publication naturally handles asynchronous hardware updates with different frequencies
- It simplifies synchronization logic—no need to wait for all arms before publishing

The 40 Hz rate was selected as a balance between responsiveness (25ms update period is fast relative to motion planning timescales) and computational efficiency (higher rates provide diminishing returns).

**Thread Safety Design:**

State updates from multiple hardware interfaces arrive asynchronously on different callback threads. The aggregator protects its internal state with mutex locks to prevent race conditions during concurrent updates.

*Callback Group Strategy*: A mutually exclusive callback group ensures that joint state callbacks execute serially, preventing simultaneous access to shared state. This trades some potential concurrency for simplified synchronization logic.

### 3.4.3 Trajectory Proxy Component Design

**Component Responsibilities:**

The trajectory proxy accepts trajectory commands with prefixed joint names (from planning layer), strips namespace prefixes to create hardware-compatible trajectories, routes trajectories to appropriate arm controllers based on joint name prefixes, and relays execution feedback from hardware back to clients.

**Proxy Pattern Application:**

The trajectory proxy implements the proxy pattern by presenting the same action interface that hardware controllers present, but interposing namespace translation logic. From the client perspective, sending trajectories through the proxy appears identical to sending directly to hardware.

**Routing Logic Design:**

Trajectory routing is based on joint name prefix matching. For each trajectory command, the proxy:

1. Examines joint names to determine which arm(s) are involved
2. Creates arm-specific trajectory commands by filtering joints and stripping prefixes
3. Forwards arm-specific commands to corresponding hardware controller action servers

*Design Consideration*: The routing logic must handle three cases: single-arm trajectories (all joints belong to one arm), dual-arm trajectories (joints from both arms), and error cases (unknown joint names). The design treats single-arm and dual-arm cases uniformly through the filtering logic.

**Action Interface Design:**

The proxy provides action interfaces (goal, feedback, result) that mirror the hardware controller interfaces. This enables transparent substitution—client code that worked with direct hardware control can work unchanged with the proxy.

*Feedback Relay*: Execution feedback (current position, error, time remaining) is relayed from hardware back through the proxy to clients. This maintains the action semantics where clients can monitor execution progress.

### 3.4.4 State Representation Design

**Unified State Format:**

The unified state representation uses standard ROS joint state messages containing arrays of joint names, positions, velocities, and efforts. The message format is:

```
header:
  stamp: current_time
  frame_id: ""
name: [arm_1_joint_1, arm_1_joint_2, ..., arm_2_joint_6]
position: [θ₁₁, θ₁₂, ..., θ₂₆]
velocity: [ω₁₁, ω₁₂, ..., ω₂₆]
effort: [τ₁₁, τ₁₂, ..., τ₂₆]
```

*Design Benefits*: Using standard message types ensures compatibility with existing ROS tools (visualization, logging, analysis). The flat array structure is efficient for serialization and transmission.

**Joint Name Ordering:**

Joint names in the unified state follow a consistent ordering: all arm_1 joints in ascending numeric order, followed by all arm_2 joints in ascending numeric order. Consistent ordering simplifies debugging and validation—the joint state array structure is always predictable.

### 3.4.5 Transform Tree Integration Design

The state management design integrates with the ROS transform tree (TF) system to provide geometric relationships between robot links.

**Transform Publisher Design:**

A robot state publisher component consumes the unified joint state and computes forward kinematics for all robot links. The resulting transforms are published to the TF tree, providing a real-time representation of robot geometry.

*Design Integration*: The robot state publisher is a standard component from the ROS ecosystem. The state management design ensures compatibility by providing joint states in the standard format expected by the robot state publisher.

**Transform Tree Structure:**

The TF tree has a single root frame (world) with both arm base frames as children. Each arm's kinematic chain extends from its base frame through its links. The unified tree structure enables:

- Coordinate transformations between any two frames in the system
- Geometric queries (distance between links, relative poses)
- Visualization of complete system geometry

**Update Frequency:**

Transform updates (15 Hz) are slower than joint state updates (40 Hz) because transform publication is more computationally expensive. The reduced rate was chosen as sufficient for visualization and planning while limiting computational burden.

---

## 3.5 Planning and Trajectory Execution Design

### 3.5.1 Design Objectives

The planning and execution design must support motion planning in high-dimensional configuration spaces (twelve dimensions for dual-arm system), provide inverse kinematics for converting Cartesian goals to joint configurations, generate smooth trajectories respecting kinematic and dynamic constraints, route trajectories to appropriate controllers based on coordination mode, and monitor execution progress with error handling.

### 3.5.2 Motion Planning Infrastructure Design

**Planning Framework Selection:**

The design leverages established motion planning frameworks rather than implementing custom planning algorithms. This decision was based on:

- Maturity: Established frameworks are extensively tested and validated
- Completeness: Modern sampling-based planners provide probabilistic completeness guarantees
- Performance: Optimized implementations outperform custom code
- Maintainability: Using standard frameworks reduces maintenance burden

**Sampling-Based Planning Design:**

Sampling-based algorithms were selected over deterministic approaches (roadmaps, cell decomposition) because:

- High-dimensional spaces favor sampling—exhaustive decomposition becomes intractable above 6-8 dimensions
- Sampling-based planners naturally handle complex collision geometries
- Probabilistic completeness is sufficient for manipulation tasks

*Algorithm Selection*: The default planner (RRTConnect) was chosen for its balance of completeness and performance. Bidirectional search from start and goal typically finds solutions faster than single-tree approaches.

**Planning Pipeline Design:**

The planning pipeline consists of:

1. **Request Adapters**: Pre-process planning requests to resolve frame references, validate workspace bounds, and check start state validity
2. **Core Planner**: Explore configuration space and find collision-free path
3. **Response Adapters**: Post-process paths to add timing information, validate solutions, and generate visualization

*Design Rationale*: The adapter pattern enables modular extension—new validation or processing steps can be added without modifying the core planner.

### 3.5.3 Inverse Kinematics Design

**IK Solver Integration:**

Inverse kinematics translates Cartesian goals (desired end-effector poses) into joint configurations. The design integrates an IK solver library that provides:

- Iterative solving using Jacobian-based methods
- Configurable timeout and search resolution
- Collision-aware IK that rejects solutions in collision

**IK Solver Configuration:**

Key configuration parameters:

- *Search Resolution*: Step size for iterative solving (0.005 radians = ~0.3 degrees)
- *Timeout*: Maximum solving time (5 milliseconds)
- *Attempts*: Multiple solution attempts with different seed states

*Design Trade-offs*: The 5ms timeout balances solution quality against responsiveness. Shorter timeouts might fail on difficult IK problems, while longer timeouts delay planning. Multiple attempts with varied seeds improve success rate while bounding total computation time.

**Seed State Strategy:**

IK solving is sensitive to initial seed configuration. The design uses the current robot state as the seed, hypothesizing that solutions near the current configuration are more likely to be reachable through smooth motion.

*Alternative Seeds*: If IK fails with the current state seed, subsequent attempts use randomly perturbed seeds. This provides fallback strategies when the current state is far from solutions.

### 3.5.4 Trajectory Parameterization Design

**Time-Optimal Parameterization:**

After finding a geometric path (sequence of collision-free waypoints), the system must compute velocity and acceleration profiles that traverse the path as quickly as possible while respecting joint limits.

*Algorithm Selection*: Time-optimal parameterization algorithms analyze the path and compute maximum safe velocities for each segment based on curvature, joint velocity limits, and acceleration constraints.

**Velocity Scaling Design:**

A velocity scaling factor (default 30%) is applied globally to all computed velocities. This conservative scaling provides safety margins:

- Model uncertainties: Actual joint limits may be lower than specified
- Execution tracking: Controllers follow faster commands less accurately
- Emergency stopping: Reduced velocities enable faster reaction to unexpected events

*Design Consideration*: Velocity scaling is applied after time-optimal parameterization rather than modifying joint limits before planning. This enables runtime adjustment of scaling without replanning.

**Trajectory Smoothing:**

The parameterization process includes smoothing that removes unnecessary waypoints and simplifies trajectories while maintaining collision-free properties.

*Design Benefits*: Smoother trajectories reduce jerk (rate of acceleration change), improving execution quality and reducing wear on mechanical components.

### 3.5.5 Controller Architecture Design

**Controller Design Philosophy:**

The controller architecture provides independent controllers per arm plus a unified controller for coordinated execution. This design supports all three coordination modes:

- Synchronous mode uses the unified controller
- Hybrid mode uses independent controllers
- Asynchronous mode uses independent controllers

**Controller Interfaces:**

Each controller presents an action interface (FollowJointTrajectory) that:

- Accepts trajectory goals with target waypoints
- Executes trajectories through position commands to hardware
- Publishes feedback on execution progress
- Reports result on completion or failure

*Design Standard*: The FollowJointTrajectory action interface is a ROS standard, ensuring compatibility with various robot hardware and higher-level frameworks.

**Update Rate Design:**

Controllers operate at 100 Hz update rate, commanding new target positions every 10 milliseconds. This rate was selected as:

- Fast enough for smooth motion on typical manipulators
- Slow enough that communication latency remains small relative to control period
- Standard rate used by many robotic systems

**Trajectory Interpolation:**

Between trajectory waypoints, controllers interpolate target positions. The interpolation strategy (typically cubic or quintic splines) ensures smooth motion without position discontinuities.

### 3.5.6 Execution Monitoring Design

**Progress Monitoring:**

During execution, the system monitors trajectory following through:

- Position error: difference between commanded and actual joint positions
- Velocity tracking: whether actual velocities match commanded profiles  
- Timing: whether execution is ahead or behind expected schedule

*Design Purpose*: Monitoring enables detection of execution problems (hardware faults, excessive load, tracking failures) that might compromise task completion.

**Error Handling Strategy:**

The design includes error detection but delegates error recovery decisions to higher-level task logic. When errors are detected, the system:

1. Halts trajectory execution if errors exceed thresholds
2. Reports error status through action result
3. Awaits instructions from client code on how to proceed

*Design Rationale*: Recovery strategies are task-specific (retry, replan, abort), so the execution layer reports errors without imposing recovery policies.

**Timeout Mechanism:**

A configurable timeout (default 120 seconds) prevents indefinite blocking on stalled trajectories. If execution does not complete within the timeout, the action is terminated with a timeout error.

*Design Parameter*: The 120-second timeout accommodates slow velocities (20% scaling) over long paths while preventing truly stuck trajectories from blocking indefinitely.

---

## 3.6 Component Design

This section details the design of major system components, their responsibilities, interfaces, and interactions.

### 3.6.1 Joint State Aggregator Component

**Component Overview:**

The Joint State Aggregator bridges namespace-isolated hardware interfaces and unified planning representations. It is a critical component in the state management layer.

**Responsibilities:**

- Subscribe to joint state topics from multiple robot namespaces
- Aggregate states into unified representation with namespace prefixes
- Publish unified state at consistent rate
- Maintain thread-safe internal state under concurrent updates

**Interface Design:**

*Inputs:*
- `/arm_1/joint_states` (JointState message, unprefixed names)
- `/arm_2/joint_states` (JointState message, unprefixed names)

*Outputs:*
- `/joint_states` (JointState message, prefixed names)

*Parameters:*
- `robot_namespaces`: List of robot namespaces to aggregate
- `publish_frequency`: Unified state publication rate (Hz)

**Internal Design:**

The component maintains a dictionary mapping namespace to most recent joint state. When a state update arrives, it is stored in the dictionary with mutex protection. A timer callback executes at the configured frequency, aggregating all stored states into a unified message.

*State Representation:*
```
internal_state = {
  "arm_1": JointState(name=[joint_1, ...], position=[...], ...),
  "arm_2": JointState(name=[joint_1, ...], position=[...], ...)
}
```

**Synchronization Design:**

A mutex protects the internal state dictionary. Callback threads acquire the mutex before updating state, and the timer thread acquires it before reading state for aggregation. This ensures atomic updates and prevents partially-consistent state reads.

**Prefix Application Logic:**

During aggregation, joint names are transformed:
```
Original: [joint_1, joint_2, ..., joint_6] from /arm_1/joint_states
Transformed: [arm_1_joint_1, arm_1_joint_2, ..., arm_1_joint_6]
```

The prefix includes an underscore separator for readability and to match URDF joint naming conventions.

**Edge Case Handling:**

- Missing state: If any arm's state is missing (null), aggregation is skipped for that cycle
- Stale state: No explicit staleness checking—most recent state is always used
- Initialization: Component waits until all arm states are received before publishing

### 3.6.2 Trajectory Proxy Component

**Component Overview:**

The Trajectory Proxy routes trajectory commands from the planning layer to appropriate hardware controllers while handling namespace translation.

**Responsibilities:**

- Accept trajectory commands with prefixed joint names
- Strip namespace prefixes to create hardware-compatible commands
- Route commands to appropriate controllers
- Relay execution feedback and results

**Interface Design:**

*Inputs (Action Servers):*
- `/arm_1/follow_joint_trajectory_prefixed` (FollowJointTrajectory action)
- `/arm_2/follow_joint_trajectory_prefixed` (FollowJointTrajectory action)

*Outputs (Action Clients):*
- `/arm_1/controller/follow_joint_trajectory` (FollowJointTrajectory action)
- `/arm_2/controller/follow_joint_trajectory` (FollowJointTrajectory action)

*Parameters:*
- `trajectory_timeout_sec`: Maximum execution time before timeout
- `server_timeout_sec`: Maximum time waiting for hardware server connection

**Proxy Logic Design:**

When a trajectory goal arrives at the proxy:

1. **Namespace Extraction**: Identify target namespace from joint name prefixes
2. **Prefix Stripping**: Remove namespace prefixes from all joint names
3. **Goal Forwarding**: Create new goal with unprefixed trajectory, forward to hardware client
4. **State Tracking**: Track goal handle for feedback relay
5. **Result Relay**: Forward execution result back to original client

**Concurrent Execution Support:**

The proxy can handle multiple concurrent trajectory executions (different arms moving simultaneously). This is enabled through:

- Reentrant callback group allowing concurrent action callbacks
- Separate action clients per arm preventing interference
- Stateless processing where each goal is handled independently

**Error Propagation Design:**

Errors from hardware controllers (planning failure, execution error, timeout) are propagated back to clients unchanged. The proxy does not attempt error recovery or filtering—it transparently relays hardware status.

### 3.6.3 Coordination Manager Component (Conceptual)

**Component Overview:**

While MARS does not implement a centralized coordination manager component, the conceptual design clarifies how coordination logic is distributed across the system.

**Coordination Responsibilities Distribution:**

- *Mode Selection*: Delegated to client code through planning group choice
- *Trajectory Generation*: Handled by motion planner based on selected group
- *Trajectory Splitting*: Performed by utility functions when hybrid mode is used
- *Execution Dispatch*: Managed by execution layer based on trajectory content

**Design Rationale for Distributed Coordination:**

Rather than a monolithic coordination manager, coordination logic is distributed because:

- Planning group selection naturally encodes coordination mode
- Trajectory splitting is a pure function with no state
- Execution dispatch follows standard action interfaces

This distribution keeps components focused and avoids a "god object" that touches all system aspects.

**Alternative Design Considered:**

A centralized coordinator that encapsulates mode selection, planning invocation, and execution dispatch was considered but rejected because it would:

- Create tight coupling between planning and execution layers
- Complicate testing (coordinator would need mock implementations of many components)
- Obscure the coordination mode being used (hidden behind coordinator interface)

### 3.6.4 Motion Planner Interface Component

**Component Overview:**

The motion planner interface wraps the underlying planning library, providing a consistent interface for planning requests regardless of algorithm or configuration.

**Responsibilities:**

- Accept planning requests (start state, goal state, planning group)
- Configure and invoke appropriate planning algorithm
- Process planning results and return structured responses
- Handle planning failures gracefully

**Interface Design:**

*Inputs:*
- Planning request containing:
  - Planning group name (arm_1, arm_2, or dual)
  - Start state (joint configuration or "current")
  - Goal state (joint configuration, pose, or named state)
  - Planning constraints (optional)

*Outputs:*
- Planning result containing:
  - Success/failure status
  - Trajectory (if successful)
  - Error code and description (if failed)
  - Planning time statistics

**Request Processing Design:**

The component processes requests through several stages:

1. **Validation**: Check that planning group exists, start/goal states are valid
2. **Scene Update**: Ensure planning scene reflects current robot state
3. **Constraint Processing**: Convert high-level constraints to planner-specific format
4. **Planning Invocation**: Call underlying planning algorithm
5. **Result Processing**: Extract trajectory and statistics from planner output

**Goal Specification Flexibility:**

The interface accepts multiple goal specification formats:

- *Joint goals*: Explicit target joint angles
- *Pose goals*: Target end-effector poses (automatically invokes IK)
- *Named goals*: Pre-defined configurations from semantic description

This flexibility enables clients to specify goals naturally for their task domain.

### 3.6.5 Workspace Scene Manager Component

**Component Overview:**

The Workspace Scene Manager maintains the collision scene used for planning, handling scene updates and collision object management.

**Responsibilities:**

- Load static collision objects from configuration
- Update robot state in planning scene
- Provide scene query interfaces (collision checking, distance queries)
- Handle dynamic collision objects (future capability)

**Interface Design:**

*Configuration Input:*
- Workspace scene definition file specifying static obstacles

*Runtime Updates:*
- Joint state updates from state aggregation layer

*Query Interface:*
- Collision check for configuration
- Distance between links
- Collision pairs for configuration

**Scene Update Strategy:**

Robot state updates trigger scene updates where joint positions are updated in the planning scene's internal representation. The update frequency (15 Hz) is independent of joint state publication frequency (40 Hz) to limit computational overhead.

*Design Trade-off*: Some latency exists between actual robot state and planning scene state, but the 67ms maximum latency (1/15 Hz) is acceptable for planning timescales.

**Static Object Management:**

Static objects (workspace boundaries, fixtures) are loaded at initialization from configuration files. The design assumes static objects remain fixed—no runtime addition or removal is supported.

*Rationale*: Static object management simplifies the implementation. Dynamic object handling would require scene synchronization protocols and consistency management.

---

## 3.7 Data Flow Architecture

### 3.7.1 State Update Flow

The state update data flow represents how joint positions flow from hardware through aggregation to planning:

```
Hardware Controllers
    ↓ [Position sensors]
Robot Drivers (arm_1, arm_2)
    ↓ [Publish to /arm_X/joint_states]
Joint State Aggregator
    ↓ [Aggregate + prefix → /joint_states]
Robot State Publisher
    ↓ [Forward kinematics]
TF Tree
    ↓ [Transforms]
Planning Scene (state update)
```

**Flow Characteristics:**

- *Direction*: Unidirectional (hardware → planning)
- *Frequency*: 40 Hz at aggregator output
- *Latency*: <25ms from hardware to planning scene
- *Reliability*: Best-effort (missed updates are not critical)

### 3.7.2 Trajectory Command Flow

The trajectory command flow represents how motion commands flow from planning through execution to hardware:

```
Planning Request (client code)
    ↓
Motion Planner
    ↓ [Generated trajectory with prefixed names]
Trajectory Proxy
    ↓ [Strip prefixes, route by namespace]
Hardware Controllers
    ↓ [Position commands]
Robot Drivers
    ↓ [Protocol translation]
Physical Robots
```

**Flow Characteristics:**

- *Direction*: Unidirectional (planning → hardware)
- *Frequency*: Event-driven (per planning request)
- *Reliability*: Reliable action interface with feedback
- *Latency*: Milliseconds from proxy to hardware

### 3.7.3 Feedback Flow

Execution feedback flows in the reverse direction from trajectory commands:

```
Physical Robots
    ↓ [Execution monitoring]
Robot Drivers
    ↓ [Status messages]
Hardware Controllers
    ↓ [Action feedback]
Trajectory Proxy
    ↓ [Relay feedback]
Client Code
```

**Feedback Information:**

- Current trajectory point
- Position error magnitude
- Time remaining
- Execution state (moving, succeeded, failed)

**Flow Characteristics:**

- *Direction*: Reverse of command flow
- *Frequency*: 10 Hz during execution
- *Purpose*: Enables monitoring and reactive behavior

---

## 3.8 Design Trade-offs and Rationale

### 3.8.1 Centralized vs. Distributed Planning

**Design Choice:** Centralized planning with unified scene

**Trade-offs:**

*Advantages:*
- Complete information: Planner sees all arms' configurations
- Deterministic behavior: Same inputs yield same outputs
- Simpler collision checking: Single scene, single checker
- Standard framework compatibility

*Disadvantages:*
- Computational scaling: Planning time grows with dimensionality
- Single point of failure: Planner failure affects all arms
- Limited scalability: Difficult to extend beyond 2-3 arms

**Rationale:** For dual-arm systems, twelve-dimensional planning remains tractable with modern algorithms. The determinism and completeness benefits outweigh the scalability concerns for the target application.

**Alternative Approach:** Distributed planning where each arm plans independently and coordination is achieved through workspace reservation or reactive collision avoidance. This scales better but can fail to find solutions that centralized planning would discover.

### 3.8.2 Multi-Mode vs. Single-Mode Coordination

**Design Choice:** Three distinct coordination modes

**Trade-offs:**

*Advantages:*
- Flexibility: Can optimize for different task requirements
- Efficiency: No unnecessary coordination overhead for independent tasks
- Clarity: Explicit mode selection makes behavior predictable

*Disadvantages:*
- Complexity: More code to implement and test
- Mode selection burden: Client must choose appropriate mode
- Potential misuse: Selecting wrong mode for task

**Rationale:** Real manipulation scenarios have diverse coordination requirements. Supporting multiple modes enables one system to handle all scenarios efficiently.

**Alternative Approach:** Single coordination mode that attempts to balance all requirements. This would simplify implementation but force all tasks into the same operational model, sacrificing either safety (if loosely coordinated) or efficiency (if tightly coordinated).

### 3.8.3 Active vs. Passive State Aggregation

**Design Choice:** Active aggregation with dedicated component

**Trade-offs:**

*Advantages:*
- Explicit control over timing and synchronization
- Central location for state validation and monitoring
- Cleaner interface for planning components

*Disadvantages:*
- Additional component to implement and maintain
- Potential single point of failure
- Extra message hop adds minimal latency

**Rationale:** The benefits of explicit control and clean interfaces outweigh the cost of an additional component. The dedicated aggregator provides a natural integration point for future enhancements (state validation, anomaly detection, logging).

**Alternative Approach:** Rely on passive aggregation tools or have planning components subscribe to multiple topics. This saves a component but complicates client code and makes synchronization implicit.

### 3.8.4 Link Padding Magnitude

**Design Choice:** 5cm uniform padding on all critical links

**Trade-offs:**

*Advantages:*
- Conservative safety: Accounts for all expected error sources with margin
- Simplicity: Uniform padding is easy to configure and validate
- Predictability: Constant behavior across all configurations

*Disadvantages:*
- Workspace reduction: Some geometrically possible configurations become infeasible
- Conservative for low-risk scenarios: Over-cautious when arms are distant

**Rationale:** Physical systems operating near humans or expensive equipment justify conservative safety margins. The workspace reduction is acceptable given the workspace size relative to typical manipulation distances.

**Alternative Approach:** Adaptive padding based on velocity or distance could reduce workspace constraints while maintaining safety. However, this adds complexity in validation and verification.

### 3.8.5 Fixed vs. Variable Update Rates

**Design Choice:** Fixed publication rates (40 Hz state, 15 Hz TF, 100 Hz control)

**Trade-offs:**

*Advantages:*
- Predictable timing: Components can rely on consistent update intervals
- Simpler synchronization: No need to handle variable-rate inputs
- Resource allocation: Known computational budget

*Disadvantages:*
- Potential inefficiency: Publishing even when state hasn't changed
- Latency variability: Updates may arrive just before or after publication

**Rationale:** Predictable timing simplifies system behavior and enables better performance analysis. The computational cost of fixed-rate publication is negligible on modern hardware.

**Alternative Approach:** Event-driven publication (publish only on state change) would reduce unnecessary messages but complicates temporal reasoning and synchronization.

---

## 3.9 Design Validation Approach

The design was validated through multiple complementary approaches:

**Analytical Validation:** Design decisions were analyzed against requirements to ensure each functional and non-functional requirement is addressed by at least one component. This requirements tracing confirmed design completeness.

**Prototype Implementation:** Early prototypes of critical components (state aggregation, trajectory proxying) were implemented to validate that the design concepts were practically realizable before committing to full implementation.

**Expert Review:** The design was reviewed by domain experts who identified potential issues and suggested improvements, particularly regarding safety margins and coordination mode semantics.

**Simulation Testing:** The design was validated in simulation environments where sensor noise, actuator delays, and dynamic effects could be controlled to validate that the design handles real-world conditions.

The combination of these validation approaches provided confidence that the design would meet objectives before full implementation began.

---

## 3.10 Future Work and Extensibility Considerations

While outside the scope of this thesis, several design considerations support future extensions:

**Performance Optimization:** The current design prioritizes clarity and correctness over maximum performance. Future work could optimize collision matrix computation, implement caching strategies for repeated planning queries, and explore parallel collision checking for better performance on multi-core hardware.

**Failure Handling and Recovery:** The current design detects failures but delegates recovery to higher-level logic. Future extensions could implement automatic retry with relaxed constraints, replanning from current state on execution failure, and graceful degradation modes when one arm fails.

**Extensibility to Additional Arms:** The namespace-based architecture naturally extends to systems with more than two arms. Future work could validate scalability by extending to three or four arm systems and implementing hierarchical coordination where subsets of arms coordinate independently.

**Dynamic Obstacle Handling:** The phase-based execution strategy could be extended with sensor integration for detecting unexpected obstacles, incremental replanning when obstacles are detected, and reactive collision avoidance as a fallback safety mechanism.

**Learning-Based Coordination:** The multi-mode architecture could be extended with learned policies for mode selection based on task characteristics, trajectory optimization through reinforcement learning, and adaptive safety margins learned from execution experience.

These extensions would build upon the foundation established by the current design while addressing scenarios beyond the initial scope.

---

## 3.11 Chapter Summary

This chapter presented the architectural design of MARS, organized around the four fundamental challenges: shared workspace management, task coordination and synchronization, joint state management, and planning with trajectory execution.

The design employs centralized planning with a unified collision scene to provide comprehensive awareness of both manipulators during motion generation. Multi-mode coordination enables the system to adapt between tightly synchronized bimanual tasks, collision-aware independent execution, and fully asynchronous operation. Active state aggregation bridges namespace-isolated hardware interfaces with the unified planning representation through explicit translation components. The planning infrastructure integrates sampling-based algorithms with inverse kinematics and trajectory parameterization to support both joint-space and Cartesian planning.

Key design decisions were justified through analysis of trade-offs and discussion of alternative approaches. The centralized planning approach was chosen for its completeness and determinism despite scalability limitations. Multi-mode coordination was selected to provide flexibility across diverse task requirements despite increased complexity. Active state aggregation provides explicit control and clean interfaces at the cost of an additional component. Conservative link padding ensures safety at the expense of some workspace reduction.

The component-level design details how joint state aggregators, trajectory proxies, motion planner interfaces, and workspace scene managers fulfill their responsibilities within the overall architecture. Data flows through the system were characterized in terms of direction, frequency, and reliability.

The next chapter (Chapter 4: Implementation) describes how these design concepts were realized through specific algorithms, data structures, and implementation patterns. The focus shifts from architectural decisions to technical mechanisms that bring the design to life in a functioning system.
