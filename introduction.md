# Chapter 1: Introduction

## 1.1 Background

Robotic manipulation has evolved significantly from isolated single-arm systems to collaborative multi-arm configurations capable of performing complex tasks in shared workspaces. While single-arm manipulators excel at sequential operations, many real-world scenarios require the dexterity, coordination, and throughput that only multi-arm systems can provide. Industrial assembly lines, warehouse logistics, surgical robotics, and advanced manufacturing increasingly demand robotic systems that can perform bimanual manipulation tasks such as coordinated object handling, synchronized assembly, and collaborative pick-and-place operations.

The transition from single-arm to multi-arm robotic systems introduces fundamental challenges that extend beyond simply deploying multiple robots in proximity. When two or more manipulators operate in overlapping workspaces, several critical problems emerge: how to prevent collisions between moving arms while maximizing workspace utilization, how to coordinate their movements to achieve both synchronized and independent operations, how to manage the combined state of a high-dimensional system in real-time, and how to plan and execute trajectories that satisfy both safety constraints and task requirements.

These challenges become particularly acute in bimanual manipulation scenarios where both arms must work together on a single task. Unlike parallel independent operations where arms work in separate zones, bimanual tasks require precise spatial relationships between end-effectors, temporal synchronization of movements, and continuous awareness of inter-arm collision risks. Traditional approaches that plan for each arm independently and hope they don't interfere are insufficient for such scenarios.

Contemporary multi-arm robotic systems employ various strategies to address these challenges, ranging from centralized planning approaches that treat multiple arms as a single high-degree-of-freedom robot, to distributed coordination schemes where arms negotiate workspace access dynamically. However, the diversity of manipulation tasks means that no single coordination strategy is universally optimal. Some tasks benefit from tight synchronization where both arms move in perfect harmony, while others require complete independence to maximize throughput. Many practical scenarios fall between these extremes, needing collision-aware planning without strict temporal coupling.

This thesis addresses these challenges through the design and implementation of the Multi-Arm Robotic System (MARS), a flexible coordination framework that supports multiple operational modes tailored to different task requirements. By integrating centralized motion planning, multi-mode coordination strategies, and robust state management, MARS enables safe and efficient bimanual manipulation in shared workspaces.

## 1.2 Motivation

The motivation for this work stems from three key observations about the current state of multi-arm robotic systems.

**Limitation of Single-Mode Approaches**

Existing multi-arm systems typically implement a single coordination paradigm, forcing all tasks to conform to one operational model. Systems designed for tight synchronization sacrifice throughput when performing independent tasks, while systems optimized for parallel operation struggle with tasks requiring precise coordination. Real-world applications demand flexibility: a dual-arm system might need to perform synchronized bimanual grasping in one moment, then switch to parallel pick-and-place operations the next. The lack of adaptable coordination strategies limits the practical applicability of multi-arm systems across diverse task portfolios.

**Complexity of High-Dimensional Planning**

Multi-arm systems operate in high-dimensional configuration spaces where naive approaches become computationally intractable. A dual six-degree-of-freedom system creates a twelve-dimensional planning problem where the number of potential collision checks grows quadratically with the number of links. Without careful architectural design and optimization strategies, motion planning becomes prohibitively slow, preventing real-time operation. Furthermore, managing the state of such systems requires sophisticated mechanisms to maintain consistency between the unified planning representation and the physical hardware interfaces.

**Safety in Shared Workspaces**

When manipulators operate in overlapping workspaces, collision avoidance transitions from a desirable feature to a critical safety requirement. Traditional approaches that rely solely on runtime collision detection provide insufficient safety margins for physical systems where modeling errors, execution delays, and mechanical tolerances can lead to unplanned contact. Systems must incorporate proactive safety mechanisms during planning while maintaining the flexibility needed for efficient task execution.

These limitations motivated the development of MARS as a comprehensive solution that addresses coordination flexibility, computational efficiency, and safety simultaneously. Rather than optimizing for a single scenario, MARS provides a framework where the coordination strategy adapts to task requirements, collision avoidance is integrated into the planning pipeline, and state management maintains real-time performance despite the system's complexity.

The practical motivation extends to the growing deployment of collaborative robots in environments where flexibility and safety are paramount. Manufacturing facilities increasingly require reconfigurable systems that can handle product variety, warehouse automation demands high-throughput manipulation with safety guarantees, and research laboratories need platforms for investigating advanced bimanual manipulation strategies. MARS addresses these needs through a principled architecture that balances performance, safety, and flexibility.

## 1.3 Problem Statement

This thesis addresses four fundamental challenges that arise when coordinating multiple robotic manipulators in shared workspaces:

### Challenge 1: Shared Workspace Management

When multiple manipulators operate in overlapping workspace regions, the primary bottleneck for operational efficiency is collision avoidance. Unlike isolated single-arm setups where workspace is exclusive, multi-arm systems require persistent spatial negotiation within a shared work envelope. The core challenge lies in maintaining real-time awareness of every arm's position and projected trajectory to prevent path intersections while maximizing workspace utilization.

This challenge manifests at multiple levels. At the geometric level, the system must maintain accurate collision models for all robot links and detect potential interference throughout planned trajectories. At the computational level, naive pairwise collision checking between all link combinations becomes prohibitively expensive as system complexity grows. At the strategic level, the system must balance safety margins against operational efficiency, ensuring arms maintain adequate separation without unnecessarily constraining their motion.

The workspace management problem is further complicated by the dynamic nature of manipulation tasks. Unlike structured industrial environments where robot paths are predetermined and validated offline, flexible manipulation systems must handle arbitrary goal configurations specified at runtime. The system must guarantee collision-free operation for novel trajectories without excessive computational overhead that would prevent real-time performance.

### Challenge 2: Task Coordination and Synchronization

Multi-arm systems must coordinate not just in space, but in time and purpose. Different manipulation tasks impose vastly different coordination requirements. Bimanual grasping of a single object requires both arms to reach their grasp points simultaneously with precise relative positioning. Sequential handoff operations need temporal ordering but tolerate looser timing constraints. Parallel operations in separate workspace zones benefit from complete independence to maximize throughput.

The coordination challenge encompasses both planning and execution dimensions. During planning, the system must reason about inter-arm relationships appropriate to the task at hand. Some scenarios require unified planning that generates coordinated trajectories for all arms simultaneously, while others benefit from independent planning that treats each arm separately. During execution, the system must maintain appropriate coupling between arm movements, whether through synchronized controller commands, loosely coordinated parallel execution, or completely independent operation.

Existing approaches typically commit to a single coordination paradigm, forcing all tasks into the same operational model regardless of their specific requirements. This inflexibility either constrains performance when tight coordination is unnecessary or fails to achieve the precision needed for truly synchronized operations. The challenge is designing a framework that supports multiple coordination modes and enables appropriate mode selection based on task characteristics.

### Challenge 3: Joint State Management

Multi-arm systems create a state management problem of significant complexity. A dual-arm system with six-degree-of-freedom manipulators operates in a twelve-dimensional joint space. Maintaining a coherent representation of this high-dimensional state while interfacing with physical hardware that operates in separate namespaces requires sophisticated coordination mechanisms.

The state management challenge involves several interconnected problems. First, the system must aggregate joint states from multiple hardware interfaces that publish data independently and potentially asynchronously. Second, it must resolve naming conflicts when multiple arms have identically named joints. Third, it must maintain temporal consistency so that the unified state representation accurately reflects the physical system at each instant. Fourth, it must handle the bidirectional flow of information, aggregating state information from hardware while distributing trajectory commands back to appropriate controllers.

These problems are exacerbated by real-time constraints. Motion planning requires current state information to generate valid trajectories, making stale or inconsistent state data problematic. The state management system must operate at frequencies sufficient to support responsive planning and smooth execution while handling the complexity of multiple asynchronous data sources.

### Challenge 4: Planning and Trajectory Execution

Generating and executing feasible trajectories for multi-arm systems requires addressing challenges at both the task-space and joint-space levels. For task-space goals specified as desired end-effector poses, the system must solve inverse kinematics for high-dimensional configurations where solutions may not be unique or may not exist. For joint-space goals, the system must navigate high-dimensional configuration spaces where collision-free paths are not always obvious.

The planning challenge is compounded by the need to support both individual arm motion and coordinated multi-arm trajectories. Single-arm planning operates in six-dimensional space where standard algorithms perform well, but coordinated planning operates in twelve-dimensional space where the exponential growth of the state space becomes problematic. The system must employ planning algorithms that scale to high dimensions while maintaining real-time performance.

Trajectory execution introduces its own challenges. Planned trajectories must be parameterized with appropriate velocities and accelerations that respect joint limits and dynamic constraints. The system must route trajectory commands to appropriate hardware controllers, potentially splitting unified multi-arm trajectories into arm-specific segments. It must monitor execution progress and handle failures gracefully, whether due to kinematic infeasibility, collision detection, or hardware errors.

Furthermore, the system must support both joint-space planning, where goals are specified as target joint configurations, and Cartesian planning, where goals are specified as target end-effector poses. The latter requires tight integration between motion planning and inverse kinematics, with the system automatically converting pose goals to valid joint configurations before planning.

These four challenges are deeply interconnected. Effective workspace management depends on accurate state information and collision-aware planning. Coordination strategies influence both state management requirements and planning complexity. Planning algorithms must account for workspace constraints and coordinate with execution systems. This thesis presents MARS as an integrated solution that addresses these challenges holistically through a principled architectural design.

## 1.4 Aims and Objectives

### 1.4.1 Overall Aim

The overall aim of this thesis is:

**"To design and implement a flexible multi-mode coordination framework for dual-arm robotic systems operating in shared workspaces, enabling safe and efficient bimanual manipulation tasks."**

This aim addresses the fundamental problem of coordinating multiple manipulators in overlapping workspaces while supporting diverse task requirements through adaptable coordination strategies. The focus on multi-mode coordination reflects the observation that no single operational paradigm is optimal for all manipulation scenarios. By providing flexibility in coordination strategy, integrated collision avoidance mechanisms, and robust state management, MARS enables dual-arm systems to perform both tightly synchronized bimanual tasks and independent parallel operations safely and efficiently.

### 1.4.2 Specific Objectives

The aim is achieved through twelve specific objectives organized around the four challenge areas identified in the problem statement:

#### Objectives for Challenge 1: Shared Workspace Management

**Objective 1.1: Implement Centralized Collision Avoidance**

Develop a unified planning scene that aggregates all joints from both manipulators into a single configuration space, incorporating link padding to create safety margins between robot links during motion planning and execution.

*Success Criteria:*
- Planning scene correctly represents complete multi-arm system geometry
- Safety margins maintain minimum separation between manipulators during all planned motions
- Collision checking validates both self-collision within each arm and inter-arm collisions

**Objective 1.2: Optimize Collision Checking Performance**

Configure collision matrix to disable geometrically impossible collision pairs, reducing computational overhead while maintaining safety guarantees for all reachable configurations.

*Success Criteria:*
- Collision matrix eliminates unnecessary checks for adjacent links and geometrically separated components
- Planning performance scales appropriately with configuration space dimensionality
- All geometrically feasible collision pairs remain actively checked

**Objective 1.3: Validate Collision-Free Operation**

Demonstrate through empirical testing that the system maintains collision-free operation across diverse scenarios including workspace crossing, near-miss trajectories, and complex multi-phase tasks.

*Success Criteria:*
- Test scenarios confirm arms maintain required separation during motion
- Stress tests with intentionally challenging trajectories execute without collision
- Phase-based execution correctly decomposes complex tasks into safe segments

#### Objectives for Challenge 2: Task Coordination and Synchronization

**Objective 2.1: Design Multi-Mode Coordination Architecture**

Implement three distinct coordination modes supporting different temporal and spatial coupling requirements: synchronous mode for tightly coordinated bimanual tasks, hybrid mode for collision-aware planning with flexible execution timing, and asynchronous mode for independent parallel operations.

*Success Criteria:*
- All three coordination modes implemented and operational
- Clear architectural separation enables mode selection at runtime
- Each mode provides appropriate coupling characteristics for its intended use cases

**Objective 2.2: Validate Temporal Coordination Across Modes**

Demonstrate through test scenarios that each coordination mode achieves its intended temporal characteristics, from precise synchronization in synchronous mode to complete independence in asynchronous mode.

*Success Criteria:*
- Synchronous mode produces temporally aligned trajectories where both arms reach waypoints simultaneously
- Hybrid mode executes collision-free trajectories with flexible timing
- Asynchronous mode enables independent arm operation with different task durations

**Objective 2.3: Establish Mode Selection Guidelines**

Develop criteria for selecting appropriate coordination modes based on task characteristics including workspace interaction requirements, temporal coupling needs, and throughput priorities.

*Success Criteria:*
- Guidelines map task requirements to appropriate coordination strategies
- Mode selection criteria consider spatial interaction, timing constraints, and efficiency goals
- Validation scenarios demonstrate appropriate mode selection for representative tasks

#### Objectives for Challenge 3: Joint State Management

**Objective 3.1: Develop State Aggregation System**

Implement thread-safe joint state aggregation that combines namespace-isolated hardware outputs into a unified high-dimensional state vector with bidirectional namespace translation between hardware and planning representations.

*Success Criteria:*
- Unified state vector correctly aggregates joints from all manipulators
- Namespace translation bidirectionally maps between hardware names and planning model
- Thread-safe implementation prevents race conditions during concurrent updates

**Objective 3.2: Implement Trajectory Proxying**

Create trajectory routing mechanism that accepts unified trajectory commands from the planning system and distributes them to appropriate hardware controllers after namespace translation.

*Success Criteria:*
- Trajectory proxy correctly routes commands to appropriate arm controllers
- Namespace stripping converts planning representation to hardware-compatible commands
- Execution monitoring provides feedback on trajectory following progress

**Objective 3.3: Achieve Real-Time State Synchronization**

Integrate state aggregation with system-level state representation to maintain consistent real-time awareness of complete system configuration with acceptable latency for planning and control.

*Success Criteria:*
- State updates published at frequency sufficient for real-time planning
- Latency between hardware state changes and unified state representation remains bounded
- State consistency maintained across all system components

#### Objectives for Challenge 4: Planning and Trajectory Execution

**Objective 4.1: Integrate Motion Planning Infrastructure**

Configure sampling-based motion planning algorithms supporting high-dimensional configuration space exploration with inverse kinematics solvers for converting task-space goals to joint-space configurations.

*Success Criteria:*
- Sampling-based planners successfully navigate high-dimensional configuration spaces
- Inverse kinematics solvers generate valid joint configurations for reachable poses
- Planning pipeline integrates collision checking with trajectory generation

**Objective 4.2: Implement Trajectory Parameterization and Control**

Develop trajectory time parameterization that computes velocity and acceleration profiles respecting joint limits, along with controller architecture enabling both unified multi-arm execution and independent per-arm control.

*Success Criteria:*
- Time parameterization generates smooth trajectories respecting kinematic constraints
- Controller architecture supports both coordinated and independent execution
- Trajectory following achieves acceptable accuracy on physical hardware

**Objective 4.3: Validate Cartesian Coordination Capabilities**

Demonstrate through empirical testing that the system successfully plans and executes both single-arm and bimanual Cartesian goals where end-effector poses are specified in task space rather than joint space.

*Success Criteria:*
- Single-arm Cartesian planning successfully reaches target end-effector poses
- Bimanual Cartesian planning generates collision-free trajectories for coordinated pose goals
- Empirical validation confirms pose accuracy meets manipulation task requirements

### 1.4.3 Scope of Work

The scope of this thesis encompasses the design, implementation, and validation of MARS as a functional dual-arm coordination framework. The work focuses on the fundamental coordination challenges of workspace management, task synchronization, state management, and trajectory planning rather than on higher-level cognitive functions or learning-based adaptation.

**Included in Scope:**

The thesis addresses the complete pipeline from system architecture through implementation to empirical validation. This includes the design of coordination modes and their underlying mechanisms, implementation of collision avoidance strategies through centralized planning, development of state aggregation and trajectory proxying infrastructure, integration of motion planning and inverse kinematics, and comprehensive validation through test scenarios covering both joint-space and Cartesian planning.

The validation includes operation on physical robotic hardware, demonstrating that the implemented system functions reliably in real-world conditions rather than only in simulation. Test scenarios cover representative manipulation tasks including synchronized bimanual operations, collision avoidance in shared workspaces, and coordinated end-effector positioning.

**Excluded from Scope:**

This work does not address dynamic obstacle avoidance beyond the static workspace boundaries defined during initialization. The system employs phase-based execution strategies where complex tasks are decomposed into sequential collision-free segments, rather than implementing real-time replanning in response to moving obstacles.

Higher-level task planning and automated task decomposition are beyond the scope of this work. The system provides coordination primitives that enable bimanual manipulation, but does not include automated reasoning about which subtasks to perform or how to decompose complex goals into executable primitives. Task sequencing is manually specified for validation scenarios.

Learning-based approaches for coordination strategy selection, trajectory optimization, or adaptation to new scenarios are not addressed. The system provides a framework with multiple predefined coordination modes, but mode selection is based on task analysis rather than learned policies.

Force control, compliance, and haptic feedback are excluded from the current implementation, which focuses on position-controlled motion. The system commands joint positions rather than forces, making it suitable for free-space motion and rigid grasping but not for contact-rich manipulation requiring force regulation.

The framework is demonstrated on a dual-arm configuration, but extension to three or more arms, while architecturally feasible through the namespace-based design, is not implemented or validated in this work.

## 1.5 Contributions

This thesis makes the following contributions to the field of multi-arm robotic manipulation:

**Multi-Mode Coordination Framework**

The primary contribution is the design and implementation of a coordination framework that supports multiple operational paradigms within a single architecture. Rather than committing to a single coordination strategy, MARS provides three distinct modes—synchronous, hybrid, and asynchronous—each optimized for different task characteristics. This flexibility enables the same hardware platform to efficiently handle diverse manipulation scenarios without architectural reconfiguration.

**Integrated Collision Avoidance Strategy**

The thesis demonstrates an effective approach to collision avoidance in shared workspaces through the combination of centralized planning, link padding for safety margins, and collision matrix optimization for computational efficiency. This integrated strategy maintains safety guarantees while enabling real-time planning performance in high-dimensional configuration spaces.

**State Management Architecture**

The state aggregation and trajectory proxying mechanisms developed for MARS address the practical challenge of integrating namespace-isolated hardware interfaces with unified planning representations. The bidirectional translation approach and thread-safe implementation provide a template for managing state in complex multi-robot systems.

**Empirical Validation Framework**

The comprehensive validation approach, including test scenarios covering joint-space coordination, Cartesian planning, collision avoidance stress tests, and multi-phase execution, provides a methodical framework for evaluating multi-arm coordination systems. The validation on physical hardware demonstrates real-world applicability beyond simulation-only validation.

**Open Architecture Implementation**

The complete implementation provides a reference architecture for researchers and practitioners developing multi-arm systems, demonstrating how contemporary motion planning frameworks can be effectively configured for dual-arm coordination while maintaining modularity and extensibility.

## 1.6 Thesis Structure

The remainder of this thesis is organized as follows:

**Chapter 2: Literature Review** surveys existing approaches to multi-arm coordination, examining centralized versus distributed planning strategies, synchronous versus asynchronous execution paradigms, collision avoidance techniques, and state management approaches in multi-robot systems. The chapter positions MARS within the broader context of multi-arm manipulation research and identifies the specific gaps that motivated this work.

**Chapter 3: System Design** presents the architectural design of MARS, detailing the rationale behind key design decisions. The chapter describes the multi-mode coordination architecture, collision avoidance strategy, state management pipeline, and planning infrastructure. Design alternatives are discussed along with justifications for the chosen approaches.

**Chapter 4: Implementation** provides detailed technical documentation of the MARS implementation, covering system architecture, state management mechanisms, collision avoidance configuration, coordination mode implementations, planning and execution infrastructure, end-effector control, and system configuration. The chapter includes algorithmic descriptions and implementation patterns demonstrating how the design concepts are realized in practice.

**Chapter 5: Validation and Results** presents empirical validation of the implemented system through comprehensive testing. The chapter describes test scenarios for each challenge area, presents quantitative and qualitative results, analyzes system performance across different coordination modes, and demonstrates operation on physical hardware. Validation results are evaluated against the success criteria defined for each objective.

**Chapter 6: Discussion** reflects on the results, discussing the strengths and limitations of the implemented system, comparing the three coordination modes, analyzing trade-offs in the design, and examining cases where the system performs well versus scenarios that expose limitations. The chapter also discusses broader implications for multi-arm robotics and lessons learned during development.

**Chapter 7: Conclusion** summarizes the thesis contributions, reviews the achievement of stated objectives, and proposes directions for future work. The chapter reflects on how MARS addresses the identified challenges and discusses potential extensions including dynamic obstacle handling, learning-based coordination, and scaling to systems with more than two arms.

The appendices provide supplementary material including detailed configuration files, additional test results, and technical specifications of the hardware platform.
