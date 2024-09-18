// JavaScript for handling syllabus display
function openSyllabus(subjectId) {
    var modal = document.getElementById("syllabusModal");
    var subjectTitle = document.getElementById("subjectTitle");
    var subjectUnits = document.getElementById("subjectUnits");

    // Set the modal to visible
    modal.style.display = "flex";

    // Clear previous syllabus content
    subjectUnits.innerHTML = '';

    // Based on the subjectId, set the subject title and syllabus units
    if (subjectId === 'COMPUTER COMMUNICATION NETWORKS') {
        subjectTitle.innerHTML = "COMPUTER COMMUNICATION NETWORKS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit1')">Unit 1</a>
                    <div id="unit1" class="unit-syllabus">Classification of Computer Networks, Network Architecture, Network Applications. Computer Network Types, Network LAN Technologies, Computer Network Topologies, Network Security.
                     Network Models: Protocol Layering, Layered Tasks, OSI Model, TCP/IP Protocol Suite, Comparison of the OSI and TCP/IP Reference models.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit2')">Unit 2</a>
                    <div id="unit2" class="unit-syllabus">Signals, Transmission Impairment: Guided Media, Unguided Media. Channel Capacity: Bandwidth, Error-rate, Encoding, Transmission Modes.
                    Multiplexing: Frequency Division Multiplexing, Time Division Multiplexing, Wavelength Division Multiplexing, Code Division Multiplexing, Network Switching: Circuit Switching, Message Switching, Packet Switching, Connecting Devices: Hubs, Switches, Routers and Gateways.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit3')">Unit 3</a>
                    <div id="unit3" class="unit-syllabus">Data-Link Layer: Introduction: Nodes and Links, Services, Categories of link, Sub layers, Link Layer addressing: Types of addresses, ARP. Data Link Layer Protocols: Simple Protocol, Stop and Wait protocol, Piggybacking.
Access Control: Random Access: ALOHA, CSMA, CSMA/CD, CSMA/CA. Controlled Access: Reservation, Polling, Token Passing. Wired LANs: Ethernet, Wireless LANs.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit4')">Unit 4</a>
                    <div id="unit4" class="unit-syllabus">Network Layer: Introduction, Packetizing, Routing and Forwarding, Other services, Packet Switching: Datagram Approach, Virtual Circuit Approach.
IPv4 Addresses: Address Space, Classful Addressing, Classless Addressing, IPv6 Addressing, IPv4 to IPv6 Transition, DHCP.
Network Layer Protocols: Internet Protocol (IP): Datagram Format, Fragmentation, ICMPv4: Messages, Routing Algorithms: Distance Vector Routing, Link State Routing, Routing Information Protocol (RIP)</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit5')">Unit 5</a>
                    <div id="unit5" class="unit-syllabus">Transport Layer: Introduction: Transport Layer Services, Connectionless and Connection Oriented Protocols,
Transport Layer Protocols: Simple protocol, Stop and wait protocol, Go-Back-N Protocol, Selective repeat protocol, User Datagram Protocol: User Datagram, UDP Services, UDP Applications, Transmission Control Protocol: TCP Services, TCP Features, Flow control, Error control, TCP congestion control.
Application Layer: Introduction, Client Server Model, Protocols: DNS, SMTP, FTP, POP, HTTP.</div>
                </li>
            </ul>`;
    } else if (subjectId === 'MICROWAVE ENGINEERING') {
        subjectTitle.innerHTML = "MICROWAVE ENGINEERING - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Microwave Transmission Lines : Introduction, Microwave Spectrum and Bands, Applications of Microwaves.
Waveguide Components and Applications — Solution of Wave Equations in Rectangular Coordinates, TE/TM mode analysis, Expressions for Fields, Characteristic Equation and Cut-off Frequencies, Filter Characteristics, Dominant and Degenerate Modes, Sketches of TE and TM mode fields in the cross-section, Mode Characteristics — Phase and Group Velocities, Wavelengths and Impedance Relations, Impossibility of TEM Mode, Illustrative Problems. Micro strip Lines— Introduction, Zo Relations, Effective Dielectric Constant, Losses, Q factor.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Cavity Resonators — Introduction, Rectangular Cavities, Q Factor and Coupling Coefficients, Illustrative Problems
Multiplexing: Coupling Mechanisms — Probe, Loop, Aperture types. Waveguide Attenuators — Different Types, Rotary Vane Attenuators; Waveguide Phase Shifters — Types, Dielectric and Rotary Vane Phase Shifters, Waveguide Multiport Junctions — E plane and H plane Tees, Magic Tee. Directional Couplers — 2 Hole, Bethe Hole types, Illustrative Problems. Scattering Matrix— Significance, Formulation and Properties, S Matrix Calculations for — 2 port Junctions, E plane and H plane Tees, Magic Tee, Circulator and Isolator, Illustrative Problems. Ferrites— Composition and Characteristics, Faraday rotation, Ferrite Components — Gyrator, Isolator, Circulator.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">O-Type Microwave Tubes O Type and M Type Classifications, Limitations.
Two Cavity Klystrons — Structure, Re-entrant Cavities, Velocity Modulation Process and Applegate Diagram, Bunching Process and Small Signal Theory — Expressions for output Power and Efficiency.
Reflex Klystrons — Structure, Velocity Modulation and Applegate Diagram, Mathematical Theory of Bunching, Power Output, Efficiency, Oscillating Modes and O/P Characteristics, Effect of Repeller Voltage on Power O/P, Illustrative Problems.
TWTs – Significance, Types and Characteristics of Slow Wave Structures; Structure of TWT and Amplification Process (qualitative treatment), Suppression of Oscillations, Gain Considerations.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">M-Type Microwave Tubes : Introduction, Cross-field Effects, Magnetrons — Different Types, Cylindrical Traveling Wave Magnetron, Hartre Conditions, Modes of Resonance and P1-Mode Operation, Separation of P1-Mode, O/P characteristics, Illustrative Problems.
Microwave Solid State Devices : Introduction, Classification, Applications. TEDs — Introduction, Gunn Diodes — Principle, Characteristics, Basic Modes of Operation – Gunn Oscillation Modes, LSA Mode. PIN Diode (Quality treatment)</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Microwave Measurements : Description of Microwave Bench — Different Blocks and their Features, Errors and Precautions, Microwave Power Measurement, Bolometer Measurement of Attenuation, Frequency Standing waveMeasurements — Measurement of Low and High VSWR, Cavity Q, Impedance Measurements. Gain and Radiation pattern measurement.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'DESIGN FOR TESTABILITY') {
        subjectTitle.innerHTML = "DESIGN FOR TESTABILITY - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">BASICS OF TESTING AND FAULT MODELING: Introduction- Principle of testing - types of testing - DC and AC parametric tests - fault modeling. Stuck-at fault - fault equivalence - fault collapsing - fault dominance - fault simulation.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">TESTING AND TESTABILITY OF COMBINATIONAL CIRCUITS Test generation basics - test generation algorithms - path sensitization - Boolean difference – Algorithm – PODEM - Testable combinational logic circuit design.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">TESTING AND TESTABILITY OF SEQUENTIAL CIRCUITS Testing of sequential circuits as iterative combinational circuits - state table verification – test generation based on circuit structure - Design of testable sequential circuits - Ad Hoc design rules - scan path technique (scan design) - partial scan - Boundary scan.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">MEMORY, DELAY FAULT AND IDDQ TESTING Testable memory design - RAM fault models - test algorithms for RAMs – Delay faults – Delay test- IDDQ testing - testing methods - limitations of IDDQ testing.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">BUILT-IN SELF-TEST Test pattern generation of Built-in Self-Test (BIST) - Output response analysis – BIST architectures.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'ROBOTICS') {
        subjectTitle.innerHTML = "ROBOTICS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Introduction: Brief History, Types and applications of robots. Present status and future trends in robotics, Overview of robot subsystems. Challenges in robotics, Characteristics of robots, Robot configurations and concept of work space</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Sensors & Actuators Sensors-Position sensors, Velocity sensors, Proximity sensors, Touch and Slip Sensors, Force and Torque sensors. Actuators-DC Motors, Stepper Motors, Servomotors, Relay, Pistons. Grippers and Manipulators Gripper joints, Gripper force, Serial manipulator, Parallel Manipulator, selection of Robot-Selection based on the Application</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Mechanical Aspects Introduction to Manipulator Kinematics, Position and orientation of rigid bodies, Planar and spatial mechanism description, Homogenous transformations, Denavit - Hardenberg (DH) notation, Forward and inverse kinematic analysis, Examples.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Linear and rotational velocity of rigid bodies, Velocity propagation from link to link, Jacobians, Singularities, Static forces in manipulators, Jacobians in force domain, Cartesian transformation of velocities and static forces. Examples.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Trajectory Generation, General consideration in path description and generation, Joint space schemes, Collision free path planning, Robot programming. Robot Control, Independent joint control, PD and PID feedback, Issues in nonlinear control, Examples.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'SATELLITE COMMUNICATION AND NAVIGATION TECHNOLOGY') {
        subjectTitle.innerHTML = "SATELLITE COMMUNICATION AND NAVIGATION TECHNOLOGY - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Kepler’s law, Orbital elements, Orbit perturbations, Geostationary and non-geostationary constellation, Antenna look angles, EIRP, Transmission Losses, Link power Budget equation, System Noise, Carrier to Noise ratio, Inter modulation Noise</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Concept of Space segment and Earth segment, Altitude control, TT&C subsystems, Antenna subsystem, Transponders, Architecture of Satellite hub and Satellite terminal, Cellular backhaul, Satellite constellations, overview and concept of broadcasting satellites, such as DVB-S2X, SD and HDTV, orbital positions, Transponder capacity, frequency bands for satellite broadband broadcast services, VSAT</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Compare Single Access and Multiple Access techniques, Pre assigned FDMA and TDMA, Demand Assigned FDMA and TDMA, Preamble, Post amble, Carrier recovery, Network synchronization, CDMA throughput, Downlink analysis</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">GPS principle of operation, architecture, C/A and P-Code, GPS/INS, GPS/pseudo lite, GPS/cellular, RINEX navigation and observation formats, Ambiguity resolution, principle of operation of DGPS, GLONASS and Galileo IRNS system, Concepts of Wide area augmentation system (WAAS) and Local area augmentation system (LAAS)</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Radar Block diagram, Radar range equation, Pulse repetition frequency Automatic directional finder, Introduction to VOR and ILS, Radar transmitter, Linear beam power tubes, Solid State RF power sources, Super heterodyne Receiver, Dynamic Range, Matched filter</div>
                </li>
            </ul>`;
    }else if (subjectId === 'DESIGN FOR REINFORCEMENT LEARNING AND RECOMMENDER SYSTEMS') {
        subjectTitle.innerHTML = "DESIGN FOR REINFORCEMENT LEARNING AND RECOMMENDER SYSTEMS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">The Reinforcement Learning Problem- History of Reinforcement Learning, Reinforcement Learning, Elements of Reinforcement Learning, Limitations and Scope, An Extended Example: Tic-Tac-Toe, Eligibility Traces, TD Gammon, Samuel’s Checkers Player</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Q-learning algorithm, SARSA algorithm, Bellman optimality equations, relationship between Q-learning and the Bellman optimality equations, Value Function Approximation, on-policy and off-policy methods for prediction and control.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Markov decision process, reinforcement learning problem, Anatomy of a RL algorithm, RL algorithm types.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Introducing Recommender Systems- types of recommender systems, two systems heavily dependent on recommender technology: MovieLens and Amazon.com.
Non-Personalized and Content-Based techniques for non- and lightly-personalized recommendations, meaningful summary statistics, basic techniques for content-based filtering and then explore a variety of advanced interfaces and content-based computational techniques being used in recommender systems</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Fundamental techniques for making personalized recommendations, user-user collaborative filtering, variations of the user-user algorithm, and the benefits and drawbacks of the general approach, item-item collaborative filtering algorithm.
Offline Evaluation, Online Evaluation, several families of metrics, measure prediction accuracy, rank accuracy, decision-support, and other factors such as diversity, product coverage, and serendipity.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'SYSTEM MODELLING AND SIMULATION') {
        subjectTitle.innerHTML = "SYSTEM MODELLING AND SIMULATION - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Introduction to simulation, Advantages, Disadvantages, Areas of application, System environment, components of a system, Model of a system, Discrete and continuous systems, Types of models, steps in a simulation study., Discrete event system simulation. Simulation Examples: Simulation of Queuing systems, Simulation of Inventory System, Other simulation examples</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Random Number Generation: eneral Principles: Concepts in discrete - event simulation, event scheduling/ Time advance algorithm, simulation using event scheduling. Random Numbers: Properties, Generations methods, Techniques for generation of random numbers, Tests for Random number- Frequency test, Runs test.
Random Variate Generation eneral Principles: Concepts in discrete - event simulation, event scheduling/ Time advance algorithm, simulation using event scheduling. Random Numbers: Properties, Generations methods, Techniques for generation of random numbers, Tests for Random number- Frequency test, Runs test.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Statistical Models in Simulation Review of terminology and concepts, Useful statistical models, Discrete distributions. Continuous distributions, Poisson process, Empirical distributions.
Queuing ModelsCharacteristics of queuing systems, Queuing notation, Long-run measures of performance of queuing systems, Long-run measures of performance of queuing systems.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Input Modelling: Data collection, Identification and distribution with data, parameter estimation, Goodness of fit tests, Selection of input models without data, Multivariate and time series input model/analysis.
Verification and validation of model Model building, Verification, Calibration and Validation of models.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Output Analysis – Types of Simulations with respect to output Analysis, Stochastic nature of output data, Measures of performance and their estimation, Output analysis of terminating simulation, Output analysis of steady state simulations.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'LOW POWER VLSI') {
        subjectTitle.innerHTML = "LOW POWER VLSI - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Needs for Low Power VLSI and Sources of power dissipation: Needs for Low Power VLSI, Charging and Discharging Capacitance, Short-circuit Current in CMOS Circuit, CMOS Leakage Current, Static Current, Principles of Low Power Design, Low Power Figure of Merits.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Simulation Power Analysis: SPICE Circuit Simulation, Discrete Transistor Modeling and Analysis-Tabular Transistor Model and Switch Level Analysis, Gate-level Logic Simulation, Architecture-level Analysis-Power Models Based on Activities, Power Model Based on Component Operations and Abstract Statistical Power Models, Data Correlation Analysis in DSP Systems- Data Correlation Analysis in DSP Systems-Dual Bit Type Signal Model and Data path Module Characterization and Power Analysis, Monte Carlo Simulation</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Circuit Level Power Reduction Techniques: Transistor and Gate Sizing-Sizing an Inverter Chain, Transistor and Gate Sizing for Dynamic Power Reduction and Transistor Sizing for Leakage Power Reduction, Equivalent Pin Ordering, Network Restructuring and Reorganization-Transistor Network Restructuring, Partitioning and Reorganization, Special Latches and Flip-flops-Self-gating, Combinational and Double Edge Triggered Flip-flops, Low Power Digital Cell Library-Cell Sizes and Spacing and Varieties of Boolean Functions, Adjustable Device Threshold Voltage</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">ILogic Level Power Reduction Techniques: Gate Reorganization, Signal Gating, Logic Encoding- Binary versus Gray Code and Bus Invert Encoding, State Machine Encoding- Transition Analysis, Output Don't-care Encoding and Design Trade-offs in State Machine Encoding-computation Logic- Pre-computation Condition, Alternate Pre-computation Architectures and Design Issues in Pre-computation Logic Technique</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Architecture and System Level Power Reduction Techniques: Power and Performance Management- Microprocessor Sleep Modes, Performance Management and Adaptive Filtering, Switching Activity Reduction-Guarded Evaluation, Bus Multiplexing and Glitch Reduction by Pipelining, Parallel Architecture with Voltage Reduction. Special Techniques-Power Reduction in Clock Networks-Clock Gating techniques, Other Clock Power Reduction Techniques.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'LINUX PROGRAMMING & DEVICE DRIVERS') {
        subjectTitle.innerHTML = "LINUX PROGRAMMING & DEVICE DRIVERS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Linux utilities: A brief history of UNIX, architecture and features of UNIX, introduction to vi editor. General purpose utilities, file handling utilities, security by file permissions, process utilities, disk utilities, networking commands; Text processing and backup utilities: Text processing utilities and backup utilities; SED: Scripts, operation, addresses, commands; AWK: Execution, fields and records, scripts, operation, patterns, actions, associative arrays, string and mathematical functions, system commands in awk, applications</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Shell: Shell responsibilities, types of shell, pipes and i/o redirection, shell as a programming language, here documents, running a shell script, the shell as a programming language, shell meta characters, file name substitution, shell variables, command substitution, shell commands, quoting, test command, control structures, arithmetic in shell, interrupt processing, functions, and debugging scripts; File structure and directories: Introduction to file system, file descriptors, file types, file system structure; File metadata: Inodes; System calls for file I/O operations: open, create, read, write, close, lseek, dup2, file status information-stat family; File and record locking: fcntl function, file permissions, file ownership, links;</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Process: Process identifiers, process structure: process table, viewing processes, system processes, process scheduling; Starting new processes: Waiting for a process, process termination, zombie processes, orphan process, system call interface for process management, fork, vfork, exit, wait, waitpid, exec. Signals: Signal functions, unreliable signals, interrupted system calls, kill, raise, alarm, pause, abort, system, sleep functions, signal sets.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Data Management: Managing memory: malloc, free, realloc, calloc; File locking: Creating lock files, locking regions, use of read and write with locking, competing locks, other lock commands, deadlocks; Inter process communication: Pipe, process pipes, the pipe call, parent and child processes, named pipes, semaphores, shared memory, message queues; Shared memory: Kernel support for shared memory, APIs for shared memory, shared memory example; Semaphores: Kernel support for semaphores, APIs for semaphores, file locking with semaphores</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Linux Kernel: Roles of Kernel, Kernel Programming, Configuring, Compiling and booting the Linux kernel Device Drivers: What is Device Driver, Types of Device Drivers, Classes of Device Drivers, System Programming Vs Device Drivers, Driver Programming, Role of Device Drivers, Example</div>
                </li>
            </ul>`;
    }else if (subjectId === 'RF AND MIXED SIGNAL CIRCUITS') {
        subjectTitle.innerHTML = "RF AND MIXED SIGNAL CIRCUITS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Electromagnetic spectrum and Microwave bands, RF behaviour of Passive components: Tuned resonant circuits, Resistors, Inductors and Capacitors - Voltage and Current in capacitor circuits, Tuned RF / IF Transformers, MOS Device review</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Examples of transmission lines, Transmission line equations and Biasing, Micro Strip Transmission Lines, Special Termination Conditions, Sourced and Loaded Transmission Lines, single and multiport networks and its properties</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Impedance matching using discrete components, Micro strip line matching networks, Amplifier classes of Operation and Biasing networks</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Filter Basics, Lumped filter design, Distributed Filter Design, Diplexer Filters, Crystal and Saw filters, Active Filters, Tunable filters, Power Combiners / Dividers, Directional Couplers, Hybrid Couplers, Isolators, RF Diodes, BJTs, FETs, HEMTs models, Design of mixers at GHz range, Mixed signal IC issues</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">RF Transistor Amplifier Design: Characteristics of Amplifiers – Amplifier Circuit Configurations, Amplifier Matching Basics, Stability Considerations, Small Signal amplifier design, MMIC amplifiers, Low noise amplifiers, VGA Amplifiers, Synchronizers</div>
                </li>
            </ul>`;
    }else if (subjectId === 'SEQUENCE MODELS') {
        subjectTitle.innerHTML = "SEQUENCE MODELS - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Revision of Linear Algebra, Probability, Convex Optimization, More Optimization (SGD), From Frequency to Meaning: Vector Space Models of Semantics, Advanced word vector, representations: language models, single layer networks</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Recurrent neural networks - for parsing, for different tasks (e.g. sentiment analysis), backpropagation algorithm, chain rule, Feedforward Neural Networks, The Vanishing Gradient Problem, Jacobian matrix, backpropagation through time.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Long Short-Term Memory, Gated Feedback Recurrent Neural Networks, Empirical Evaluation of Gated Recurrent Neural Networks on Sequence Modelling, Learning Word Embeddings- Word2Vec.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Image captioning, video captioning, Video to text, real-time screen description, vision question and answering, One-Shot Video Object Segmentation, case study.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Basic Models, Beam Search, Attention Model Intuition, Attention Model, Trigger Word Detection, DALL.E creating Images from Text, Image transformers, vision transformers.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'FIBER OPTIC AND LASER INSTRUMENTATION') {
        subjectTitle.innerHTML = "FIBER OPTIC AND LASER INSTRUMENTATION - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Construction of Optical Fiber Cable, Guiding Mechanism, Components of Optical Fiber Communication, Principles of light propagation, Acceptance angle, Total Internal Reflection (TIR), skew mode and Numerical aperture, Types of Fiber, single mode, multimode Fiber, graded index, step index Fibers,</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Mechanical and Transmission Characteristics, Scattering losses, Absorption losses, Dispersion, Connectors and Splicers, Fiber Termination, Optical Sources, Light Emitting Diodes, Optical Detectors: PIN Diode.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Introduction to Fiber optic sensor, Types of Fiber optic sensor, Temperature/Pressure Sensor, Intrinsic sensor, Extrinsic sensor, Displacement Sensor and Phase Modulated sensor, Fiber optics instrumentation system, Optical Domain Reflectors, Fiber Scattering Loss Measurement, Measurement of attenuation, Fiber dispersion Measurement, Fiber Absorption Measurement, Near Field Scattering techniques and End Reflection Method, Types of Modulators.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Level of Lasers, Two-Level Laser, Three Level Laser, Quasi Three and four level lasers, Properties of laser, Divergence, Coherence, Mono chromacity and Brightness and Directionality, Modes of lasers, Types of lasers, Solid lasers, Gas lasers, liquid lasers and Semiconductor lasers.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Laser for measurement of distance, velocity, length, Types of LIDAR, Construction, Working, and LIDAR Applications, Material Processing, Laser Trimming. Medical applications of lasers, Laser instruments for surgery, removal of tumors of vocal cards, brain surgery, plastic surgery, gynecology and oncology.</div>
                </li>
            </ul>`;
    }else if (subjectId === 'INFORMATION THEORY AND CODING') {
        subjectTitle.innerHTML = "INFORMATION THEORY AND CODING - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">Unit 1</a>
                    <div id="unit21" class="unit-syllabus">Concept of amount of information and its properties, Average information, Entropy and its properties, Information rate, Mutual information and its properties, Classification of Channels-Binary symmetric Channel and Binary Erasure Channel, Channel Matrices for different Channels.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit22')">Unit 2</a>
                    <div id="unit22" class="unit-syllabus">Shannon-Hartley Theorem, Capacity of a Gaussian channel, Shannon’s Limit, Tradeoff between bandwidth and SNR, Introduction to source coding, Shannon’s source coding theorem, Kraft inequality, Fixed-Length Coding, Shannon Fano coding, Huffman coding, Coding efficiency calculations, Data Compression Techniques-Run length coding, Lempel Ziv algorithm and Arithmetic coding</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit23')">Unit 3</a>
                    <div id="unit23" class="unit-syllabus">Introduction to channel coding, Classification of channel coding techniques-Error detection and correction codes, Systematic and Non-systematic codes, Matrix description of Linear Block codes, Encoding using Generator Matrix, Syndrome Calculation, Decoding of linear block codes, Error detection and error correction capabilities of linear block codes.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit24')">Unit 4</a>
                    <div id="unit24" class="unit-syllabus">Introduction, Polynomial Representation of Code words, Generator Polynomial, Systematic cyclic codes, Encoder design, Syndrome Calculation, Error Detection, Decoder design, and Limitations of Cyclic Codes.</div>
                </li>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit25')">Unit 5</a>
                    <div id="unit25" class="unit-syllabus">Introduction, Encoder Design, Encoding-Time Domain and Transform domain approach, Graphical approach: Code tree, Trellis and State diagram, Decoding of Convolutional Codes- Viterbi algorithm, Sequential Decoding, Advantages and Limitations of Convolutional codes, Comparison of Block codes and convolutional codes</div>
                </li>
            </ul>`;
    }
    else if (subjectId === 'COMPUTER COMMUNICATION NETWORKS LAB') {
        subjectTitle.innerHTML = "COMPUTER COMMUNICATION NETWORKS LAB - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">List of Experiments</a>
                    <div id="unit21" class="unit-syllabus">
                    <ol>
                    <li>Installation of Open-source Graphical Network Simulation software</li>
                    <li>Study of LAN topologies, network devices and network tools</li>
                    <li> Setup a network and configure IP addressing, subnetting.</li>
                    <li>Simulate the transmission of ping message over different network topology.</li>
                    <li>Create a network to perform static routing without subnetting.</li>
                    <li>Create a network to perform static routing with subnetting</li>
                    <li>Implementation and study of ARP protocol</li>
                    <li>Implementation of dynamic routing using RIP</li>
                    <li> Implementation of dynamic routing using EIGRP</li>
                    <li>Create a network to perform standard accesses list</li>
                    <li>Create a network to perform extended standard accesses list</li>
                    <li>Design and setup VLAN routing with one router.</li>
                    </div>
                </li>
                
            </ul>`;
    }
    else if (subjectId === 'MICROWAVE ENGINEERING LAB') {
        subjectTitle.innerHTML = "MICROWAVE ENGINEERING LAB - Syllabus";
        subjectUnits.innerHTML = `
            <ul>
                <li><a href="javascript:void(0)" onclick="toggleUnit('unit21')">List of Experiments</a>
                    <div id="unit21" class="unit-syllabus">
                    <ol>
                    <li>Mode Characteristics of Reflex Klystron</li>
                    <li>Gunn diode Characteristics and power measurement</li>
                    <li> Microwave Frequency and wavelength Measurement</li>
                    <li> Measurement of Attenuation for fixed and variable attenuator</li>
                    <li> Measurement of VSWR and Impedance of a given Load</li>
                    <li>Measurement of Scattering Parameters of Directional Coupler</li>
                    <li> Measurement of Scattering Parameters of a Magic Tee</li>
                    <li>Measurement of Scattering Parameters of Circulators and Isolators</li>
                    <li>  Measurement of radiation pattern and Gain of a Horn Antenna</li>
                    <li>Demonstration of Vector Network Analyzer</li>
                    <li>Demonstration of Anechoic Chamber.</li>
                    
                    </div>
                </li>
                
            </ul>`;
    }
    // Add more subject blocks with unit details similarly
}

function toggleUnit(unitId) {
    var unitContent = document.getElementById(unitId);
    if (unitContent.style.display === "block") {
        unitContent.style.display = "none";
    } else {
        unitContent.style.display = "block";
    }
}

function closeSyllabus() {
    var modal = document.getElementById("syllabusModal");
    modal.style.display = "none";
}

// Close the modal when clicking outside of it
window.onclick = function(event) {
    var modal = document.getElementById("syllabusModal");
    if (event.target == modal) {
        modal.style.display = "none";
    }
}
