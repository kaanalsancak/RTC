# RTC
Real-Time-Control
# ⚙️ Control Laws – Wiki Overview

Welcome to the Control Laws section of the Control Systems Wiki.  
This section categorizes modern and classical control techniques based on the mathematical law or strategy they implement.

---

## 📚 Categories

### 1. [Classical Controllers](./1-Classical-Controllers/)
Traditional controllers based on error correction:
- Proportional (P)
- Proportional-Integral (PI)
- Proportional-Derivative (PD)
- Proportional-Integral-Derivative (PID)

### 2. [Optimal Control](./2-Optimal-Control/)
Controllers that minimize a cost function:
- Linear Quadratic Regulator (LQR)
- Linear Quadratic Gaussian (LQG)
- Model Predictive Control (MPC)

### 3. [State-Space Control](./3-State-Space-Control/)
Modern control strategies using system states:
- State Feedback & Pole Placement
- Observer-Based Controllers

### 4. [Robust Control](./4-Robust-Control/)
Designed for uncertain systems:
- H-Infinity (H∞) Control
- μ-Synthesis (Mu-Synthesis)

### 5. [Adaptive Control](./5-Adaptive-Control/)
Controllers that modify their behavior in real-time:
- Model Reference Adaptive Control (MRAC)
- Self-Tuning Regulator (STR)
- Gain Scheduling

### 6. [Nonlinear Control](./6-Nonlinear-Control/)
Designed for systems with nonlinear behavior:
- Feedback Linearization
- Sliding Mode Control
- Backstepping

### 7. [Intelligent Control](./7-Intelligent-Control/)
AI-driven control strategies:
- Fuzzy Logic
- Neural Networks
- Reinforcement Learning
- Genetic Algorithms / Swarm Optimization

### 8. [Hybrid & Event-Based Control](./8-Hybrid-and-Event-Based-Control/)
Used in cyber-physical and real-time systems:
- Event-Triggered Control
- Switched Systems
- Hybrid Control

---

## 📌 Usage

This wiki is useful for:
- Students and researchers in Control Theory
- Developers working on embedded, robotic, or automation systems
- Anyone exploring control design techniques

Start exploring by diving into any category above 👆

# File Structure
Control-Laws/
├── README.md
├── 1-Classical-Controllers/
│   ├── P-Control.md
│   ├── PI-Control.md
│   ├── PD-Control.md
│   └── PID-Control.md
├── 2-Optimal-Control/
│   ├── LQR.md
│   ├── LQG.md
│   └── Model-Predictive-Control-MPC.md
├── 3-State-Space-Control/
│   ├── State-Feedback-Pole-Placement.md
│   └── Observer-Based-Control.md
├── 4-Robust-Control/
│   ├── H-Infinity-Control.md
│   └── Mu-Synthesis.md
├── 5-Adaptive-Control/
│   ├── MRAC.md
│   ├── Self-Tuning-Regulator.md
│   └── Gain-Scheduling.md
├── 6-Nonlinear-Control/
│   ├── Feedback-Linearization.md
│   ├── Sliding-Mode-Control.md
│   └── Backstepping.md
├── 7-Intelligent-Control/
│   ├── Fuzzy-Logic-Control.md
│   ├── Neural-Network-Control.md
│   ├── Reinforcement-Learning-Control.md
│   └── Evolutionary-Algorithms.md
└── 8-Hybrid-and-Event-Based-Control/
    ├── Event-Triggered-Control.md
    ├── Switched-Control.md
    └── Hybrid-Control.md
