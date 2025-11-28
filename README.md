# Collapsed Structure Search Rover (CSSR)

An autonomous multi-sensor robotic system designed to enhance body detection capabilities in disaster search and rescue operations.

## 🎯 Project Overview

This prototype addresses critical challenges in Collapsed Structure Search and Rescue (CSSR) operations by deploying autonomous robotics equipped with thermal and ultrasonic sensing technology. Traditional manual search methods are time-consuming and hazardous; this system significantly improves detection accuracy and reduces operational risk.

**Key Achievement:** 95% detection accuracy in controlled environments with 1-2m operational range and 1-hour mission endurance.

---

## 🚀 Features

### Core Capabilities
- **Multi-Sensor Fusion:** Integrated thermal imaging and ultrasonic sensing for reliable body detection
- **Autonomous Navigation:** Obstacle avoidance and systematic path planning through debris-filled environments
- **Real-Time Data Transmission:** WiFi/LoRa-based wireless communication for live telemetry and decision support
- **Extended Mission Duration:** 1-hour continuous operation per charge with optimized power management

### Technical Specifications
- **Detection Range:** 1-2 meters for thermal signatures; 2cm-4m for ultrasonic obstacles
- **Thermal Accuracy:** 95% in controlled tests; 70% in outdoor simulations (accounting for ambient temperature variation)
- **Obstacle Avoidance:** 90% success rate in controlled environments; 80% in complex debris fields
- **Navigation Speed:** 0.5 m/s average (optimized for sensor data accuracy)
- **Coverage Efficiency:** ~5 minutes per 20 m² in open environments; 50-70% longer in dense rubble

---

## 🔧 Hardware Architecture

### Primary Components

| Component | Model/Type | Purpose |
|-----------|-----------|---------|
| **Microcontroller** | ESP32 | Central processing unit (WiFi + Bluetooth enabled) |
| **Thermal Sensor** | FLIR Lepton | Heat signature detection for body location |
| **Ultrasonic Sensors** | HC-SR04 | Distance measurement and obstacle detection |
| **Mobility** | Tracked Chassis + DC Motors | Rugged terrain navigation with superior traction |
| **Motor Driver** | L298N Dual H-Bridge | Speed/direction control via PWM |
| **Power System** | 12V Li-Ion Battery (2Ah) | 1-hour continuous operation |
| **Communication** | ESP8266/LoRa Module | Real-time data transmission (range: 15km LoRa; WiFi for short-range) |
| **Vision** | Raspberry Pi Camera V2 (optional) | Visual confirmation and live video feed |

### Custom Integration
- Sensor interface board for seamless component interconnection
- Battery Management System (BMS) to prevent overcharging/thermal issues
- GPIO pin mapping for sensor/motor control via ESP32

---

## 📊 Performance Metrics

### Detection Performance
- **Thermal Detection Accuracy:** 95% (controlled); 70% (outdoor with thermal interference)
- **False Positive Mitigation:** Data fusion approach minimizes erroneous alerts
- **Ultrasonic Ranging Precision:** ±0.5cm at 1m distance

### Autonomous Operations
- **Path Planning Algorithm:** Systematic grid-based area coverage
- **Environmental Coverage:** 20 m² scanning area in ~5 minutes (uncluttered)
- **Terrain Adaptability:** Effective on uneven rubble, concrete, and metal surfaces

### Power & Endurance
- **Battery Life:** ~1 hour continuous operation
- **Power Optimization:** Intermittent sensor cycling extends runtime by 10-15%
- **Peak Power Draw:** Motors + thermal sensor (main contributors)

---

## 🛠️ Software Implementation

### Algorithms
1. **Thermal Detection:** Temperature threshold filtering with body-signature pattern recognition
2. **Obstacle Avoidance:** Real-time ultrasonic distance monitoring with reactive steering
3. **Path Planning:** Autonomous grid traversal with waypoint navigation
4. **Data Fusion:** Multi-sensor input processing for confident body localization

### Communication Protocol
- **Short-Range:** ESP8266 WiFi for local SAR command centers
- **Long-Range:** LoRa for field operations up to 15km in open terrain
- **Data Format:** JSON telemetry packets (sensor readings + GPS + status flags)

### Control Architecture
- Microcontroller-based state machine for autonomous operation
- Manual override capability for operator intervention
- Real-time debugging via serial communication

---

## 🧪 Testing & Validation

### Environmental Scenarios
- Controlled laboratory tests with calibrated heat sources
- Simulated debris fields with varied obstacle geometries
- Outdoor testing with ambient thermal interference
- Endurance runs for battery and component reliability

### Performance Results
| Metric | Controlled Environment | Complex Debris | Outdoor Conditions |
|--------|----------------------|---------------|--------------------|
| Detection Accuracy | 95% | 85% | 70% |
| Obstacle Avoidance Success | 90% | 80% | 75% |
| Coverage Time (20m²) | 5 min | 8-12 min | 10-15 min |
| Battery Runtime | 1 hour | 55 min | 45-50 min |

---

## 📈 Challenges & Solutions

| Challenge | Impact | Solution |
|-----------|--------|----------|
| **Thermal Interference** | Sunlit surfaces cause false positives | Multi-parameter fusion; background subtraction algorithm |
| **Complex Obstacle Navigation** | Irregular debris shapes confuse sensors | Implementation of A* pathfinding; future LiDAR integration |
| **Limited Battery Life** | Restricts mission duration | Energy harvesting (solar); larger capacity batteries planned |
| **Partial Heat Obstruction** | False negatives under heavy rubble | Additional GPR sensor; multi-angle scanning |

---

## 🔮 Future Enhancements

### Near-Term (Phase 2)
- Ground-Penetrating Radar (GPR) integration for subsurface detection
- LiDAR for improved 3D obstacle mapping
- Machine learning for autonomous decision-making
- Solar charging for extended outdoor operations

### Long-Term (Phase 3)
- Multi-robot coordination for large-scale SAR operations
- AI-based victim prioritization (differentiating body sizes/temperatures)
- Autonomous docking stations for field deployment
- Integration with emergency response management systems

---

## 📋 Project Outcomes

### Attainments
- **Engineering Knowledge:** Linear circuit design, microcontroller programming, sensor integration
- **Problem Analysis & Solution Design:** Comprehensive requirements analysis leading to multi-sensor architecture
- **Modern Tools Expertise:** ESP32, Arduino, MATLAB/Simulink, Vivado (hardware prototyping)
- **Team Collaboration:** Effective cross-functional development and testing
- **Societal Impact:** Enhanced safety for rescue operations; reduced manual search risk

### Applications
- Natural disaster response (earthquakes, building collapses)
- Mining and industrial accident rescue
- Confined space exploration
- Emergency management support systems

---

## 🚀 Quick Start

### Prerequisites
- ESP32 DevKit
- Arduino IDE or PlatformIO
- Required libraries: DHT, HC-SR04, FLIR Lepton driver, WiFi

### Setup
```bash
# Clone repository
git clone https://github.com/shankar1955/CSSR-Rover-Prototype.git
cd CSSR-Rover

# Install dependencies
pip install -r requirements.txt

# Flash firmware
cd firmware
platformio run --target upload
```

### Calibration
1. Power on the rover in open space
2. Run `calibration_guide.md` procedures for thermal/ultrasonic sensors
3. Adjust detection thresholds in `thermal_detection.cpp`
4. Test with simulated heat sources (heated water bottles)

---

## 📝 License

This project is developed as part of an academic engineering curriculum and is available for educational and non-commercial use. Unauthorized commercial deployment is restricted.

---

## 👥 Contributors

**Shankar M** (Reg: 23EE047)  
**Hari Venkatesh R** (Reg: 23EE013)

Department of Electrical and Electronics Engineering  
Chennai Institute of Technology, Chennai-69



---

*Last Updated: November 28, 2025*
