# Task Turning Algorithm Description

## Overview

The task turning algorithm controls an autonomous boat to navigate through a pool while maintaining a safe distance from the right shore and avoiding front obstacles. The boat uses ultrasonic sensors to detect distances and adjusts motor speeds accordingly.

## Main Control Loop

The algorithm runs in an infinite loop, continuously reading sensor data and making decisions:

1. **Sensor Reading**: Read front and right sonar distances
2. **State Check**: Determine if currently in turning mode
3. **Decision Making**: Choose between obstacle avoidance or straight movement correction
4. **Motor Control**: Set appropriate motor speeds based on decision

## Turning State Management

When an obstacle is detected in front, the boat enters turning mode:

- **Entry Condition**: Front distance < safe threshold
- **Exit Conditions**:
  - Timeout (maximum turn time exceeded)
  - Front obstacle cleared (distance >= safe threshold)

During turning, the boat performs a left turn with adaptive speed boost based on proximity to obstacle.

## Straight Movement Correction

When no front obstacle, the boat attempts to maintain straight course:

- **Error Calculation**: Compare right distance to target safe distance
- **Tolerance Check**: If within acceptable range, maintain straight with slight bias
- **Correction Logic**:
  - If too far from shore: turn right (increase left motor speed)
  - If too close to shore: turn left (increase right motor speed)
- **Cooldown Mechanism**: Prevent rapid successive corrections
- **Stability Hold**: Brief pause after correction to let boat settle

## Heading Alignment

For significant deviations, perform heading alignment:

- **Method**: Use derivative of right distance over time
- **Logic**: If distance increasing, boat is yawing outward → turn right to align
- **Iteration**: Repeat until derivative within tolerance or max iterations reached

## Adaptive Speed Control

- **Base Speed**: Constant forward propulsion
- **Differential Gearing**: Asymmetric motor speeds for turning/correction
- **Boost Calculation**: Proportional to obstacle proximity
- **Constraints**: All speeds clamped within min/max limits

## Safety Features

- **Obstacle Detection**: Front sensor prevents collisions
- **Timeout Protection**: Prevents infinite turning loops
- **Cooldown Periods**: Reduces oscillation in corrections
- **Median Filtering**: Sonar readings use median of multiple samples for noise reduction

## Key Parameters

- Distance thresholds for safe/obstacle zones
- Motor speed ranges and gear ratios
- Timing delays for stability and alignment
- Tolerance bands for straight movement
- Maximum turn duration

This algorithm enables reliable autonomous navigation in constrained environments with reactive obstacle avoidance and proactive course correction.