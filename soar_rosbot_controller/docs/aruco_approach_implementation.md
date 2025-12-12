# ArUco Approach Implementation

## Overview

This document summarizes the implementation of the ArUco approach behavior using Soar cognitive architecture integrated with ROS2. The robot moves forward when an ArUco tag is detected and stops when it loses sight of the tag.

## Problem Statement

Create Soar rules that make a robot:
- **Move forward** when an ArUco tag is detected
- **Stop** when the ArUco tag is not detected

## Key Fixes and Solutions

### 1. Soar State Initialization (o-support vs i-support)

**Problem:** The original `elaborate*state*name` rule created `^name` with i-support, which gets retracted when conditions change.

**Solution:** Use an `init-agent` operator that creates the state name with o-support (persistent):

```soar
sp {propose*init-agent
   (state <s> ^superstate nil
             -^name)
-->
   (<s> ^operator <o> +)
   (<o> ^name init-agent)
}

sp {apply*init-agent
   (state <s> ^operator.name init-agent)
-->
   (<s> ^name aruco-approach)
}
```

### 2. Wait Operator for Impasse Handling

**Problem:** Soar hits goal stack depth errors when there's no operator to select (operator no-change impasse).

**Solution:** Add a `wait` operator to handle state no-change impasses:

```soar
sp {propose*wait
   (state <s> ^attribute state
              ^choices none
             -^operator.name wait)
-->
   (<s> ^operator <o>)
   (<o> ^name wait)
}

sp {apply*wait
   (state <s> ^operator <o>)
   (<o> ^name wait)
-->
   (<o> ^random elaboration)
}

sp {prefer*others*over*wait
   (state <s> ^operator <o1> +
              ^operator <o2> +)
   (<o1> ^name <> wait)
   (<o2> ^name wait)
-->
   (<s> ^operator <o1> > <o2>)
}
```

### 3. Proper Apply Rule Syntax for O-Support

**Problem:** Using `^operator.name move-forward` creates i-support matches, but output-link modifications require o-support.

**Solution:** Use `^operator <o>` followed by `(<o> ^name move-forward)` for all rules that modify the output-link:

```soar
# Correct - o-support
sp {apply*move-forward*remove-stop
   (state <s> ^operator <o>
              ^io.output-link <ol>)
   (<o> ^name move-forward)
   (<ol> ^command <cmd>)
   (<cmd> ^name stop)
-->
   (<ol> ^command <cmd> -)
}

# Incorrect - i-support (don't use for apply rules)
sp {apply*move-forward*remove-stop
   (state <s> ^operator.name move-forward
              ^io.output-link <ol>)
   ...
}
```

### 4. Subscriber Queue Draining

**Problem:** The original `soar_ros::Subscriber` queued every ROS message. When detection changed from `false` to `true`, thousands of old `false` messages were still in the queue, causing significant lag.

**Solution:** Override `process_r2s()` to drain the queue and only use the latest value:

**File:** `perception_subscribers.hpp`

```cpp
class ArUcoDetectedSubscriber : public soar_ros::Subscriber<std_msgs::msg::Bool>
{
public:
  // ... constructor ...

  // Override process_r2s to drain queue and use only the latest value
  void process_r2s() override
  {
    // Drain the queue, keeping only the latest message
    std::optional<std_msgs::msg::Bool> latest;
    auto res = this->m_r2sQueue.tryPop();
    while (res.has_value()) {
      latest = res;
      res = this->m_r2sQueue.tryPop();
    }

    if (!latest.has_value()) {
      return;  // No messages
    }

    bool detected = latest.value().data;

    sml::Identifier * il = this->m_pAgent->GetInputLink();

    // Remove old WMEs if they exist
    if (aruco_wme_ != nullptr) {
      aruco_wme_->DestroyWME();
      aruco_wme_ = nullptr;
      detected_wme_ = nullptr;
    }

    // Create new WMEs
    aruco_wme_ = il->CreateIdWME("aruco");
    detected_wme_ = aruco_wme_->CreateIntWME("detected", detected ? 1 : 0);
  }

  void parse(std_msgs::msg::Bool msg) override
  {
    // Not used - we override process_r2s directly
    (void)msg;
  }

private:
  sml::Identifier* aruco_wme_;
  sml::WMElement* detected_wme_;
};
```

### 5. Continuous Velocity Publishing

**Problem:** The motion controller only published velocity when the command changed. Some robots/simulators need continuous velocity commands to maintain motion.

**Solution:** Publish every received command, but only log when it changes:

**File:** `motion_controller.cpp`

```cpp
void MotionController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & command = msg->data;

  RCLCPP_DEBUG(this->get_logger(), "Received command: %s", command.c_str());

  // Convert command to velocity
  geometry_msgs::msg::Twist velocity = commandToVelocity(command);

  // Create TwistStamped message with timestamp
  geometry_msgs::msg::TwistStamped velocity_stamped;
  velocity_stamped.header.stamp = this->now();
  velocity_stamped.header.frame_id = "base_link";
  velocity_stamped.twist = velocity;

  // Always publish velocity
  velocity_pub_->publish(velocity_stamped);

  // Log only when command changes
  if (command != last_command_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Command changed to: %s - Publishing velocity: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
      command.c_str(), velocity.linear.x, velocity.linear.y, velocity.angular.z);
    last_command_ = command;
  }
}
```

### 6. CMakeLists.txt Install Target

**Problem:** The `soar_maze_controller` executable was being built but not installed.

**Solution:** Add it to the install targets:

```cmake
install(TARGETS
  soar_maze_controller
  motion_controller
  DESTINATION lib/${PROJECT_NAME}
)
```

## Final Soar Rules Structure

The complete `aruco_approach.soar` file contains:

| Rule | Purpose |
|------|---------|
| `propose*init-agent` | Proposes initialization when no name exists |
| `apply*init-agent` | Creates persistent state name `aruco-approach` |
| `propose*wait` | Handles state no-change impasses |
| `apply*wait` | Prevents operator no-change impasse |
| `prefer*others*over*wait` | Makes wait lowest priority |
| `propose*move-forward` | Proposes when `^aruco.detected 1` |
| `propose*stop` | Proposes when `^aruco.detected 0` |
| `apply*move-forward` | Creates `move-forward` command on output-link |
| `apply*stop` | Creates `stop` command on output-link |
| `apply*move-forward*remove-stop` | Removes `stop` command when `move-forward` selected |
| `apply*stop*remove-move-forward` | Removes `move-forward` command when `stop` selected |
| `apply*remove-completed-command` | Cleans up commands marked as complete |

## Files Modified

| File | Changes |
|------|---------|
| `soar_rules/aruco_approach.soar` | Complete rewrite with proper Soar patterns |
| `soar_rules/move_forward.soar` | Added init-agent and wait operators |
| `include/soar_rosbot_controller/perception_subscribers.hpp` | Queue draining for latest value |
| `src/motion_controller.cpp` | Continuous velocity publishing |
| `CMakeLists.txt` | Fixed install targets |

## Data Flow

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  ArUco Detector │────▶│  /aruco/detected │────▶│ ArUcoDetected   │
│     (ROS2)      │     │    (Bool topic)  │     │   Subscriber    │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
                                                          │
                                                          ▼
                                                ┌─────────────────┐
                                                │  Soar Input     │
                                                │  ^aruco.detected│
                                                └────────┬────────┘
                                                          │
                                                          ▼
                                                ┌─────────────────┐
                                                │  Soar Rules     │
                                                │  (aruco_approach│
                                                │   .soar)        │
                                                └────────┬────────┘
                                                          │
                                                          ▼
                                                ┌─────────────────┐
                                                │  Soar Output    │
                                                │  ^command.name  │
                                                └────────┬────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│     Robot       │◀────│    /cmd_vel      │◀────│    Motion       │
│   (Gazebo/HW)   │     │ (TwistStamped)   │     │   Controller    │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

## Lessons Learned

1. **Soar o-support vs i-support**: Apply rules that modify working memory (especially output-link) need o-support. Use `^operator <o>` + `(<o> ^name X)` pattern, not `^operator.name X`.

2. **Message queue management**: High-frequency ROS topics can overwhelm Soar's processing. Drain queues and use only the latest value for reactive behaviors.

3. **Continuous commands**: Some robot platforms require continuous velocity commands. Don't skip publishing just because the command hasn't changed.

4. **Impasse handling**: Always provide a way to handle impasses (wait operator) to prevent goal stack depth errors.

5. **State persistence**: Use operators to create persistent state attributes rather than elaboration rules.
