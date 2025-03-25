import React, { Component } from "react";
import ROSLIB from "roslib";

interface KeyboardComponentProps {
  ros: ROSLIB.Ros;
  joypadTopicName: string;
  joyEnable: boolean;
  onAxesChange?: (axes: number[]) => void;
  onButtonsChange?: (buttons: number[]) => void; // เปลี่ยนเป็น number[] แทน GamepadButton
  onKeyboardConnection?: (connection: boolean) => void;
}

interface KeyboardComponentState {
  pressedKeys: Set<string>;
  joypadRosTopic: ROSLIB.Topic<ROSLIB.Message>;
  sensorJoyTopic: ROSLIB.Topic<ROSLIB.Message>;
  robotSpeedLeft: number;
  robotSpeedRight: number;
  robotSpeedFlipper: number;
  boostMode: boolean;
  onetimeTicker: boolean;
}

class KeyboardComponent extends Component<KeyboardComponentProps, KeyboardComponentState> {
  constructor(props: KeyboardComponentProps) {
    super(props);
    this.state = {
      pressedKeys: new Set(),
      joypadRosTopic: new ROSLIB.Topic({
        ros: this.props.ros,
        name: this.props.joypadTopicName,
        messageType: "std_msgs/Float32MultiArray",
      }),
      sensorJoyTopic: new ROSLIB.Topic({
        ros: this.props.ros,
        name: "/gui/output/joy",
        messageType: "sensor_msgs/Joy",
      }),
      robotSpeedLeft: 0,
      robotSpeedRight: 0,
      robotSpeedFlipper: 0,
      boostMode: false,
      onetimeTicker: false,
    };
  }

  componentDidMount() {
    console.log("ROS connection:", this.props.ros);
    // เพิ่ม event listener สำหรับคีย์บอร์ด
    window.addEventListener("keydown", this.handleKeyDown);
    window.addEventListener("keyup", this.handleKeyUp);

    // เริ่ม loop เพื่ออัปเดตสถานะ
    this.gameLoop();
  }

  componentWillUnmount() {
    // ลบ event listener เมื่อ component ถูกทำลาย
    window.removeEventListener("keydown", this.handleKeyDown);
    window.removeEventListener("keyup", this.handleKeyUp);
  }

  handleKeyDown = (event: KeyboardEvent) => {
    const { pressedKeys } = this.state;
    pressedKeys.add(event.key.toLowerCase());
    this.setState({ pressedKeys });

    if (this.props.onKeyboardConnection && pressedKeys.size === 1) {
      this.props.onKeyboardConnection(true); // แจ้งว่าเริ่มใช้งานคีย์บอร์ด
    }
  };

  handleKeyUp = (event: KeyboardEvent) => {
    const { pressedKeys } = this.state;
    pressedKeys.delete(event.key.toLowerCase());
    this.setState({ pressedKeys });

    if (this.props.onKeyboardConnection && pressedKeys.size === 0) {
      this.props.onKeyboardConnection(false); // แจ้งว่าหยุดใช้งานคีย์บอร์ด
    }
  };

  publishFloat32MultiArray = (data: number[]) => {
    const float32MultiArrayMessage = new ROSLIB.Message({
      layout: {
        dim: [],
        data_offset: 0,
      },
      data: data,
    });
    this.state.joypadRosTopic.publish(float32MultiArrayMessage);
  };

  publishJoyMessage = (axes: number[], buttons: number[]) => {
    const joyMessage = new ROSLIB.Message({
      header: {
        stamp: { sec: 0, nsec: 0 },
        frame_id: "",
      },
      axes: axes,
      buttons: buttons,
    });
    this.state.sensorJoyTopic.publish(joyMessage);
  };

  updateKeyboardState = () => {
    const { pressedKeys } = this.state;

    // กำหนดการควบคุมด้วยคีย์บอร์ด (คล้ายโค้ด Python)
    let robotSpeedLeft = this.state.robotSpeedLeft;
    let robotSpeedRight = this.state.robotSpeedRight;
    let robotSpeedFlipper = this.state.robotSpeedFlipper;

    if (pressedKeys.has("w")) {
      robotSpeedLeft += 1;
      robotSpeedRight += 1;
    }
    if (pressedKeys.has("s")) {
      robotSpeedLeft -= 1;
      robotSpeedRight -= 1;
    }
    if (pressedKeys.has("a")) {
      robotSpeedLeft -= 1;
      robotSpeedRight += 1;
    }
    if (pressedKeys.has("d")) {
      robotSpeedLeft += 1;
      robotSpeedRight -= 1;
    }

    // จำลอง axes และ buttons สำหรับ sensor_msgs/Joy
    const axes = [
      pressedKeys.has("a") ? -1 : pressedKeys.has("d") ? 1 : 0, // แกนซ้าย-ขวา
      pressedKeys.has("w") ? 1 : pressedKeys.has("s") ? -1 : 0, // แกนหน้า-หลัง
      0, // placeholder
    ];
    const buttons = [
      pressedKeys.has("q") ? 1 : 0,
      pressedKeys.has("e") ? 1 : 0,
      pressedKeys.has("r") ? 1 : 0,
      pressedKeys.has("t") ? 1 : 0,
    ];

    this.setState({
      robotSpeedLeft,
      robotSpeedRight,
      robotSpeedFlipper,
    });

    if (this.props.joyEnable) {
      const publishFloat = [robotSpeedRight, robotSpeedLeft, robotSpeedFlipper];
      this.publishJoyMessage(axes, buttons);
      this.publishFloat32MultiArray(publishFloat);
    } else {
      this.publishFloat32MultiArray([0, 0, 0]);
    }
  };

  gameLoop = () => {
    this.updateKeyboardState();
    requestAnimationFrame(this.gameLoop);
  };

  render() {
    const { pressedKeys } = this.state;

    return (
      <div>
        {/* //<p>Pressed Keys: {Array.from(pressedKeys).join(", ")}</p> */}
      </div>
    );
  }
}

export default KeyboardComponent;