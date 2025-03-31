import React, { Component } from 'react';

import ROSLIB from 'roslib';

import logo from './image/logo.png';
import wifi_white from './image/wifi_white.png';
import settings from './image/setting.png';
import joy from './image/joy.png';
import connect from './image/connect.png';
import connectionLost from './image/connection_lost.jpg';
import logoImage from "./image/logo.png";
import joy1 from "./image/joy.png";
import keyboard from "./image/rk68-Photoroom.png"

import { Container, Row, Col, Button, Image } from 'react-bootstrap';
import configs from '../../configs';
import { initROSMasterURI } from '../../components/ROS/Connector/ROSConnector';
import ImageViewer from '../../components/ROS/ImageViewer';
import GamepadComponent from '../../components/GamepadAPI';


import './index_new.css';
import FlipperVisualization from '../../components/FlipperVisualization';

// /usb_cam/image_raw/compressed

interface IProps { }

interface IState {
  ros: ROSLIB.Ros;
  robotConnection: boolean
  onKeyboardConnection: boolean
  cameraA: string
  attachState: boolean
  detection: boolean
  startButton: boolean
  togglePopup: boolean
  readRobotFlipperAngle: number
  readRobotSpeedLeft: number
  readRobotSpeedRight: number
  readRobotPitchAngle: number
  previousUpdate: number;
  servo_left_angle: number;
  servo_right_angle: number;
  motorL: number;
  motorR: number;
  arm1_position: number;
  arm2_position: number;
  arm3_position: number;
  arm4_position: number;
  arm5_position: number;
  wifiSignal: number;
}

class App extends Component<IProps, IState> {

  constructor(props: IProps) {
    super(props)

    this.state = {
      ros: initROSMasterURI(configs.ROSMasterURL.url),
      robotConnection: false,
      cameraA: "",
      attachState: false,
      onKeyboardConnection: false,
      startButton: false,
      detection: false,
      togglePopup: false,
      readRobotFlipperAngle: 0,
      readRobotSpeedRight: 0,
      readRobotSpeedLeft: 0,
      readRobotPitchAngle: 0,
      previousUpdate: 0,
      servo_right_angle: 135,
      servo_left_angle: 135,
      motorL: 0,
      motorR: 0,
      arm1_position: 90,
      arm2_position: 90,
      arm3_position: 90,
      arm4_position: 90,
      arm5_position: 90,
      wifiSignal: 0,
    }
  }


  // subscribeCameraA(ros: ROSLIB.Ros, topicName: string) {
  //   const { cameraA } = this.state;

  //   const imageTopic = new ROSLIB.Topic({
  //     ros: ros,
  //     name: topicName, // adjust the topic name based on your setup
  //     messageType: 'sensor_msgs/CompressedImage',
  //   });

  //   imageTopic.subscribe((message: ROSLIB.Message) => {
  //     const compressedImageMessage = message as ROSLIB.Message & { format: string; data: string };
  //     const format = compressedImageMessage.format;
  //     const imageData = compressedImageMessage.data;

  //     const imageUrl = `data:image/${format};base64,${imageData}`;


  //     this.setState({ cameraA: imageUrl });
  //   });
  // }
  subscribeServo1Angle(ros: ROSLIB.Ros, servo1_angle: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: servo1_angle, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const servoMessage = message as ROSLIB.Message & { data: number };
      console.log("Received servo1_angle:", servoMessage.data); // Debug
      this.setState({ servo_left_angle: servoMessage.data });
    });
  }

  subscribeServo2Angle(ros: ROSLIB.Ros, servo2_angle: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: servo2_angle, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const servoMessage = message as ROSLIB.Message & { data: number };
      console.log("Received servo2_angle:", servoMessage.data); // Debug
      this.setState({ servo_right_angle: servoMessage.data });
    });
  }

  subscribemotorL(ros: ROSLIB.Ros, motorL: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: motorL, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const servoMessage = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", servoMessage.data); // Debug
      this.setState({ motorL: servoMessage.data });
    });
  }

  subscribemotorR(ros: ROSLIB.Ros, motorR: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: motorR, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const servoMessage = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", servoMessage.data); // Debug
      this.setState({ motorR: servoMessage.data });
    });
  }

  subscribeArm1(ros: ROSLIB.Ros, Arm1: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: Arm1, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const arm1_sub = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", arm1_sub.data); // Debug
      this.setState({ arm1_position: arm1_sub.data });
    });
  }

  subscribeArm2(ros: ROSLIB.Ros, Arm2: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: Arm2, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const arm2_sub = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", arm2_sub.data); // Debug
      this.setState({ arm2_position: arm2_sub.data });
    });
  }

  subscribeArm3(ros: ROSLIB.Ros, Arm3: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: Arm3, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const arm3_sub = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", arm3_sub.data); // Debug
      this.setState({ arm3_position: arm3_sub.data });
    });
  }

  subscribeArm4(ros: ROSLIB.Ros, Arm4: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: Arm4, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const arm4_sub = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", arm4_sub.data); // Debug
      this.setState({ arm4_position: arm4_sub.data });
    });
  }

  subscribeArm5(ros: ROSLIB.Ros, Arm5: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: Arm5, // '/servo1_angle'
      messageType: 'std_msgs/Int16', // ตรงกับ joy.py
    });

    servoTopic.subscribe((message: ROSLIB.Message) => {
      const arm5_sub = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", arm5_sub.data); // Debug
      this.setState({ arm5_position: arm5_sub.data });
    });
  }

  subscribeWifiSignal(ros: ROSLIB.Ros, Hotspot_burritos: string) {
    const wifiTopic = new ROSLIB.Topic({
      ros: ros,
      name: Hotspot_burritos, // '/wifi_signal'
      messageType: 'std_msgs/Float32', // ประเภทข้อความที่ใช้ใน ROS
    });

    wifiTopic.subscribe((message: ROSLIB.Message) => {
      const wifiMessage = message as ROSLIB.Message & { data: number };
      console.log("Received WiFi Signal:", wifiMessage.data); // Debug
      this.setState({ wifiSignal: wifiMessage.data });
    });
  }


  subscribeRobot(ros: ROSLIB.Ros, topicName: string) {
    const { cameraA } = this.state;

    const robotReadTopic = new ROSLIB.Topic({
      ros: ros,
      name: topicName, // adjust the topic name based on your setup
      messageType: 'std_msgs/Float32MultiArray',
    });

    robotReadTopic.subscribe((message: ROSLIB.Message) => {
      const data = message as ROSLIB.Message &
      {
        layout: {
          dim: [],
          data_offset: 0,
        },
        data: [0, 0, 0, 0],
      };

      let flipperAngle = Math.floor(data.data[2] * (360 / 140) % 360) > 180 ? Math.floor(data.data[2] * (360 / 140) % 360) - 360 : Math.floor(data.data[2] * (360 / 140) % 360)

      this.setState({ readRobotSpeedLeft: Math.floor(data.data[0] / 35.255), readRobotSpeedRight: Math.floor(data.data[1] / 35.255) })
      this.setState({ readRobotPitchAngle: data.data[3] })

      if (flipperAngle - this.state.previousUpdate > 10) {
        console.log("updated")
        this.setState({ readRobotFlipperAngle: flipperAngle })
      }
      this.setState({ previousUpdate: flipperAngle })
      // const format = compressedImageMessage.format;
      // const imageData = compressedImageMessage.data;

      // const imageUrl = `data:image/${format};base64,${imageData}`;


      // this.setState({ cameraA: imageUrl });
    });
  }

  componentDidMount = () => {
    const { ros } = this.state
    console.log("did")
    ros.on('connection', () => {
      console.log("ROS : Connected to ROS Master : ", configs.ROSMasterURL.url);
      this.subscribeRobot(ros, '/motor_array')
      this.subscribeServo1Angle(ros, '/servo1_angle'); // เพิ่มการสมัครสมาชิก
      this.subscribeServo2Angle(ros, '/servo2_angle');
      this.subscribemotorL(ros, '/motorL'); // เพิ่มการสมัครสมาชิก
      this.subscribemotorR(ros, '/motorR');
      this.subscribeArm1(ros, '/arm1'); // เพิ่มการสมัครสมาชิก
      this.subscribeArm2(ros, '/arm2');
      this.subscribeArm3(ros, '/arm3'); // เพิ่มการสมัครสมาชิก
      this.subscribeArm4(ros, '/arm4');
      this.subscribeArm5(ros, '/arm5'); // เพิ่มการสมัครสมาชิก
      this.subscribeWifiSignal(ros, '/wifi_signal');
      this.setState({ robotConnection: true })
    })

    ros.on('close', () => {
      this.setState({ robotConnection: false })
    })
    ros.on('error', () => {
      console.log("ROS : Can't connect to ros master : ", configs.ROSMasterURL.url);
      this.setState({ robotConnection: false })
    })
  }

  componentWillUnmount = () => {
    const { ros } = this.state
    // ros.close();
    // this.setState({attachState : true})
    // console.log("umount")
  }

  alignmentStyle: React.CSSProperties = {
    // transform: `rotate(${flipImageDeg ? flipImageDeg : 180}deg)`,
    paddingLeft: 0,
    paddingRight: 0,
    position: 'relative',
    display: 'flex',
    // transition: 'transform 0.5s ease', // Add a smooth transition for a better visual effect
  };

  render() {
    const { servo_left_angle, servo_right_angle, motorL, motorR, arm1_position, arm2_position, arm3_position, arm4_position, arm5_position , wifiSignal} = this.state;

    return (

      <div className="all">
        <GamepadComponent ros={this.state.ros} joypadTopicName={'/gui/output/robot_control'} onKeyboardConnection={(connection) => {
          this.setState({ onKeyboardConnection: connection });
        }} joyEnable={this.state.startButton} />
        <nav>
          <div className="Name">RMRC</div>  
          <div className="logo">
            <img src={logoImage}></img>
          </div>
        </nav>
        <div className="container">
          <div className="Cam1 glass">
            {/* <img src="http://localhost:8000/video_feed" alt="Camera Stream" /> */}
            <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/usb_cam/image_raw/compressed'} height={'100%'} width={'100%'} rotate={180} hidden={false}></ImageViewer>
            <div className="textcam">CAM 1</div>
            <button className="zoom"></button>
          </div>
          <div className="Cam2 glass">
            <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/usb_cam1/image_raw/compressed'} height={'100%'} width={'100%'} rotate={180} hidden={false}></ImageViewer>
            <div className="textcam">CAM 2</div>
          </div>
          <div className="CamAI glass">
            <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/detect_marker/image_raw/compressed'} height={'100%'} width={'100%'} rotate={180} hidden={!this.state.detection} ></ImageViewer>
            <div className="detectbutton">
              <Button variant="primary" onClick={() => {
                this.setState({ detection: !this.state.detection })
              }}>
                {this.state.detection ? "Detection Off" : "Detection On"}
              </Button>
            </div>
            <div className="textcam">CAM AI</div>
          </div>
          <div className="StatusBar">
            <div className="speedstatus glass">
              <div className="leftrpm">Speed L : {motorL} </div>
              <div className="rightrpm">Speed R : {motorR} </div>
            </div>

            <div className="flipperstatus glass">
              <div className="leftflipper">Flipper L : {servo_left_angle}°</div>
              <div className="rightflipper">Flipper R : {servo_right_angle}°</div>
            </div>
            <div className="Joystatus glass">
              <img src={keyboard}></img>
              <div className="statustext">
                <div>
                  Keyboard Status: {this.state.onKeyboardConnection ? "Connected" : "Disconnected"}
                </div>
              </div>
            </div>
            <div className="wifisignal glass">
                 WiFi Signal: {wifiSignal} dBm {wifiSignal > -50 ? "(Good)" : wifiSignal > -70 ? "(Fair)" : "(Weak)"}
            </div>
            {/* <div className="joybutton">
              {
                !this.state.startButton ? <Button variant="primary" onClick={() => {
                  this.setState({ startButton: !this.state.startButton })
                }}>
                  {!this.state.startButton ? "Start" : "Stop"}
                </Button> : <Button variant="danger" onClick={() => {
                  this.setState({ startButton: !this.state.startButton })
                }}>
                  {!this.state.startButton ? "Start" : "Stop"}
                </Button>
              }
            </div> */}
            <div className="Arm glass">
              <div className="servoarm">Servo1 : {arm1_position}°</div>
             
              <div className="servoarm">Servo2 : {arm2_position}°</div>
              <div className="servoarm">Servo3 : {arm3_position}°</div>
              <div className="servoarm">Servo4 : {arm4_position}°</div>
              <div className="servoarm">Servo5 : {arm5_position}°</div>
              
            </div>
            <div className="IMAGE">
            </div>
          </div>
        </div>
      </div>
    //   <div className="wifiSignal">
    //   WiFi Signal: {wifiSignal} dBm {wifiSignal > -50 ? "(Good)" : wifiSignal > -70 ? "(Fair)" : "(Weak)"}
    // </div>

    );
  }
}

export default App;