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
  joyConnection: boolean
  cameraA: string
  attachState: boolean
  detection: boolean
  startButton: boolean
  readRobotFlipperAngle: number
  readRobotSpeedLeft: number
  readRobotSpeedRight: number
  readRobotPitchAngle : number
  previousUpdate : number;
  servo1Angle: number; 
  servo2Angle: number;
  motor: number;
}

class App extends Component<IProps, IState> {

  constructor(props: IProps) {
    super(props)

    this.state = {
      ros: initROSMasterURI(configs.ROSMasterURL.url),
      robotConnection: false,
      cameraA: "",
      attachState: false,
      joyConnection: false,
      startButton: false,
      detection: false,
      readRobotFlipperAngle: 0,
      readRobotSpeedRight: 0,
      readRobotSpeedLeft: 0,
      readRobotPitchAngle : 0,
      previousUpdate : 0,
      servo1Angle: 90,
      servo2Angle: 90,
      motor: 0,
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
      this.setState({ servo1Angle: servoMessage.data });
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
      this.setState({ servo2Angle: servoMessage.data });
    });
  }

  subscribemotor(ros: ROSLIB.Ros, motor: string) {
    const servoTopic = new ROSLIB.Topic({
      ros: ros,
      name: motor, // '/servo1_angle'
      messageType: 'geometry_msgs/Twist', // ตรงกับ joy.py
    });
  
    servoTopic.subscribe((message: ROSLIB.Message) => {
      const servoMessage = message as ROSLIB.Message & { data: number };
      console.log("Received motor:", servoMessage.data); // Debug
      this.setState({ motor: servoMessage.data });
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
        data: [0,0,0,0],
      };

      let flipperAngle = Math.floor(data.data[2] * (360/140)  % 360) > 180 ?  Math.floor(data.data[2] * (360/140)  % 360) - 360 : Math.floor(data.data[2] * (360/140)  % 360)

      this.setState({readRobotSpeedLeft : Math.floor(data.data[0] / 35.255) , readRobotSpeedRight : Math.floor(data.data[1] / 35.255)} )
      this.setState({readRobotPitchAngle : data.data[3]})

      if(flipperAngle - this.state.previousUpdate > 10){
        console.log("updated")
        this.setState({readRobotFlipperAngle : flipperAngle})
      }
      this.setState({previousUpdate : flipperAngle})
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
      this.subscribeRobot(ros , '/motor_array')
      this.subscribeServo1Angle(ros, '/servo1_angle'); // เพิ่มการสมัครสมาชิก
      this.subscribeServo2Angle(ros, '/servo2_angle'); 
      this.subscribemotor(ros, '/cmd_vel');// เพิ่มการสมัครสมาชิก
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
    const { servo1Angle } = this.state;
    const { servo2Angle } = this.state;
    const { motor } = this.state;
    return (
        
        <div className="all">
          <GamepadComponent ros={this.state.ros} joypadTopicName={'/gui/output/robot_control'} onJoyStickConnection={(connection) => {
          this.setState({ joyConnection: connection });
        }} joyEnable={this.state.startButton} />
          <nav>
            <div className="Name">RMRC</div>
            <div className="logo">
              <img src={logoImage}></img>
            </div>
          </nav>
          <div className="container">
            <div className="Cam1 glass">
            <img src="http://localhost:8000/video_feed" alt="Camera Stream" />
            {/* <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/usb_cam/image_raw/compressed'} height={'850px'} width={'400px'} rotate={180} hidden={false}></ImageViewer> */}
              <div className="textcam">CAM 1</div>
            </div>
            <div className="Cam2 glass">
            <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/usb_cam1/image_raw/compressed'} height={'850px'} width={'400px'} rotate={180} hidden={false}></ImageViewer>
              <div className="textcam">CAM 2</div>
            </div>
            <div className="CamAI glass">
            <ImageViewer ros={this.state.ros} ImageCompressedTopic={'/detect_marker/image_raw/compressed'} height={'850px'} width={'400px'} rotate={180} hidden={!this.state.detection} ></ImageViewer>
              <div className="textcam">CAM AI</div>
            </div>
            <div className="StatusBar">
              <div className="speedstatus glass">
                <div className="leftrpm">Speed L : {motor} rpm</div>
                <div className="rightrpm">Speed R : {motor} rpm</div>
              </div>
  
              <div className="flipperstatus glass">
                <div className="leftflipper">Flipper L : {servo1Angle}°</div>
                <div className="rightflipper">Flipper R : {servo2Angle}°</div>
              </div>
              <div className="Joystatus glass">
                <img src={joy1}></img>
                <div className="statustext">
                <div>
                   Joy Status: {this.state.joyConnection ? "Connected" : "Disconnected"}
                </div>
                </div>
              </div>
              <div className="joybutton">
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
              </div>
            </div>
          </div>
          <div className="detectbutton">
          <Button variant="primary" onClick={() => {
              this.setState({ detection: !this.state.detection })
            }}>
              {this.state.detection ? "Detection Off" : "Detection On"}
            </Button>
          </div>
          
        </div>
      
    );
  }
}

export default App;