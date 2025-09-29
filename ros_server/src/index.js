const rclnodejs = require('rclnodejs');
const express = require('express');
const app = express();
const cors = require('cors');

app.use(cors());
app.use(express.json())
const port = 3000;

var motorSpeedState = { mot_1_rad_sec: 0, mot_2_rad_sec: 0 };
var AccelerationVelsState = {x_velocity: 0, y_velocity: 0, z_velocity: 0};
var AngularVelsState = {x_angular_velocity: 0, y_angular_velocity: 0, z_angular_velocity: 0};

var ultra_distance = {right: 0.0, center: 0.0, left: 0.0};

var humidityState = {humidity: 0.0};
var temperatureState = {temperature_celcius: 0.0, temperature_fahrenheit: 0.0};

async function sub_motor_speed() {
  const node = rclnodejs.createNode('motor_speed_sub');

  node.createSubscription(
    'system_msgs/msg/MotorVels',
    'motor_vels',
    (state) => {
      motorSpeedState = state;
    }
  );

  node.spin();
}

async function sub_dht() {
  const node = rclnodejs.createNode('dht_sub');

  node.createSubscription(
    'system_msgs/msg/TemperatureVals',
    '/DHT11/temperature',
    (state) => {
      temperatureState = state;
    }
  );

  node.createSubscription(
    'system_msgs/msg/HumidityVal',
    '/DHT11/humidity',
    (state) => {
      humidityState = state;
    }
  );

  node.spin();
}

async function sub_acce() {
  const node = rclnodejs.createNode('acce_sub');

  node.createSubscription(
    'system_msgs/msg/AccelerationVels',
    '/accelerometer/relative_acceleration',
    (state) => {
      AccelerationVelsState = state;
    }
  );

  node.createSubscription(
    'system_msgs/msg/AngularVels',
    '/accelerometer/relative_angle_velocity',
    (state) => {
      AngularVelsState = state;
    }
  );

  node.spin();
}

async function sub_dist() {
  const node = rclnodejs.createNode('ultra_sub');

  node.createSubscription(
    'system_msgs/msg/DistanceVals',
    'ultrasonic/distance',
    (state) => {
      ultra_distance = state;
    }
  );

  node.spin();
}

async function send_ros_motor_cmd(motor1_speed, motor2_speed, node, publisher) {
  
  let msgObject = rclnodejs.createMessageObject('system_msgs/msg/MotorCommand');

  msgObject={
    is_pwm : true,
    mot_1_req_rad_sec : motor1_speed,
    mot_2_req_rad_sec : motor2_speed}

  if(ultra_distance.center < 25 && (motor1_speed < 0 || motor2_speed < 0)){
    msgObject={
      is_pwm : true,
      mot_1_req_rad_sec : 0,
      mot_2_req_rad_sec : 0}
  }

  publisher.publish(msgObject);
  node.spinOnce();

}

const delay = (delayInms) => {
  return new Promise(resolve => setTimeout(resolve, delayInms));
};

var forward_until_wall_flag = false;

async function forward_until_wall_motor_cmd(node, publisher) {
  
  let msgObject = rclnodejs.createMessageObject('system_msgs/msg/MotorCommand');

  forward_until_wall_flag = true;

  while(ultra_distance.center > 25 && forward_until_wall_flag){
    msgObject={
      is_pwm : true,
      mot_1_req_rad_sec : -195,
      mot_2_req_rad_sec : -202}
      
    publisher.publish(msgObject);
    node.spinOnce();

    let delayres = await delay(250);
  }

  msgObject={
    is_pwm : true,
    mot_1_req_rad_sec : 0,
    mot_2_req_rad_sec : 0}
    
  publisher.publish(msgObject);
  node.spinOnce();

}

app.listen(port, async() => {
  console.log(`Server is running on port ${port}`);
  await rclnodejs.init();

  const node = rclnodejs.createNode('ros_server_node');
  const publisher = node.createPublisher('system_msgs/msg/MotorCommand', 'motor_command');

  //start subs
  sub_motor_speed();
  sub_acce();
  sub_dist();
  sub_dht();

  app.post("/motor_command", async (req, res) => {
    try {

      const { motor1_speed, motor2_speed } = req.body;
  
      //console.log(`Motor speed command: ${motor1_speed} ${motor2_speed}`);
      send_ros_motor_cmd(motor1_speed * -1, motor2_speed * -1, node, publisher);
  
      res.json(`Motor speed command: ${motor1_speed} ${motor2_speed}`);
  
    } catch (err) {
      console.error(err.message);
    }
  });

  app.post("/motor_forward_until_wall_command", async (req, res) => {
    try {
  
      forward_until_wall_motor_cmd(node, publisher);
      res.json(`Forward until wall sent!`);
  
    } catch (err) {
      console.error(err.message);
    }
  });

  app.post("/stop_forward_until_wall_command", async (req, res) => {
    try {
  
      forward_until_wall_flag = false;

      res.json(`Stopped forward until wall sent!`);
  
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_speed", async (req, res) => {
    try {
      //console.log(`Get motor speed: ${motorSpeedState.mot_1_rad_sec * -1} ${motorSpeedState.mot_2_rad_sec}`);
      var avg_speed = ((motorSpeedState.mot_1_rad_sec * -1) + motorSpeedState.mot_2_rad_sec) / 2
      res.json(avg_speed);
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_acce", async (req, res) => {
    try {
      res.json(AccelerationVelsState);
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_ang", async (req, res) => {
    try {
      res.json(AngularVelsState);
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_ultra_dist", async (req, res) => {
    try {
      res.json(ultra_distance);
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_temp", async (req, res) => {
    try {
      res.json(temperatureState.temperature_celcius);
    } catch (err) {
      console.error(err.message);
    }
  });

  app.get("/get_hum", async (req, res) => {
    try {
      res.json(humidityState.humidity);
    } catch (err) {
      console.error(err.message);
    }
  });

});