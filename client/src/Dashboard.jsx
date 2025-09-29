import React from 'react';
import { useEffect } from 'react';
import { useState } from 'react';
import html2canvas from 'html2canvas';

function Dashboard() {
  const [motor1_speed, setMotor1Speed] = useState(0);
  const [motor2_speed, setMotor2Speed] = useState(0);

  const [isWPressed, setIsWPressed] = useState(false);
  const [isAPressed, setIsAPressed] = useState(false);
  const [isSPressed, setIsSPressed] = useState(false);
  const [isDPressed, setIsDPressed] = useState(false);

  const [speed, setSpeed] = useState(0);

  const [accelerationVelsState, setAccelerationVelsState] = useState({x_velocity: 0, y_velocity: 0, z_velocity: 0});
  const [ultraDistState, setUltraDistState] = useState({right: 0.0, center: 0.0, left: 0.0});
  const [humidityState, setHumidityState] = useState(0);
  const [temperatureState, setTemperatureState] = useState(0);

  useEffect(() => {
    const fetchSpeedData = async () => {
      try {
        const response1 = await fetch(`http://localhost:3000/get_speed`)
          .then((response1) => response1.json())
          .then((data1) => { setSpeed(data1); });
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };

    const fetchSensorData = async () => {
      try {
        const response2 = await fetch(`http://localhost:3000/get_acce`)
          .then((response2) => response2.json())
          .then((data2) => { setAccelerationVelsState(data2); });
        const response4 = await fetch(`http://localhost:3000/get_ultra_dist`)
          .then((response4) => response4.json())
          .then((data4) => { setUltraDistState(data4); });
        const response5 = await fetch(`http://localhost:3000/get_hum`)
          .then((response5) => response5.json())
          .then((data5) => { setHumidityState(data5); });
        const response6 = await fetch(`http://localhost:3000/get_temp`)
          .then((response6) => response6.json())
          .then((data6) => { setTemperatureState(data6); });
      } catch (error) {
        console.error('Error fetching data:', error);
      }
    };

    fetchSpeedData();
    fetchSensorData();

    const intervalId1 = setInterval(fetchSpeedData, 10);
    const intervalId2 = setInterval(fetchSensorData, 1000);

    return () => {
      clearInterval(intervalId1);
      clearInterval(intervalId2);
    };
  }, []);


  useEffect(() => {
    let keyPressTimeouts = {};

    function handleKeyDown(e) {
      const key = e.keyCode;

      if (!keyPressTimeouts[key]) {
        keyPressTimeouts[key] = setTimeout(() => {
          clearTimeout(keyPressTimeouts[key]);
          keyPressTimeouts[key] = null;

          let newMotor1Speed = 0;
          let newMotor2Speed = 0;

          //Combinations
          if (isWPressed && isAPressed) {
            newMotor1Speed += 200;
            newMotor2Speed += 100;
            console.log("Keyboard A + W")
          } else if (isWPressed && isDPressed) {
            newMotor1Speed += 100;
            newMotor2Speed += 200;
            console.log("Keyboard W + D")
          } else if (isSPressed && isAPressed) {
            newMotor1Speed -= 200;
            newMotor2Speed -= 100;
            console.log("Keyboard S + A")
          } else if (isSPressed && isDPressed) {
            newMotor1Speed -= 100;
            newMotor2Speed -= 200;
            console.log("Keyboard S + D")
          } else {
            //Single Keys
            if (isWPressed) {
              newMotor1Speed += 200;
              newMotor2Speed += 200;
              console.log("Keyboard W")
            }
            if (isAPressed) {
              newMotor1Speed += 100;
              newMotor2Speed -= 100;
              console.log("Keyboard A")
            }
            if (isSPressed) {
              newMotor1Speed -= 200;
              newMotor2Speed -= 200;
              console.log("Keyboard S")
            }
            if (isDPressed) {
              newMotor1Speed -= 100;
              newMotor2Speed += 100;
              console.log("Keyboard D")
            }
          }

          setMotor1Speed(newMotor1Speed);
          setMotor2Speed(newMotor2Speed);

          console.log(newMotor1Speed, newMotor2Speed);
          sendMotorCommand(newMotor1Speed, newMotor2Speed);
        }, 250); // 250ms delay for FSM
      }

      switch (key) {
        case 87: // W
          setIsWPressed(true);
          break;
        case 65: // A
          setIsAPressed(true);
          break;
        case 83: // S
          setIsSPressed(true);
          break;
        case 68: // D
          setIsDPressed(true);
          break;
        default:
          break;
      }
    }

    function handleKeyUp(e) {

      switch (e.keyCode) {
        case 87: // W
          setIsWPressed(false);
          break;
        case 65: // A
          setIsAPressed(false);
          break;
        case 83: // S
          setIsSPressed(false);
          break;
        case 68: // D
          setIsDPressed(false);
          break;
        default:
          break;
      }
    }

    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);

    // Don't forget to clean up
    return function cleanup() {
      document.removeEventListener('keydown', handleKeyDown);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, [isWPressed, isAPressed, isSPressed, isDPressed]);

  //Screenshot Function 
  function captureScreenshot() {
    const targetElement = document.body;
    console.log("screenshotting");
    html2canvas(targetElement).then(canvas => {
      const screenshotDataUrl = canvas.toDataURL('image/png');
      
      fetch('http://localhost:5000/save-screenshot', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ image: screenshotDataUrl }),
      })
        .then(response => response.json())
        .then(data => {
          console.log(data);
        });
    });
  }

  function donothing() {
    console.log("Hello");
  }

  function stop(){
    console.log("Stopping");
  }

  async function sendMotorForwardUntilWallCommand() {
    const data = {};

    try {
      const response = await fetch(
        `http://localhost:3000/motor_forward_until_wall_command`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(data),
        }
      );

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const responseData = await response.json();
      console.log(responseData);
      return responseData;

    } catch (err) {

      console.error("Error sending motor command:", err.message);
      throw err;
    }
  }

  async function sendStopForwardUntilWallCommand() {
    const data = {};

    try {
      const response = await fetch(
        `http://localhost:3000/stop_forward_until_wall_command`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(data),
        }
      );

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const responseData = await response.json();
      console.log(responseData);
      return responseData;

    } catch (err) {

      console.error("Error sending motor command:", err.message);
      throw err;
    }
  }

  async function sendMotorCommand(motor1_speed, motor2_speed) {
    const data = { motor1_speed, motor2_speed };

    try {
      const response = await fetch(
        `http://localhost:3000/motor_command`,
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(data),
        }
      );

      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }

      const responseData = await response.json();
      console.log(responseData);
      return responseData;

    } catch (err) {

      console.error("Error sending motor command:", err.message);
      throw err;
    }
  }

  return (
    <div className="container-fluid" style={{ marginTop: '50px' }}>
      <div className="header" style={{ marginTop: '20px' }}>
        <div className="row">
          <div className="col-md-9 mx-auto">
            <div className="row">
              <div className="col-md xy-auto">
                <div className="row" style={{ width: '900px' }} >
                  <h1>
                    Video Feed
                  </h1>
                </div>
                <div className="row" style={{ width: '900px', marginBottom: '15px' }} >
                  <iframe className="col-md-12 flip" src="http://192.168.8.229:8081/" controls="controls" style={{ height: '500px', overflow: 'hidden' }} />
                </div>
              </div>
            </div>
            <div className="row xy-auto">
              <div className="col-md xy-auto">
                <div className="col-md xy-auto">
                  <h5>Telemetry</h5>
                </div>
                <div className="d-flex justify-content-around pt-2" style={{ backgroundColor: 'lightgrey', height: '170px', width: '900px' }}>
                  <div className="d-flex flex-column w-100 justify-content-start">
                    <h6 className="mb-1">Distance from Robot</h6>
                    <p className={(ultraDistState.left > 50) ? 'mb-1 text-success textB' : ((ultraDistState.left > 25) ? 'mb-1 text-warning textB' : 'mb-1 text-danger textB')}>
                      <strong>Left: </strong> {ultraDistState.left.toFixed(2)}cm</p>
                    <p className={(ultraDistState.center > 50) ? 'mb-1 text-success textB' : ((ultraDistState.center > 25) ? 'mb-1 text-warning textB' : 'mb-1 text-danger textB')}>
                      <strong>Center: </strong> {ultraDistState.center.toFixed(2)}cm</p>
                    <p className={(ultraDistState.right > 50) ? 'mb-1 text-success textB' : ((ultraDistState.right > 25) ? 'mb-1 text-warning textB' : 'mb-1 text-danger textB')}>
                      <strong>Right: </strong> {ultraDistState.right.toFixed(2)}cm</p>
                  </div>
                  <div className="d-flex flex-column w-100 justify-content-start">
                    <h6 className="mb-1">Acceleration</h6>
                    <p className="mb-1"><strong>X-Axis: </strong> {accelerationVelsState.x_velocity.toFixed(2)}</p>
                    <p className="mb-1"><strong>Y-Axis: </strong> {accelerationVelsState.y_velocity.toFixed(2)}</p>
                    <p className="mb-1"><strong>Z-Axis: </strong> {accelerationVelsState.z_velocity.toFixed(2)}</p>
                  </div>
                  <div className="d-flex flex-column w-100 justify-content-start">
                    <h6 className="mb-1">Enviroment</h6>
                    <p className={(temperatureState < 30) ? 'mb-1 text-success' : ((temperatureState < 50) ? 'mb-1 text-warning' : 'mb-1 text-danger')}>
                      <strong>Temperature: </strong> {temperatureState.toFixed(2)}C°</p>
                    <p className="mb-1"><strong>Humidity: </strong> {humidityState.toFixed(2)}%</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
          <div className="col-md-3 mx-auto" style={{ marginTop: '22px' }}>
            <div className="row">
              <div className="col-md-12 mx-auto">
              <div style={{ backgroundColor: 'lightgrey', height: '310px', marginBottom: '20px' }}>
                <h2>Location </h2>
                <div className="mx-auto" style={{width: 250, height: 250, overflow: 'hidden'}}>
                  <iframe id="myIframe" className='iRelative rot90' src="http://localhost:8888" width="500px" height="600px" ></iframe>
                </div>
              </div>
             
                <div style={{ backgroundColor: 'lightgrey', height: '100px', marginBottom: '20px' }}>
                  <h2>Speed </h2>

                  <div className="row pb-2 pt-2">
                    <div className="row-md-4 mx-auto">
                      <h3 className={(speed == 0) ? 'text-center text-success' : ((speed < 4) ? 'text-center text-warning' : 'text-center text-danger')}>
                        {speed.toFixed(2)} rad/s
                      </h3>
                    </div>
                  </div>
                </div>
              </div>
            </div>
            <div className="row">
              <div className="col-md-12">
                <div style={{ backgroundColor: 'lightgrey', height: '140px', marginBottom: '20px' }}>
                  <h2>Commands</h2>
                  <div className="row-md-4 mx-auto">
                    <button type="button" className="btn btn-primary float-right" onClick={stop}>
                    STOP
                    </button>
                    <button type="button" className="btn btn-primary float-right" onClick={captureScreenshot}>
                      Screenshot
                    </button>
                    <button type="button" className="btn btn-primary float-right" onClick={sendMotorForwardUntilWallCommand}>
                      Smart Forward
                    </button>
                    <button type="button" className="btn btn-primary float-right" onClick={sendStopForwardUntilWallCommand}>
                      Stop Forward
                    </button>
                  </div>
                </div>
              </div>
            </div>
            <div className="row">
              <div className="col-md-12">
                <div className="bg-secondary" style={{ backgroundColor: 'lightgrey', height: '140px' }}>
                  <h2>Controls </h2>
                  <div className="row pt-1">
                    <div className="col-md-4 mx-auto">
                      <button type="button" className={isWPressed ? 'btn btn-warning w-100' : 'btn btn-primary w-100'}>W</button>
                    </div>
                  </div>
                  <div className="row pt-1">
                    <div className="col-md-4 mx-auto">
                      <button type="button" className={isAPressed ? 'btn btn-warning w-100' : 'btn btn-primary w-100'}>A</button>
                    </div>
                    <div className="col-md-4 mx-auto">
                      <button type="button" className={isSPressed ? 'btn btn-warning w-100' : 'btn btn-primary w-100'}>S</button>
                    </div>
                    <div className="col-md-4 mx-auto">
                      <button type="button" className={isDPressed ? 'btn btn-warning w-100' : 'btn btn-primary w-100'}>D</button>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div >
  );
}

export default Dashboard;