import React from 'react';
import Background from './assets/background.png';
import Radar from './assets/radar.png';
import Sebastian from './assets/sebastian_larrivee.webp';
import Daniel from './assets/Daniel2.png';
import Alexander from './assets/Alexander2.jpg';
import Eric from './assets/Eric2.jpg';
import Mustafa from './assets/Mustafa.jpg';
import Laith from './assets/Laith.jpg';

function Home() {


  return (
    <div className="d-flex justify-content-center align-items-center flex-column" style={{  }}>
      <div className="d-flex align-items-center" style={{ marginBottom: '60px' }}>
        <div class="loader">
          <span></span>
        </div>

        <h1 style={{ marginBottom: '0' }}>SEARCH AND RESCUE ROBOTICS</h1>
      </div>
      <h1> About Us </h1>
      <div className="d-flex align-items-center" style={{ marginBottom: '60px' }}>
        <p className="lead text-primary" style={{ marginBottom: '0', margineLeft:'10px', marginRight: '10px'}}>A robot that would aid with search and rescue efforts within an urban environment.

          Urban Search and Rescue (USAR) refers to technical rescue operations involving the location, extraction, and initial medical stabilization of victims trapped in an urban environment

          Compromised urban environments can be comprised of collapsed or partially collapsed buildings, gas or chemical leaks, or compromised tunnels or trenches.

          These environments can be caused by natural disasters, war, or accidents

          To reduce risks to rescuers and survivors, the robot can be deployed to scout an affected area using various sensors and establish communication between a rescue worker and a survivor.

          A rescue robot can access places in which human rescue personnel cannot and will be able to explore an area for an extended period on time.
        </p>
      </div>

     
    <img
        src={Background}
        alt="background"
        style={{
          position: 'fixed',
          bottom: 0,
          left: '50%',
          transform: 'translateX(-50%)',
          maxWidth: '100%',
          opacity:'0.2'
        }}
      />


      <div className="boxes">
        <h1>Team Members</h1>
        <div className="boxes-info">
          <div>
            <img src={Sebastian} alt="Team Member" />
            <p>Sebastian Larrivee</p>
          </div>
          <div>
            <img src={Daniel} alt="Team Member" />
            <p>Daniel Zhao</p>
          </div>
          <div>
            <img src={Eric} alt="Team Member" />
            <p>Eric Lin</p>
          </div>
          <div>
            <img src={Alexander} alt="Team Member" />
            <p>Alexander Joe</p>
          </div>
          <div>
            <img src={Laith} alt="Team Member" />
            <p>Laith Freije</p>
          </div>
          <div>
            <img src={Mustafa} alt="Team Member" />
            <p>Mustafa Bayirli</p>
          </div>
        </div>
      </div>
      

    </div>

  );
}

export default Home;
