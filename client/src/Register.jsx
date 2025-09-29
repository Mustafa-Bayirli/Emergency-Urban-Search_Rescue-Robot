import React from 'react';
import Background from './assets/background.png';
import Radar from './assets/radar.png';

function Register() {
  return (
    <div className="d-flex justify-content-center align-items-center flex-column" style={{ height: '100vh' }}>
      <div class="form-box">
        <form class="form">
        <span class="title">Sign up</span>
            <span class="subtitle">Create a free account with your email.</span>
            <div class="form-container">
              <input type="text" class="input" placeholder="Full Name"></input>
              <input type="email" class="input" placeholder="Email"></input>
              <input type="password" class="input" placeholder="Password"></input>
            </div>
            <button>Sign up</button>
        </form>
        <div class="form-section">
            <p>Have an account? <a href="/Login">Log in</a> </p>
        </div>
    </div>
     
    </div>
  );
}

export default Register;
