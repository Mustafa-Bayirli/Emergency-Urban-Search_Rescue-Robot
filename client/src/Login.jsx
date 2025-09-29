import React from 'react';
import Background from './assets/background.png';

function Login() {
  return (
    <div className="d-flex justify-content-center align-items-center flex-column" style={{ height: '100vh' }}>
      <div class="form-box">
        <form class="form">
        <span class="title">Login</span>
            <span class="subtitle">Enter your email and password to log in.</span>
            <div class="form-container">
              <input type="email" class="input" placeholder="Email"></input>
              <input type="password" class="input" placeholder="Password"></input>
            </div>
            <button>Log in</button>
        </form>
        <div class="form-section">
            <p>Want to create an account? <a href="/Register">Register</a> </p>
        </div>
    </div>
      
    </div>
  );
}

export default Login;
