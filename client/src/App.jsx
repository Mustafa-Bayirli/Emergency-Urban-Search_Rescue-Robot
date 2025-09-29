import React from 'react';
import Home from './Home';
import Dashboard from './Dashboard';
import Register from './Register';
import Analytics from './Analytics';
import Login from './Login';
import './App.css'


import {
  createBrowserRouter,
  RouterProvider,
} from "react-router-dom";

const router = createBrowserRouter([
  {
    path: "/",
    element: <Home />,
  },
  {
    path: "/dashboard",
    element: <Dashboard />,
  },
  {
    path: "/login",
    element: <Login />,
  },
  {
    path: "/analytics",
    element: <Analytics />,
  },
]);

function App() {
  return (
    <div>
      <div style={{marginBottom:'40px'}}>
        <nav>
          <ul>
          <li>Search & Rescue Robotics</li>
          <li><a class="text-decoration-none text-primary" href="/">Home</a></li>
          <li><a class="text-decoration-none" href="/Dashboard">Dashboard</a></li>
          <li><a class="text-decoration-none" href="/Analytics">Aanlytics</a></li>
          </ul>
        </nav>
      </div>
      <RouterProvider router={router}></RouterProvider>
    </div>
  );
}

export default App
