import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import Login from './pages/Login';
import SignIn from './pages/SignIn';
import SignUp from './pages/SignUp';

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/login" element={<Login />} />
        <Route path="/" element={<Login />} />
        <Route path="/login/signin" element={<SignIn />} />
        <Route path="/login/signup/email" element={<SignUp />} />
      </Routes>
    </Router>
  );
}

export default App;
