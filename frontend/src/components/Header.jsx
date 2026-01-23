import React from 'react'
import { motion } from 'framer-motion'
import './Header.css'

function Header({ connected, simulation }) {
  return (
    <header className="header">
      <motion.div 
        className="logo"
        initial={{ opacity: 0, y: -20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5 }}
      >
        <div className="logo-icon">
          <svg viewBox="0 0 40 40" fill="none">
            <path
              d="M20 5L35 32H5L20 5Z"
              fill="url(#boatGradient)"
              stroke="var(--accent-primary)"
              strokeWidth="1.5"
            />
            <path
              d="M20 12L28 28H12L20 12Z"
              fill="var(--bg-primary)"
              opacity="0.5"
            />
            <defs>
              <linearGradient id="boatGradient" x1="20" y1="5" x2="20" y2="32">
                <stop offset="0%" stopColor="var(--accent-primary)" stopOpacity="0.8" />
                <stop offset="100%" stopColor="var(--accent-tertiary)" stopOpacity="0.3" />
              </linearGradient>
            </defs>
          </svg>
        </div>
        <div className="logo-text">
          <h1>ANCHOR</h1>
          <span className="logo-subtitle">NAVIGATION SYSTEM</span>
        </div>
      </motion.div>
      
      <motion.div 
        className="header-status"
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ duration: 0.5, delay: 0.3 }}
      >
        <div className={`connection-badge ${connected ? 'connected' : 'disconnected'}`}>
          <span className="badge-dot" />
          <span>{connected ? 'ONLINE' : 'OFFLINE'}</span>
        </div>
        
        <div className="mode-badge">
          <span className="mode-icon">{simulation ? '◉' : '●'}</span>
          <span>{simulation ? 'SIM' : 'LIVE'}</span>
        </div>
      </motion.div>
      
      <motion.div 
        className="header-time"
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ duration: 0.5, delay: 0.4 }}
      >
        <Clock />
      </motion.div>
    </header>
  )
}

function Clock() {
  const [time, setTime] = React.useState(new Date())
  
  React.useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000)
    return () => clearInterval(timer)
  }, [])
  
  return (
    <div className="clock">
      <span className="clock-time">
        {time.toLocaleTimeString('en-US', { hour12: false })}
      </span>
      <span className="clock-date">
        {time.toLocaleDateString('en-US', { 
          weekday: 'short', 
          month: 'short', 
          day: 'numeric' 
        })}
      </span>
    </div>
  )
}

export default Header

