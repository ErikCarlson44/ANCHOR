import React, { useState, useEffect, useCallback, useRef } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import Map from './components/Map'
import Radar from './components/Radar'
import Telemetry from './components/Telemetry'
import Controls from './components/Controls'
import Connection from './components/Connection'
import Header from './components/Header'
import './App.css'

function App() {
  const [telemetry, setTelemetry] = useState({
    latitude: 29.7604,
    longitude: -95.3698,
    heading: 0,
    speed: 0,
    battery: 100,
    obstacles: [],
    timestamp: Date.now()
  })
  
  const [connected, setConnected] = useState(false)
  const [simulation, setSimulation] = useState(true)
  const [connectedPort, setConnectedPort] = useState('')
  const [controls, setControls] = useState({ throttle: 0, steering: 0 })
  const [throttleLimit, setThrottleLimit] = useState(100) // Max throttle percentage
  const [trail, setTrail] = useState([])
  
  // Handle LORA connection
  const handleConnect = (port) => {
    setSimulation(false)
    setConnectedPort(port)
  }
  
  const handleDisconnect = () => {
    setSimulation(true)
    setConnectedPort('')
  }
  
  const wsRef = useRef(null)
  const keysPressed = useRef(new Set())
  
  // WebSocket connection
  useEffect(() => {
    const connectWebSocket = () => {
      const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`)
      
      ws.onopen = () => {
        console.log('🚤 Connected to boat control server')
        setConnected(true)
      }
      
      ws.onmessage = (event) => {
        const data = JSON.parse(event.data)
        setTelemetry(data)
        
        // Add to trail
        setTrail(prev => {
          const newTrail = [...prev, [data.latitude, data.longitude]]
          return newTrail.slice(-200) // Keep last 200 points
        })
      }
      
      ws.onclose = () => {
        console.log('Disconnected from server')
        setConnected(false)
        // Reconnect after 2 seconds
        setTimeout(connectWebSocket, 2000)
      }
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error)
      }
      
      wsRef.current = ws
    }
    
    connectWebSocket()
    
    return () => {
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [])
  
  // Send control commands
  const sendCommand = useCallback((throttle, steering) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({
        type: 'control',
        throttle,
        steering
      }))
    }
  }, [])
  
  // Keyboard controls
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return
      keysPressed.current.add(e.key.toLowerCase())
      
      if (e.key === ' ') {
        // Emergency stop
        setControls({ throttle: 0, steering: 0 })
        sendCommand(0, 0)
      }
    }
    
    const handleKeyUp = (e) => {
      keysPressed.current.delete(e.key.toLowerCase())
    }
    
    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)
    
    return () => {
      window.removeEventListener('keydown', handleKeyDown)
      window.removeEventListener('keyup', handleKeyUp)
    }
  }, [sendCommand])
  
  // Control update loop
  useEffect(() => {
    const interval = setInterval(() => {
      let { throttle, steering } = controls
      const keys = keysPressed.current
      const maxThrottle = throttleLimit / 100 // Convert percentage to 0-1
      
      // Throttle (0 to limit, no reverse on boats)
      if (keys.has('w')) {
        throttle = Math.min(maxThrottle, throttle + 0.15)  // Cap at limit
      } else if (keys.has('s')) {
        throttle = Math.max(0, throttle - 0.2)
      } else {
        // Gradual deceleration when no input
        throttle *= 0.95
        if (throttle < 0.02) throttle = 0
      }
      
      // Ensure throttle never exceeds limit
      throttle = Math.min(throttle, maxThrottle)
      
      // Steering (binary: full left, center, or full right)
      if (keys.has('a')) {
        steering = -1
      } else if (keys.has('d')) {
        steering = 1
      } else {
        steering = 0
      }
      
      setControls({ throttle, steering })
      sendCommand(throttle, steering)
    }, 50)
    
    return () => clearInterval(interval)
  }, [controls, sendCommand, throttleLimit])
  
  return (
    <div className="app">
      {/* Animated background */}
      <div className="bg-gradient" />
      <div className="bg-grid" />
      
      <Header connected={connected} simulation={simulation} />
      
      <main className="main-content">
        {/* Left Panel */}
        <motion.aside 
          className="left-panel"
          initial={{ x: -50, opacity: 0 }}
          animate={{ x: 0, opacity: 1 }}
          transition={{ duration: 0.6, ease: "easeOut" }}
        >
          <Telemetry data={telemetry} />
          <Controls 
            throttle={controls.throttle} 
            steering={controls.steering}
            keysPressed={keysPressed.current}
            throttleLimit={throttleLimit}
            onThrottleLimitChange={setThrottleLimit}
          />
        </motion.aside>
        
        {/* Center - Map */}
        <motion.section 
          className="map-container"
          initial={{ y: 30, opacity: 0 }}
          animate={{ y: 0, opacity: 1 }}
          transition={{ duration: 0.6, delay: 0.2, ease: "easeOut" }}
        >
          <Map 
            position={[telemetry.latitude, telemetry.longitude]}
            heading={telemetry.heading}
            obstacles={telemetry.obstacles}
            trail={trail}
          />
        </motion.section>
        
        {/* Right Panel */}
        <motion.aside 
          className="right-panel"
          initial={{ x: 50, opacity: 0 }}
          animate={{ x: 0, opacity: 1 }}
          transition={{ duration: 0.6, ease: "easeOut" }}
        >
          <Radar 
            heading={telemetry.heading}
            obstacles={telemetry.obstacles}
          />
          <Connection
            connected={connected}
            simulation={simulation}
            onConnect={handleConnect}
            onDisconnect={handleDisconnect}
          />
        </motion.aside>
      </main>
      
      {/* Status bar */}
      <footer className="status-bar">
        <span className="status-item">
          <span className={`status-dot ${connected ? 'online' : 'offline'}`} />
          {connected ? 'CONNECTED' : 'RECONNECTING...'}
        </span>
        <span className="status-item">
          MODE: {simulation ? 'SIMULATION' : `LIVE (${connectedPort})`}
        </span>
        <span className="status-item">
          UPDATE: 20Hz
        </span>
        <span className="status-item hint">
          W/S throttle • A/D rudder • SPACE all stop
        </span>
      </footer>
    </div>
  )
}

export default App

