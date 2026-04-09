import React, { useState, useEffect, useCallback, useRef } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import Map from './components/Map'
import Radar from './components/Radar'
import Telemetry from './components/Telemetry'
import Controls from './components/Controls'
import Connection from './components/Connection'
import Header from './components/Header'
import { normalizeTelemetry } from './telemetryNormalize'
import './App.css'

// Match throttle_test.py: 5% per step; throttle stays until S / Space / X lowers it (no auto-decay).
const THROTTLE_STEP = 0.05

function App() {
  const [telemetry, setTelemetry] = useState({
    latitude: 32.7872,
    longitude: -117.2350,
    heading: 0,
    speed: 0,
    battery: 100,
    satellites: 0,
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
  // Avoid `[controls]` on the interval effect — it was resetting every tick (rudder felt laggy).
  const controlsRef = useRef({ throttle: 0, steering: 0 })
  const lastSentControlRef = useRef(null)
  
  // WebSocket connection
  useEffect(() => {
    const connectWebSocket = () => {
      const ws = new WebSocket(`ws://${window.location.hostname}:8000/ws`)
      
      ws.onopen = () => {
        console.log('🚤 Connected to boat control server')
        setConnected(true)
      }
      
      ws.onmessage = (event) => {
        try {
          const raw = JSON.parse(event.data)
          const data = normalizeTelemetry(raw)
          setTelemetry(data)
          setTrail((prev) => {
            if (!Number.isFinite(data.latitude) || !Number.isFinite(data.longitude)) {
              return prev
            }
            const newTrail = [...prev, [data.latitude, data.longitude]]
            return newTrail.slice(-200)
          })
        } catch (e) {
          console.warn('Telemetry message skipped:', e)
        }
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

  const emergencyStop = useCallback(() => {
    controlsRef.current = { throttle: 0, steering: 0 }
    lastSentControlRef.current = { throttle: 0, steering: 0 }
    setControls({ throttle: 0, steering: 0 })
    sendCommand(0, 0)
  }, [sendCommand])
  
  // Keyboard controls
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.repeat) return
      keysPressed.current.add(e.key.toLowerCase())
      
      if (e.key === ' ' || e.key.toLowerCase() === 'x') {
        controlsRef.current = { throttle: 0, steering: 0 }
        lastSentControlRef.current = { throttle: 0, steering: 0 }
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
  
  // Control update loop — stable interval; rudder is −1 / 0 / +1 from A / none / D only
  useEffect(() => {
    const interval = setInterval(() => {
      let { throttle, steering } = controlsRef.current
      const keys = keysPressed.current
      const maxThrottle = throttleLimit / 100

      if (keys.has('w')) {
        throttle = Math.min(maxThrottle, throttle + THROTTLE_STEP)
      } else if (keys.has('s')) {
        throttle = Math.max(0, throttle - THROTTLE_STEP)
      }
      // No decay when keys released — same as bench throttle_test (use S or Space/X to come down).

      throttle = Math.min(throttle, maxThrottle)
      
      if (keys.has('a')) {
        steering = -1
      } else if (keys.has('d')) {
        steering = 1
      } else {
        steering = 0
      }
      
      controlsRef.current = { throttle, steering }
      setControls({ throttle, steering })

      const hasKeyInput =
        keys.has('w') || keys.has('s') || keys.has('a') || keys.has('d')
      const idle =
        !hasKeyInput && Math.abs(throttle) < 0.001 && steering === 0

      if (idle) {
        const prev = lastSentControlRef.current
        if (prev === null) {
          lastSentControlRef.current = { throttle: 0, steering: 0 }
          return
        }
        if (Math.abs(prev.throttle) < 0.001 && prev.steering === 0) {
          return
        }
        lastSentControlRef.current = { throttle: 0, steering: 0 }
        sendCommand(0, 0)
        return
      }

      const prev = lastSentControlRef.current
      const changed =
        prev === null ||
        Math.abs((prev.throttle ?? 0) - throttle) > 1e-5 ||
        prev.steering !== steering
      if (changed) {
        lastSentControlRef.current = { throttle, steering }
        sendCommand(throttle, steering)
      }
    }, 50)
    
    return () => clearInterval(interval)
  }, [sendCommand, throttleLimit])

  const nudgeThrottle = useCallback(
    (delta) => {
      const maxT = throttleLimit / 100
      const cur = controlsRef.current
      const t = Math.min(maxT, Math.max(0, cur.throttle + delta))
      const steering = cur.steering
      controlsRef.current = { throttle: t, steering }
      setControls({ throttle: t, steering })
      lastSentControlRef.current = { throttle: t, steering }
      sendCommand(t, steering)
    },
    [sendCommand, throttleLimit]
  )

  useEffect(() => {
    const onBlur = () => {
      keysPressed.current.clear()
    }
    window.addEventListener('blur', onBlur)
    return () => window.removeEventListener('blur', onBlur)
  }, [])
  
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
            onEmergencyStop={emergencyStop}
            onThrottleNudge={nudgeThrottle}
            throttleStep={THROTTLE_STEP}
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
            satellites={telemetry.satellites || 0}
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
          W/S ±5% throttle (holds) • A/D rudder • Space / X stop
        </span>
      </footer>
    </div>
  )
}

export default App

