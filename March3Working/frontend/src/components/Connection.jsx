import React, { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import './Connection.css'

// Connection states
const STATE = {
  SIMULATION: 'simulation',
  CONNECTING: 'connecting',
  CONNECTED: 'connected'
}

// Serial baud for LoRa USB bridge — must match firmware on the bridge radio
const BAUD_OPTIONS = [4800, 9600, 19200, 38400, 57600, 115200]

// json = CircuitPython Pico on USB (newline JSON). Same mode for: boat plugged in directly,
// OR ground Pico running lora_ground_bridge_code.py (LoRa to boat — PC never sees RF).
// mdot = MultiTech mDot AT (Micro UDK)
const MODEM_OPTIONS = [
  {
    value: 'json',
    label: 'Pico / JSON (boat USB or ground LoRa bridge)',
  },
  { value: 'mdot', label: 'mDot MTDOT-915 (Micro UDK, AT)' },
]

function Connection({ connected, simulation, onConnect, onDisconnect }) {
  const [ports, setPorts] = useState([])
  const [selectedPort, setSelectedPort] = useState('')
  const [baudRate, setBaudRate] = useState(9600)
  const [modemType, setModemType] = useState('json')
  const [connectionState, setConnectionState] = useState(STATE.SIMULATION)
  const [error, setError] = useState('')
  const [connectingMessage, setConnectingMessage] = useState('')

  // Sync state with props
  useEffect(() => {
    if (simulation) {
      setConnectionState(STATE.SIMULATION)
    }
  }, [simulation])

  // Fetch available COM ports
  const refreshPorts = async () => {
    setError('')
    try {
      const response = await fetch('http://localhost:8000/ports')
      const data = await response.json()
      setPorts(data.ports || [])
      if (data.ports?.length > 0 && !selectedPort) {
        setSelectedPort(data.ports[0])
      }
    } catch (err) {
      setError('Failed to fetch ports')
      setPorts([])
    }
  }

  // Fetch ports on mount
  useEffect(() => {
    refreshPorts()
  }, [])

  // Default 9600; use 115200 if your UDK needs it (GET /probe/mdot?port=COMx)
  useEffect(() => {
    setBaudRate(9600)
  }, [modemType])

  const handleConnect = async () => {
    if (!selectedPort) {
      setError('Select a port first')
      return
    }
    
    setError('')
    setConnectionState(STATE.CONNECTING)
    setConnectingMessage('Opening serial port...')
    
    try {
      // Step 1: Open serial connection
      const connectResponse = await fetch(
        `http://localhost:8000/connect/${encodeURIComponent(selectedPort)}?baud=${baudRate}&modem=${encodeURIComponent(modemType)}`,
        { method: 'POST' }
      )
      const connectData = await connectResponse.json()
      
      if (!connectData.success) {
        setError('Failed to open serial port')
        setConnectionState(STATE.SIMULATION)
        return
      }
      
      setConnectingMessage('Waiting for boat response...')
      
      // Step 2: Perform handshake with boat
      const handshakeResponse = await fetch('http://localhost:8000/handshake', {
        method: 'POST'
      })
      const handshakeData = await handshakeResponse.json()
      
      if (handshakeData.success && handshakeData.boat_connected) {
        setConnectionState(STATE.CONNECTED)
        onConnect(selectedPort)
      } else {
        setError(handshakeData.error || 'Boat did not respond')
        // Disconnect serial since boat didn't respond
        await fetch('http://localhost:8000/disconnect', { method: 'POST' })
        setConnectionState(STATE.SIMULATION)
      }
      
    } catch (err) {
      setError('Connection error')
      setConnectionState(STATE.SIMULATION)
    }
  }

  const handleQuickConnect = async () => {
    if (!selectedPort) {
      setError('Select a port first')
      return
    }
    
    setError('')
    setConnectionState(STATE.CONNECTING)
    setConnectingMessage('Quick connecting (no handshake)...')
    
    try {
      const response = await fetch(
        `http://localhost:8000/quickconnect/${encodeURIComponent(selectedPort)}?baud=${baudRate}&modem=${encodeURIComponent(modemType)}`,
        { method: 'POST' }
      )
      const data = await response.json()
      
      if (data.success) {
        setConnectionState(STATE.CONNECTED)
        onConnect(selectedPort)
      } else {
        setError('Failed to open serial port')
        setConnectionState(STATE.SIMULATION)
      }
    } catch (err) {
      setError('Connection error')
      setConnectionState(STATE.SIMULATION)
    }
  }

  const handleDisconnect = async () => {
    setConnectionState(STATE.CONNECTING)
    setConnectingMessage('Disconnecting...')
    try {
      await fetch('http://localhost:8000/disconnect', { method: 'POST' })
    } catch (err) {
      // Continue even if request fails
    }
    setConnectionState(STATE.SIMULATION)
    onDisconnect()
  }

  const sendCommand = async (cmdType) => {
    try {
      await fetch('http://localhost:8000/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ type: cmdType })
      })
    } catch (err) {
      console.error('Command failed:', err)
    }
  }

  const isConnecting = connectionState === STATE.CONNECTING
  const isConnected = connectionState === STATE.CONNECTED
  const isSimulation = connectionState === STATE.SIMULATION

  return (
    <div className="connection-panel">
      <div className="panel-header">
        <span className="panel-icon">⚡</span>
        <h2>CONNECTION</h2>
      </div>

      <div className="connection-content">
        {/* Status indicator */}
        {isConnecting ? (
          // Connecting animation
          <div className="connection-status connecting">
            <div className="connecting-animation">
              <motion.div 
                className="connecting-ring"
                animate={{ rotate: 360 }}
                transition={{ duration: 1.5, repeat: Infinity, ease: "linear" }}
              />
              <motion.div 
                className="connecting-pulse"
                animate={{ 
                  scale: [1, 1.5, 1],
                  opacity: [0.8, 0.2, 0.8]
                }}
                transition={{ duration: 1.2, repeat: Infinity, ease: "easeInOut" }}
              />
            </div>
            <div className="status-text">
              <span className="status-label">CONNECTING</span>
              <span className="status-detail">{connectingMessage}</span>
            </div>
          </div>
        ) : (
          // Normal status
          <div className={`connection-status ${isSimulation ? 'simulation' : 'live'}`}>
            <motion.div 
              className="status-indicator"
              animate={{ 
                scale: [1, 1.2, 1],
                opacity: [1, 0.7, 1]
              }}
              transition={{ 
                duration: 2, 
                repeat: Infinity,
                ease: "easeInOut"
              }}
            />
            <div className="status-text">
              <span className="status-label">
                {isSimulation ? 'SIMULATION MODE' : 'CONNECTED TO BOAT'}
              </span>
              <span className="status-detail">
                {isSimulation
                  ? 'Using simulated data'
                  : `Port: ${selectedPort} @ ${baudRate} baud (${modemType === 'mdot' ? 'mDot AT' : 'JSON'})`}
              </span>
            </div>
          </div>
        )}

        {/* Port selection */}
        <div className="port-section">
          <label className="port-label">USB SERIAL PORT</label>
          <div className="port-row">
            <select 
              className="port-select"
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              disabled={!isSimulation}
            >
              <option value="">Select COM Port...</option>
              {ports.map(port => (
                <option key={port} value={port}>{port}</option>
              ))}
            </select>
            <button 
              className="refresh-btn"
              onClick={refreshPorts}
              disabled={!isSimulation}
              title="Refresh ports"
            >
              ↻
            </button>
          </div>
          <label className="port-label" style={{ marginTop: '0.75rem' }}>DEVICE / PROTOCOL</label>
          <div className="port-row">
            <select
              className="port-select"
              value={modemType}
              onChange={(e) => setModemType(e.target.value)}
              disabled={!isSimulation}
              title="mDot uses AT (OK handshake). Pico bridge uses JSON ping/pong."
            >
              {MODEM_OPTIONS.map((m) => (
                <option key={m.value} value={m.value}>{m.label}</option>
              ))}
            </select>
          </div>
          <label className="port-label" style={{ marginTop: '0.75rem' }}>BAUD RATE</label>
          <div className="port-row">
            <select
              className="port-select"
              value={baudRate}
              onChange={(e) => setBaudRate(Number(e.target.value))}
              disabled={!isSimulation}
              title="Must match UART (mDot UDK often 115200)"
            >
              {BAUD_OPTIONS.map((b) => (
                <option key={b} value={b}>{b}</option>
              ))}
            </select>
          </div>
        </div>

        {/* Error message */}
        {error && (
          <motion.div 
            className="error-message"
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
          >
            {error}
          </motion.div>
        )}

        {/* Connect/Disconnect button */}
        {isSimulation && (
          <div className="connect-buttons">
            <button 
              className="connect-btn"
              onClick={handleConnect}
              disabled={!selectedPort}
            >
              CONNECT TO BOAT
            </button>
            <button 
              className="quick-connect-btn"
              onClick={handleQuickConnect}
              disabled={!selectedPort}
              title="Skip handshake - for testing"
            >
              QUICK CONNECT
            </button>
          </div>
        )}
        
        {isConnecting && (
          <button className="connect-btn connecting" disabled>
            <span className="btn-spinner" />
            CONNECTING...
          </button>
        )}
        
        {isConnected && (
          <div className="connected-controls">
            <div className="control-buttons">
              <button 
                className="control-btn stop-btn"
                onClick={() => sendCommand('stop')}
              >
                ⏹ STOP
              </button>
              <button 
                className="control-btn start-btn"
                onClick={() => sendCommand('start')}
              >
                ▶ START
              </button>
            </div>
            <button 
              className="disconnect-btn"
              onClick={handleDisconnect}
            >
              DISCONNECT
            </button>
          </div>
        )}

        {/* Help text */}
        <div className="connection-help">
          {isSimulation && (
            <p>
              <strong>No separate “LoRa” switch:</strong> the PC always uses USB serial. For{' '}
              <strong>LoRa</strong>, plug in the <strong>ground Pico</strong> (bridge firmware) and
              pick its COM port; for <strong>direct boat USB</strong>, plug in the boat Pico instead.
              Use <strong>Pico / JSON</strong> for both (not mDot). Baud is usually{' '}
              <strong>115200</strong> for CircuitPython. Or choose <strong>mDot</strong> for Micro UDK
              (AT handshake).
            </p>
          )}
          {isConnecting && (
            <p>Sending ping to boat and waiting for response...</p>
          )}
          {isConnected && (
            <p>Receiving live data from boat. Disconnect to return to simulation mode.</p>
          )}
        </div>
      </div>
    </div>
  )
}

export default Connection
