import React, { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import './Connection.css'

// Connection states
const STATE = {
  SIMULATION: 'simulation',
  CONNECTING: 'connecting',
  CONNECTED: 'connected'
}

function Connection({ connected, simulation, onConnect, onDisconnect }) {
  const [ports, setPorts] = useState([])
  const [selectedPort, setSelectedPort] = useState('')
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
      const connectResponse = await fetch(`http://localhost:8000/connect/${selectedPort}`, {
        method: 'POST'
      })
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
      const response = await fetch(`http://localhost:8000/quickconnect/${selectedPort}`, {
        method: 'POST'
      })
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
                {isSimulation ? 'Using simulated data' : `Port: ${selectedPort}`}
              </span>
            </div>
          </div>
        )}

        {/* Port selection */}
        <div className="port-section">
          <label className="port-label">LORA PORT</label>
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
          <button 
            className="disconnect-btn"
            onClick={handleDisconnect}
          >
            DISCONNECT
          </button>
        )}

        {/* Help text */}
        <div className="connection-help">
          {isSimulation && (
            <p>Connect your LORA module via USB and select the COM port. The GUI will ping the boat and wait for acknowledgement.</p>
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
