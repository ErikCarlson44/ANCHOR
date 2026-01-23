import React, { useState, useEffect } from 'react'
import { motion } from 'framer-motion'
import './Connection.css'

function Connection({ connected, simulation, onConnect, onDisconnect }) {
  const [ports, setPorts] = useState([])
  const [selectedPort, setSelectedPort] = useState('')
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState('')

  // Fetch available COM ports
  const refreshPorts = async () => {
    setLoading(true)
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
    setLoading(false)
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
    
    setLoading(true)
    setError('')
    try {
      const response = await fetch(`http://localhost:8000/connect/${selectedPort}`, {
        method: 'POST'
      })
      const data = await response.json()
      if (data.success) {
        onConnect(selectedPort)
      } else {
        setError('Failed to connect')
      }
    } catch (err) {
      setError('Connection error')
    }
    setLoading(false)
  }

  const handleDisconnect = async () => {
    setLoading(true)
    try {
      await fetch('http://localhost:8000/disconnect', { method: 'POST' })
      onDisconnect()
    } catch (err) {
      // Still update UI even if request fails
      onDisconnect()
    }
    setLoading(false)
  }

  return (
    <div className="connection-panel">
      <div className="panel-header">
        <span className="panel-icon">⚡</span>
        <h2>CONNECTION</h2>
      </div>

      <div className="connection-content">
        {/* Status indicator */}
        <div className={`connection-status ${simulation ? 'simulation' : 'live'}`}>
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
              {simulation ? 'SIMULATION MODE' : 'CONNECTED TO BOAT'}
            </span>
            <span className="status-detail">
              {simulation ? 'Using simulated data' : `Port: ${selectedPort}`}
            </span>
          </div>
        </div>

        {/* Port selection */}
        <div className="port-section">
          <label className="port-label">LORA PORT</label>
          <div className="port-row">
            <select 
              className="port-select"
              value={selectedPort}
              onChange={(e) => setSelectedPort(e.target.value)}
              disabled={!simulation || loading}
            >
              <option value="">Select COM Port...</option>
              {ports.map(port => (
                <option key={port} value={port}>{port}</option>
              ))}
            </select>
            <button 
              className="refresh-btn"
              onClick={refreshPorts}
              disabled={loading}
              title="Refresh ports"
            >
              ↻
            </button>
          </div>
        </div>

        {/* Error message */}
        {error && (
          <div className="error-message">
            {error}
          </div>
        )}

        {/* Connect/Disconnect button */}
        {simulation ? (
          <button 
            className="connect-btn"
            onClick={handleConnect}
            disabled={!selectedPort || loading}
          >
            {loading ? 'CONNECTING...' : 'CONNECT TO BOAT'}
          </button>
        ) : (
          <button 
            className="disconnect-btn"
            onClick={handleDisconnect}
            disabled={loading}
          >
            {loading ? 'DISCONNECTING...' : 'DISCONNECT'}
          </button>
        )}

        {/* Help text */}
        <div className="connection-help">
          {simulation ? (
            <p>Connect your LORA module via USB and select the COM port to receive real boat data.</p>
          ) : (
            <p>Receiving live data from boat. Disconnect to return to simulation mode.</p>
          )}
        </div>
      </div>
    </div>
  )
}

export default Connection
