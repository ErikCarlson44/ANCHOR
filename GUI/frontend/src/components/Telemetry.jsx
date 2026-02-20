import React from 'react'
import { motion } from 'framer-motion'
import './Telemetry.css'

function Telemetry({ data }) {
  const batteryColor = data.battery > 50 
    ? 'var(--success)' 
    : data.battery > 25 
      ? 'var(--warning)' 
      : 'var(--danger)'
  
  return (
    <div className="telemetry-panel">
      <div className="panel-header">
        <span className="panel-icon">◈</span>
        <h2>TELEMETRY</h2>
      </div>
      
      <div className="telemetry-content">
        {/* GPS Section */}
        <div className="telemetry-section">
          <div className="section-label">GPS POSITION</div>
          <div className="gps-grid">
            <div className="data-item">
              <span className="data-key">LAT</span>
              <span className="data-value">{data.latitude.toFixed(6)}°</span>
            </div>
            <div className="data-item">
              <span className="data-key">LON</span>
              <span className="data-value">{data.longitude.toFixed(6)}°</span>
            </div>
          </div>
        </div>
        
        {/* Navigation Section */}
        <div className="telemetry-section">
          <div className="section-label">NAVIGATION</div>
          <div className="nav-grid">
            <div className="data-item large">
              <span className="data-key">HEADING</span>
              <div className="heading-display">
                <span className="heading-value">{data.heading.toFixed(0).padStart(3, '0')}</span>
                <span className="heading-unit">°</span>
                <CompassIndicator heading={data.heading} />
              </div>
            </div>
            <div className="data-item large">
              <span className="data-key">SPEED</span>
              <div className="speed-display">
                <span className="speed-value">{data.speed.toFixed(1)}</span>
                <span className="speed-unit">kts</span>
              </div>
            </div>
          </div>
        </div>
        
        {/* Battery Section */}
        <div className="telemetry-section">
          <div className="section-label">POWER SYSTEM</div>
          <div className="battery-display">
            <div className="battery-header">
              <span className="battery-icon">⚡</span>
              <span className="battery-label">BATTERY</span>
              <span className="battery-percent" style={{ color: batteryColor }}>
                {data.battery.toFixed(0)}%
              </span>
            </div>
            <div className="battery-bar-container">
              <motion.div 
                className="battery-bar"
                initial={{ width: 0 }}
                animate={{ width: `${data.battery}%` }}
                style={{ background: batteryColor }}
                transition={{ duration: 0.3 }}
              />
              <div className="battery-segments">
                {[...Array(10)].map((_, i) => (
                  <div key={i} className="segment" />
                ))}
              </div>
            </div>
          </div>
        </div>
        
        {/* Obstacle Count */}
        <div className="telemetry-section">
          <div className="section-label">RADAR STATUS</div>
          <div className="obstacle-display">
            <div className="obstacle-count">
              <span className="obstacle-number">{data.obstacles.length}</span>
              <span className="obstacle-label">OBJECTS DETECTED</span>
            </div>
            <div className="radar-status-icon">
              <div className="radar-sweep" />
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

function CompassIndicator({ heading }) {
  const directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
  const index = Math.round(heading / 45) % 8
  
  return (
    <div className="compass-indicator">
      <span className="compass-direction">{directions[index]}</span>
    </div>
  )
}

export default Telemetry

